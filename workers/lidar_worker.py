from rplidar import RPLidar, RPLidarException
import threading, time, math, statistics, csv, io, base64
import numpy as np
from circle_fit import taubinSVD
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

# ── Constants (matching lidar_worker.py exactly) ─────────────────────────────
BAUDRATE                = 115200
MOTOR_STARTUP_DELAY     = 3
MAX_POINTS_TO_KEEP      = 10000   # reference name
SAFETY_MARGIN_DEGREES   = 5
CIRCLE_FIT_TOLERANCE_MM = 50
MIN_POINTS_FOR_CIRCLE_FIT = 10
CIRCLE_UPDATE_INTERVAL  = 5

DEFAULT_LIDAR_OFFSET_MM   = 50.0

DEFAULT_RAW_PATH = os.path.join("data", "lidar_scan")
DEFAULT_RAW_FILE = os.path.join(DEFAULT_RAW_PATH, "raw.csv")
DEFAULT_RAW_IMG  = os.path.join(DEFAULT_RAW_PATH, "processed_lidar.png")

os.makedirs(DEFAULT_RAW_PATH, exist_ok=True)


# ── Module-level helpers (copied verbatim from lidar_worker.py) ──────────────


def polar_to_cartesian(angle_deg, distance_mm):
    """
    (angle_deg, distance_mm) -> (x, y)
    """
    angle_rad = math.radians(angle_deg)
    x = distance_mm * math.cos(angle_rad)
    y = distance_mm * math.sin(angle_rad)
    return x, y


def fit_circle(points):

    """
    KASA method: 
    """
    if len(points) < 3:
        return None
    try:
        points = np.array(points)
        x = points[:, 0]
        y = points[:, 1]
        x_m = np.mean(x)
        y_m = np.mean(y)
        u = x - x_m
        v = y - y_m
        Suv  = np.sum(u * v)
        Suu  = np.sum(u ** 2)
        Svv  = np.sum(v ** 2)
        Suuv = np.sum(u ** 2 * v)
        Suvv = np.sum(u * v ** 2)
        Suuu = np.sum(u ** 3)
        Svvv = np.sum(v ** 3)
        A = np.array([[Suu, Suv], [Suv, Svv]])
        B = np.array([0.5 * (Suuu + Suvv), 0.5 * (Svvv + Suuv)])
        if np.linalg.det(A) == 0:
            return None
        uc, vc = np.linalg.solve(A, B)
        center_x = uc + x_m
        center_y = vc + y_m
        radius = np.sqrt(uc**2 + vc**2 + (Suu + Svv) / len(x))
        return center_x, center_y, radius
    except:
        return None


def distance_to_circle(point, circle_params):
    if circle_params is None:
        return 0
    center_x, center_y, radius = circle_params
    x, y = point
    dist_to_center = math.sqrt((x - center_x)**2 + (y - center_y)**2)
    deviation = abs(dist_to_center - radius)
    return deviation


def filter_by_circle(points, expected_radius_mm,
                     tolerance_mm=CIRCLE_FIT_TOLERANCE_MM):
    
    """Verbatim port from lidar_worker.py — no center_hint."""
    if len(points) < MIN_POINTS_FOR_CIRCLE_FIT:
        return points, None
    cartesian_points = [polar_to_cartesian(angle, distance)
                        for quality, angle, distance in points]
    circle_params = fit_circle(cartesian_points)
    if circle_params is None:
        return points, None
    center_x, center_y, fitted_radius = circle_params
    # Guard: if the fitted radius is more than 30% off from expected, the arc
    # is too small for a reliable fit (small-arc instability).  Skip the filter
    # rather than silently accepting a wildly wrong circle.
    if abs(fitted_radius - expected_radius_mm) / expected_radius_mm > 0.30:
        return points, None
    filtered = []
    for i, (quality, angle, distance) in enumerate(points):
        x, y = cartesian_points[i]
        deviation = distance_to_circle((x, y), circle_params)
        if deviation <= tolerance_mm:
            filtered.append((quality, angle, distance))
    return filtered, circle_params


def filter_scan_data(scan, min_angle, max_angle):
    """Verbatim port from lidar_worker.py."""
    filtered = []
    wraps_around = min_angle > max_angle
    for quality, angle, distance in scan:
        if wraps_around:
            if angle >= min_angle or angle <= max_angle:
                filtered.append((quality, angle, distance))
        else:
            if min_angle <= angle <= max_angle:
                filtered.append((quality, angle, distance))
    return filtered


def calculate_scan_angle(radius_cm, distance_cm,
                          safety_margin=SAFETY_MARGIN_DEGREES):
    """Verbatim port from lidar_worker.py."""
    half_angle_rad = math.atan(radius_cm / distance_cm)
    half_angle_deg = math.degrees(half_angle_rad)
    half_angle_with_margin = half_angle_deg + safety_margin
    center_angle = 0
    min_angle = center_angle - half_angle_with_margin
    max_angle = center_angle + half_angle_with_margin
    if min_angle < 0:
        min_angle += 360
    if max_angle < 0:
        max_angle += 360
    total_angle = half_angle_with_margin * 2
    return min_angle, max_angle, half_angle_deg, total_angle


# ── Thread class ──────────────────────────────────────────────────────────────

class LidarThread(threading.Thread):

    def __init__(self, name, lidar_offset_mm=DEFAULT_LIDAR_OFFSET_MM):
        """
        Parameters
        ----------
        name : str
            Thread name shown in log output.
        lidar_offset_mm : float
            Distance from the LiDAR sensor to the robot's Joint 1 axis,
            measured along the robot's forward direction (mm).
            The LiDAR sits between J1 and the tree, so its measurements
            are shorter than J1-frame distances by this amount.
            This value is added to the fitted trunk center y-coordinate
            in process_data() to shift all waypoints into the J1 frame.
            Can be overridden per process_data() call if needed.
        """
        super().__init__(name=name)
        self.daemon            = True
        self.running           = True
        self._lidar            = None
        self._scan_kwargs      = None
        self._done_event       = threading.Event()
        # ── offset initialised at construction time ───────────────────────
        self._lidar_offset_mm  = lidar_offset_mm

    # ------------------------------------------------------------------ #
    #  run()                                                              #
    # ------------------------------------------------------------------ #
    def run(self):
        print(f"[{self.name}] Lidar controller active...")
        if self._scan_kwargs is not None:
            self.scan(**self._scan_kwargs)

    # ------------------------------------------------------------------ #
    #  submit_scan() — public API                                         #
    # ------------------------------------------------------------------ #
    def submit_scan(self, port, radius_cm, distance_cm=None,
                    safety_margin_deg=SAFETY_MARGIN_DEGREES,
                    scan_duration_sec=30,
                    circle_tolerance_mm=CIRCLE_FIT_TOLERANCE_MM):
        """
        Start a background scan and return a threading.Event to wait on.

            worker = LidarThread("LidarWorker-1")
            done   = worker.submit_scan(port="COM4", radius_cm=15.0)
            done.wait()
            points, r_exp, plot_b64, trunk_center = worker.process_data()
        """
        if self.is_alive():
            raise RuntimeError(f"[{self.name}] Thread already running.")
        self._done_event.clear()
        self._scan_kwargs = dict(
            port               = port,
            radius_cm          = radius_cm,
            distance_cm        = distance_cm,
            safety_margin_deg  = safety_margin_deg,
            scan_duration_sec  = scan_duration_sec,
            circle_tolerance_mm= circle_tolerance_mm,
            done_event         = self._done_event,
        )
        self.start()
        return self._done_event

    # ------------------------------------------------------------------ #
    #  scan() — scan loop is a precise port of lidar_worker.py main()   #
    # ------------------------------------------------------------------ #
    def scan(self, port, radius_cm, distance_cm=None,
             safety_margin_deg=SAFETY_MARGIN_DEGREES,
             scan_duration_sec=30,
             circle_tolerance_mm=CIRCLE_FIT_TOLERANCE_MM,
             done_event=None):
        try:
            # ── Connect ──────────────────────────────────────────────────
            if not self._connect_lidar(port):
                raise RuntimeError("Failed to connect to RPLidar.")

            # ── Auto-detect distance if not provided ─────────────────────
            auto_detected = False
            if distance_cm is None:
                dist_mm, _ = self._detect_distance()
                if dist_mm is None:
                    raise RuntimeError(
                        "Auto-detection failed — no front-facing points found. "
                        "Pass distance_cm explicitly to skip auto-detect.")
                distance_cm   = dist_mm / 10
                auto_detected = True
                print(f"[{self.name}] Auto-detected distance: {distance_cm:.2f} cm")
                self._lidar.stop()
                time.sleep(0.5)

            # ── Angle window ──────────────────────────────────────────────
            min_angle, max_angle, half_angle, total_angle = calculate_scan_angle(
                radius_cm, distance_cm, safety_margin_deg)
            expected_radius_mm = radius_cm * 10

            print(f"[{self.name}] Scan window: {min_angle:.2f}° → {max_angle:.2f}°  "
                  f"(half {half_angle:.2f}° + {safety_margin_deg}° margin)")

            # ── Scan variables — names match lidar_worker.py exactly ────
            data_buffer            = []
            total_points_collected = 0
            points_rejected_angle  = 0
            points_rejected_circle = 0
            points_rejected_buffer = 0     # single counter, same as reference
            scan_count             = 0
            circle_params          = None
            last_circle_update     = 0
            start_time             = time.time()
            scan_end_time          = start_time + scan_duration_sec

            print(f"[{self.name}] Scan started — duration {scan_duration_sec}s  "
                  f"buffer limit {MAX_POINTS_TO_KEEP} pts")

            # ── Main scan loop — direct port of lidar_worker.py ─────────
            for scan in self._lidar.iter_scans(max_buf_meas=50000):
                current_time = time.time()
                elapsed_time = current_time - start_time

                if elapsed_time >= scan_duration_sec:
                    break

                # LAYER 1: Filter by angle
                points_before_angle_filter = len(scan)
                filtered_by_angle          = filter_scan_data(scan, min_angle, max_angle)
                points_after_angle_filter  = len(filtered_by_angle)
                points_rejected_angle += (points_before_angle_filter
                                          - points_after_angle_filter)

                # Update circle fit periodically
                if (scan_count - last_circle_update >= CIRCLE_UPDATE_INTERVAL
                        and len(data_buffer) >= MIN_POINTS_FOR_CIRCLE_FIT):
                    _, circle_params = filter_by_circle(
                        data_buffer, expected_radius_mm, circle_tolerance_mm)
                    last_circle_update = scan_count
                    if circle_params and scan_count == CIRCLE_UPDATE_INTERVAL:
                        cx, cy, r = circle_params
                        print(f"\n[{self.name}] Circle fitted: "
                              f"center=({cx:.1f}, {cy:.1f}) mm  "
                              f"r={r:.1f} mm  (expected {expected_radius_mm:.1f} mm)")

                # LAYER 2: Filter by circle
                if circle_params is not None:
                    filtered_by_circle, _ = filter_by_circle(
                        filtered_by_angle, expected_radius_mm, circle_tolerance_mm)
                    points_rejected_circle += (points_after_angle_filter
                                               - len(filtered_by_circle))
                    filtered_scan = filtered_by_circle
                else:
                    filtered_scan = filtered_by_angle

                # Quality-ranked buffer
                for quality, angle, distance in filtered_scan:
                    total_points_collected += 1
                    if len(data_buffer) < MAX_POINTS_TO_KEEP:
                        data_buffer.append((quality, angle, distance))
                    else:
                        worst_quality = min(data_buffer, key=lambda p: p[0])[0]
                        if quality > worst_quality:                                 # if quality_new > min(buffer_qualities)
                            worst_point = min(data_buffer, key=lambda p: p[0])
                            data_buffer.remove(worst_point)                         # remove point with min quality from buffer
                            data_buffer.append((quality, angle, distance))          # insert new point
                            points_rejected_buffer += 1
                        else:
                            points_rejected_buffer += 1

                # Progress
                circle_status = "Y" if circle_params else "N"
                print(f"\r[{self.name}] "
                      f"#{scan_count:4d} | "
                      f"angle {points_after_angle_filter:4d}/{points_before_angle_filter:4d} | "
                      f"circle({circle_status}) {len(filtered_scan):4d} | "
                      f"buf {len(data_buffer):5d}/{MAX_POINTS_TO_KEEP} | "
                      f"{elapsed_time / scan_duration_sec * 100:5.1f}%",
                      end='', flush=True)

                scan_count += 1

            print()  # newline after progress

            # ── Save CSV ──────────────────────────────────────────────────
            data_buffer.sort(key=lambda p: p[1])
            os.makedirs(DEFAULT_RAW_PATH, exist_ok=True)
            with open(DEFAULT_RAW_FILE, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['angle', 'distance', 'quality'])
                for quality, angle, distance in data_buffer:
                    w.writerow([f'{angle:.2f}', f'{distance:.2f}', quality])

            print(f"[{self.name}] Saved {len(data_buffer)} points → {DEFAULT_RAW_FILE}")
            print(f"[{self.name}] Stats — "
                  f"angle rejected: {points_rejected_angle}  "
                  f"circle rejected: {points_rejected_circle}  "
                  f"buffer: {points_rejected_buffer}")

        except RPLidarException as e:
            print(f"\n[{self.name}] RPLidar error: {e}")
        except Exception:
            import traceback; traceback.print_exc()
        finally:
            self._cleanup()
            if done_event:
                done_event.set()

    # ------------------------------------------------------------------ #
    #  process_data() — unchanged from original worker                   #
    # ------------------------------------------------------------------ #
    def process_data(self, path=DEFAULT_RAW_FILE, lidar_offset_mm=None):
        """
        Process a raw LiDAR CSV and return contour waypoints in J1 frame.
 
        Parameters
        ----------
        path : str
            Path to the raw CSV written by scan().
        lidar_offset_mm : float or None
            Override the instance-level offset for this call only.
            If None, uses self._lidar_offset_mm set at __init__ time.
 
        Returns
        -------
        points       : list of dict  {"x": mm, "y": mm, "z": mm}
        r_exp        : float  expanded scan radius in mm
        plot_b64     : str    base64-encoded PNG
        trunk_center : dict   {"cx_mm": float, "cy_mm": float}  in J1 frame
        """
        # Resolve which offset to use — per-call override or instance default
        offset_mm = lidar_offset_mm if lidar_offset_mm is not None \
                    else self._lidar_offset_mm
 
        print(f"[{self.name}] Processing: {path}")
        if offset_mm != 0.0:
            print(f"[{self.name}] LiDAR→J1 offset: {offset_mm:.1f} mm  "
                  f"(trunk center shifted into J1 frame)")
 
        qualities, angles, distances = self._load_csv(path)
        if not angles:
            print(f"[{self.name}] No valid data found.")
            return None, None, None, None
 
        # Distance pre-filter (MAD-based, removes background noise)
        if len(distances) >= 10:
            import statistics as _st
            med = _st.median(distances)
            mad = _st.median([abs(d - med) for d in distances])
            mad = max(mad, 1.0)
            lo, hi = med - 3.0 * mad, med + 3.0 * mad
            mask      = [lo <= d <= hi for d in distances]
            qualities = [q for q, m in zip(qualities,  mask) if m]
            angles    = [a for a, m in zip(angles,     mask) if m]
            distances = [d for d, m in zip(distances,  mask) if m]
            print(f"[{self.name}] Distance pre-filter: kept {len(distances)} / "
                  f"{len(mask)} pts  (median={med:.0f}mm  MAD={mad:.0f}mm  "
                  f"window={lo:.0f}-{hi:.0f}mm)")
            if not distances:
                print(f"[{self.name}] All points removed by pre-filter.")
                return None, None, None, None
 
        def norm(a):
            while a >  180: a -= 360
            while a <= -180: a += 360
            return a
        norm_ang = [norm(a) for a in angles]
 
        n   = max(5, len(distances) // 10)
        top = sorted(zip(distances, norm_ang))[:n]
        tw  = sum(1/d for d, _ in top)
        ca  = sum((1/d)*a for d, a in top) / tw
 
        corrected, xs, ys = [], [], []
        for na, dist in zip(norm_ang, distances):
            c = na - ca
            corrected.append(c)
            rad = math.radians(c)
            xs.append(dist * math.sin(rad))
            ys.append(dist * math.cos(rad))
 
        idx = sorted(range(len(corrected)), key=lambda i: corrected[i])
        xs  = [xs[i] for i in idx]
        ys  = [ys[i] for i in idx]
 
        y_offset_mm = 50.0
        ys_shifted  = [y + y_offset_mm for y in ys]
 
        pts = np.array(list(zip(xs, ys_shifted)))
        xc, yc, r, sigma = taubinSVD(pts)
        print(f"[{self.name}] LiDAR frame — center ({xc:.2f}, {yc:.2f}) mm | "
              f"r={r:.2f} mm | σ={sigma:.3f}")
 
        # ── Apply LiDAR→J1 offset ─────────────────────────────────────────
        # The LiDAR is closer to the tree than J1 by offset_mm.
        # Adding offset_mm to yc converts the trunk center from the LiDAR
        # frame to the J1 frame.  The x-axis is not affected.
        yc_j1 = yc - offset_mm   # ← use resolved offset_mm, not raw parameter
        cx_j1 = xc
 
        print(f"[{self.name}] J1 frame    — center ({cx_j1:.2f}, {yc_j1:.2f}) mm  "
              f"(+{offset_mm:.1f} mm on Y)")
 
        # ── Generate waypoints around J1-frame trunk center ───────────────
        expand_mm = 100.0   # gap from trunk surface to antenna tip
        n_points  = 36
        r_exp     = r + expand_mm
 
        sample_angles = np.array([
            math.radians(180 - i * (360.0 / n_points))
            for i in range(n_points)
        ])
 
        # Use J1-frame center (cx_j1, yc_j1) 
        sample_x = cx_j1 + r_exp * np.cos(sample_angles)
        sample_y = yc_j1 + r_exp * np.sin(sample_angles)
        sample_z = np.full(n_points, 10)
 
        print(f"[{self.name}] Expanded r={r_exp:.2f} mm | "
              f"center ({cx_j1:.2f}, {yc_j1:.2f}) mm (J1 frame) | {n_points} pts CCW")
 
        # Pass J1-frame center to _plot so the visualisation is correct
        plot_b64 = self._plot(pts, xc, yc, r, sigma,
                              r_exp, cx_j1, yc_j1, sample_x, sample_y)
 
        points = [
            {"x": float(sample_x[i]), "y": float(sample_y[i]), "z": float(sample_z[i])}
            for i in range(n_points)
        ]
 
        # Trunk center in J1 frame — passed to Arduino via SET_TRUNK at /init
        trunk_center = {
            "cx_mm": float(cx_j1),
            "cy_mm": float(yc_j1),
        }
 
        return points, r_exp, plot_b64, trunk_center

    # ------------------------------------------------------------------ #
    #  stop()                                                             #
    # ------------------------------------------------------------------ #
    def stop(self):
        self.running = False

    # ------------------------------------------------------------------ #                      #
    # ------------------------------------------------------------------ #
    # def _plot(self, points, xc, yc, r, sigma,
    #           r_exp, cx_j1, yc_j1, sample_x, sample_y):
    #     xs = points[:, 0] - xc
    #     ys = points[:, 1]

    #     fig, ax = plt.subplots(figsize=(7, 7))
    #     ax.scatter(xs, ys, s=6, color='steelblue', alpha=0.55, label='Scan points')
    #     ax.add_patch(plt.Circle((0, yc), r, color='black', fill=False,
    #                             linewidth=1.8, label=f'Fit  r={r:.1f} mm'))
    #     ax.plot(0, yc, 'k+', markersize=10, label='Fit center')

    #     ax.add_patch(plt.Circle((cx_j1 - xc, yc_j1), r_exp, color='green', fill=False,
    #                             linewidth=1.8, linestyle='--',
    #                             label=f'Expanded  r={r_exp:.1f} mm'))
    #     ax.plot(cx_j1 - xc, yc_j1, 'g+', markersize=10, label=f'Shifted center (y={yc_j1:.1f})')
    #     ax.scatter(sample_x, sample_y, s=55, color='red', edgecolors='black',
    #                zorder=5, label=f'{len(sample_x)}-pt contour (0° CCW)')
    #     ax.scatter(sample_x[0], sample_y[0], s=120, color='yellow',
    #                edgecolors='black', zorder=6, label='Start (0°)')
    #     ax.set_aspect('equal', adjustable='datalim')
    #     ax.axvline(0, color='grey', linewidth=0.6, alpha=0.4)
    #     ax.set_title(f'Fit r={r:.2f} mm  →  Expanded r={r_exp:.2f} mm   σ={sigma:.3f} mm')
    #     ax.set_xlabel('X (mm)');  ax.set_ylabel('Y (mm)')
    #     ax.legend(fontsize=8, loc='upper right')
    #     ax.grid(True, linestyle=':', alpha=0.5)
    #     plt.tight_layout()

    #     buf = io.BytesIO()
    #     fig.savefig(buf, format='png', dpi=120)
    #     fig.savefig(DEFAULT_RAW_IMG, format='png', dpi=120)
    #     plt.close(fig)
    #     buf.seek(0)
    #     return base64.b64encode(buf.read()).decode('utf-8')
    def _plot(self, points, xc, yc, r, sigma,
              r_exp, cx_j1, yc_j1, sample_x, sample_y, offset_mm=0.0):
        """
        Render the LiDAR scan result as a 2D Matplotlib figure and return
        it as a base64-encoded PNG string.

        Coordinate system used in this plot (display frame)
        ----------------------------------------------------
        All raw scan points are stored in the LiDAR sensor frame, where
        the sensor itself is the origin (distance = 0).  Before plotting,
        the points are re-centerd so that the fitted trunk circle center
        lands at x = 0 in the display.  The y-axis points toward the trunk.

        The y_offset_mm = 50 mm shift that was applied to ys in process_data()
        means:
          • The LiDAR sensor sits at  (−xc,  y_offset_mm)  in display coords.
            Because y_offset_mm = 50 mm, the sensor is always 50 mm above the
            plot origin on the y-axis.
          • J1 sits offset_mm behind the LiDAR along y, so its display position
            is  (−xc,  y_offset_mm − offset_mm).
            When offset_mm == 50 mm (the default), J1 lands at y = 0 — the plot
            origin — which is the most intuitive layout.

        Parameters
        ----------
        points     : np.ndarray  shape (n,2) - raw (x, y) after y_offset shift
        xc, yc     : float  TaubinSVD circle center in LiDAR frame
        r          : float  fitted trunk radius (mm)
        sigma      : float  TaubinSVD residual (mm)
        r_exp      : float  expanded scan radius = r + 100 mm (antenna gap)
        cx_j1      : float  trunk center x in J1 frame  (= xc, lateral unchanged)
        yc_j1      : float  trunk center y in J1 frame  (= yc + offset_mm)
        sample_x   : np.ndarray  waypoint x-coords in J1 frame (mm)
        sample_y   : np.ndarray  waypoint y-coords in J1 frame (mm)
        offset_mm  : float  LiDAR-to-J1 distance (mm), used to place markers
        """

        # ── 1. Re-center scan points to x=0 ──
        # pts[:,0] contains x-coords in the LiDAR frame, where xc is the circle
        # fit x-center.  Subtracting xc shifts the trunk to x=0 in display space.
        # pts[:,1] (y) is left as-is — it already contains the y_offset_mm shift
        # that places the LiDAR sensor at y = y_offset_mm (= 50 mm) and
        # implicitly places J1 at y = y_offset_mm − offset_mm.
        xs = points[:, 0] - xc   # x in display frame
        ys = points[:, 1]         # y in display frame (= LiDAR-frame y + 50 mm)

        fig, ax = plt.subplots(figsize=(7, 7))

        # ── 2. Raw scan points ────────────────────────────────────────────────────
        # Every (angle, distance) reading that survived the angle-window and
        # quality-ranked buffer is plotted here.  The cluster of points forms the
        # visible arc of the trunk surface as seen from the LiDAR.
        ax.scatter(xs, ys, s=6, color='steelblue', alpha=0.55, label='Scan points')

        # ── 3. LiDAR-frame trunk circle fit ──────────────────────────────────────
        # The TaubinSVD fit found the circle that best matches the scan arc.
        # center is at (0, yc) in display coords — x=0 because we re-centerd by xc.
        # This is the trunk boundary as perceived by the LiDAR sensor.
        ax.add_patch(plt.Circle(
            (0, yc), r,
            color='black', fill=False, linewidth=1.8,
            label=f'Trunk fit (LiDAR frame)  r={r:.1f} mm'
        ))

        # ── 4. LiDAR-frame trunk center marker ───────────────────────────────────
        # The black cross marks where the TaubinSVD estimated the trunk center to
        # be, expressed in the LiDAR sensor's own coordinate frame.  This point
        # is NOT directly usable by the robot IK — the J1-frame center (step 7) is.
        ax.plot(0, yc, 'k+', markersize=12, markeredgewidth=2,
                label=f'Trunk center (LiDAR frame)  y={yc:.1f} mm')

        # ── 5. J1-frame expanded scan circle ─────────────────────────────────────
        # The robot arm positions the antenna on a circle of radius r_exp =
        # r + 100 mm (100 mm = antenna–trunk-surface gap), centerd on the trunk
        # center expressed in the J1 frame.  This is the actual trajectory the
        # arm will follow.
        # cx_j1 − xc applies the same re-centring used for the scan points so
        # this circle aligns with the point cloud in the display.
        ax.add_patch(plt.Circle(
            (cx_j1 - xc, yc_j1), r_exp,
            color='green', fill=False, linewidth=1.8, linestyle='--',
            label=f'Scan trajectory (J1 frame)  r_exp={r_exp:.1f} mm'
        ))

        # ── 6. J1-frame trunk center marker ──────────────────────────────────────
        # The green cross is the trunk center in the robot's J1 coordinate frame,
        # obtained by adding offset_mm to the LiDAR-frame yc.  This is the origin
        # around which all 36 waypoints are generated, and the value sent to the
        # Arduino as SET_TRUNK.
        ax.plot(cx_j1 - xc, yc_j1, 'g+', markersize=12, markeredgewidth=2,
                label=f'Trunk center (J1 frame)  y={yc_j1:.1f} mm')

        # ── 7. Waypoints ──────────────────────────────────────────────────────────
        # The 36 red dots are the antenna-tip positions the arm will visit,
        # evenly spaced around the expanded circle.  sample_x and sample_y are
        # in J1 frame, so the same re-centring (− xc) is applied to align them
        # with the scan point cloud.
        ax.scatter(sample_x - xc, sample_y, s=55, color='red', edgecolors='black',
                   zorder=5, label=f'{len(sample_x)}-pt waypoints (CCW)')

        # ── 8. Start waypoint ─────────────────────────────────────────────────────
        # Waypoint 0 (θ = 180°) is the nearest point to the robot — the arm starts
        # directly in front of the trunk at minimum extension.
        ax.scatter(sample_x[0] - xc, sample_y[0], s=140, color='yellow',
                   edgecolors='black', zorder=6, label='Start waypoint (θ=180°)')

        # ── 9. LiDAR sensor position ──────────────────────────────────────────────
        # The triangle marks the physical location of the LiDAR sensor in the
        # display frame.  In process_data(), all y-coords received a +50 mm shift
        # (y_offset_mm), which places the sensor at y = +50 mm.  The x-position
        # is −xc (the same re-centring applied to the scan points).
        y_offset_mm = 50.0   # must match the value used in process_data()
        lidar_display_x = -xc
        lidar_display_y = y_offset_mm
        ax.plot(lidar_display_x, lidar_display_y,
                'r^', markersize=12, markeredgewidth=1.5,
                label=f'LiDAR sensor  ({lidar_display_x:.1f}, {lidar_display_y:.0f}) mm')

        # ── 10. Joint 1 axis position ─────────────────────────────────────────────
        # J1 is offset_mm behind the LiDAR along the forward (y) direction.
        # In display coords:  y_J1 = y_LiDAR − offset_mm = 50 − offset_mm.
        # When offset_mm = 50 mm, J1 lands exactly at y = 0 — the display origin.
        # The square symbol distinguishes it from the LiDAR triangle.
        j1_display_x = -xc
        j1_display_y = y_offset_mm - offset_mm
        ax.plot(j1_display_x, j1_display_y,
                'bs', markersize=12, markeredgewidth=1.5,
                label=f'J1 axis  ({j1_display_x:.1f}, {j1_display_y:.0f}) mm')

        # ── 11. Vertical axis reference line ──────────────────────────────────────
        # The grey vertical line at x = 0 marks the forward axis of the LiDAR
        # sensor (after re-centring).  Points to the left of it are on the
        # left side of the trunk; points to the right are on the right side.
        ax.axvline(0, color='grey', linewidth=0.6, alpha=0.4,
                   label='LiDAR forward axis')

        # ── 12. Axes, title, and save ─────────────────────────────────────────────
        # equal aspect ratio preserves the circular geometry — do not remove.
        ax.set_aspect('equal', adjustable='datalim')
        ax.set_title(
            f'LiDAR scan result\n'
            f'Trunk fit: r={r:.1f} mm  |  Scan trajectory: r_exp={r_exp:.1f} mm  '
            f'|  σ={sigma:.3f} mm  |  J1 offset={offset_mm:.0f} mm'
        )
        ax.set_xlabel('X (mm) — lateral')
        ax.set_ylabel('Y (mm) — forward (toward trunk)')
        ax.legend(fontsize=8, loc='upper right')
        ax.grid(True, linestyle=':', alpha=0.5)
        plt.tight_layout()

        # Save to file for inspection and encode for API response
        buf = io.BytesIO()
        fig.savefig(buf, format='png', dpi=120)
        fig.savefig(DEFAULT_RAW_IMG, format='png', dpi=120)
        plt.close(fig)
        buf.seek(0)
        return base64.b64encode(buf.read()).decode('utf-8')

    # ------------------------------------------------------------------ #
    #  Private helpers                                                    #
    # ------------------------------------------------------------------ #
    def _connect_lidar(self, port, max_retries=3):
        for attempt in range(max_retries):
            try:
                if self._lidar:
                    try:
                        self._lidar.stop()
                        self._lidar.stop_motor()
                        self._lidar.disconnect()
                    except Exception:
                        pass
                    self._lidar = None
                    time.sleep(1.0)

                self._lidar = RPLidar(port, baudrate=BAUDRATE, timeout=3)

                if hasattr(self._lidar, '_serial') and self._lidar._serial:
                    self._lidar._serial.reset_input_buffer()
                    self._lidar._serial.reset_output_buffer()

                # get_info() first — matches reference connection order
                info = self._lidar.get_info()
                print(f"[{self.name}] Connected (attempt {attempt+1}): {info}")
                self._lidar.start_motor()
                time.sleep(MOTOR_STARTUP_DELAY)
                return True

            except RPLidarException as e:
                print(f"[{self.name}] Connect attempt {attempt+1}/{max_retries} failed: {e}")
                time.sleep(2.0)
        return False

    def _detect_distance(self, num_scans=5, min_quality=10, front_angle_tolerance=10):
        def is_front_facing(angle):
            return angle <= front_angle_tolerance or angle >= (360 - front_angle_tolerance)

        front_dist, front_angles, scan_count = [], [], 0
        for scan in self._lidar.iter_scans(max_buf_meas=5000):
            scan_count += 1
            valid = 0
            for quality, angle, distance in scan:
                if quality >= min_quality and is_front_facing(angle):
                    front_dist.append(distance)
                    front_angles.append(angle)
                    valid += 1
            print(f"  Scan {scan_count}/{num_scans}: {valid} valid front-facing points")
            if scan_count >= num_scans:
                break

        if not front_dist:
            return None, None

        avg_dist   = statistics.mean(front_dist)
        normalized = [a - 360 if a > 180 else a for a in front_angles]
        avg_angle  = statistics.mean(normalized)
        if avg_angle < 0:
            avg_angle += 360
        return avg_dist, avg_angle

    def _load_csv(self, path):
        """Unchanged from original worker — handles both column orderings."""
        qualities, angles, distances = [], [], []
        with open(path, 'r', encoding='latin-1') as f:
            reader = csv.reader(f)
            header = None
            for row in reader:
                if not row or row[0].startswith('#') or row[0].strip() == '':
                    continue
                try:
                    float(row[0])
                except ValueError:
                    header = [col.strip().lower() for col in row]
                    continue
                try:
                    if header and 'angle' in header:
                        ai = header.index('angle')
                        di = header.index('distance')
                        qi = header.index('quality')
                        angles.append(float(row[ai]))
                        distances.append(float(row[di]))
                        qualities.append(int(float(row[qi])))
                    else:
                        qualities.append(int(float(row[0])))
                        angles.append(float(row[1]))
                        distances.append(float(row[2]))
                except (ValueError, IndexError):
                    continue
        return qualities, angles, distances

    def _cleanup(self):
        if self._lidar:
            try:
                self._lidar.stop()
                self._lidar.stop_motor()
                self._lidar.disconnect()
            except Exception:
                pass


# ── Standalone usage ──────────────────────────────────────────────────────────
if __name__ == "__main__":
    import sys

    def run_scan():
        PORT      = "COM4"    # Linux: "/dev/ttyUSB0"
        RADIUS_CM = 15.0

        worker = LidarThread("LidarWorker-1")
        done   = worker.submit_scan(
            port                = PORT,
            radius_cm           = RADIUS_CM,
            distance_cm         = None,      # None → auto-detect
            safety_margin_deg   = SAFETY_MARGIN_DEGREES,
            scan_duration_sec   = 30,
            circle_tolerance_mm = CIRCLE_FIT_TOLERANCE_MM,
        )
        print("Scanning in background — waiting...")
        done.wait()
        print("Scan complete.")

        points, r_exp, plot_b64, trunk_center = worker.process_data()
        if r_exp is not None:
            print(f"r_exp={r_exp:.2f} mm  |  {len(points)} contour points")
            with open("result_plot.png", "wb") as f:
                f.write(base64.b64decode(plot_b64))
            print("Plot saved → result_plot.png")

    def run_process_only(csv_path):
        worker = LidarThread("LidarWorker-2")
        points, r_exp, plot_b64, trunk_center = worker.process_data(csv_path)
        if r_exp is not None:
            print(f"r_exp={r_exp:.2f} mm  |  {len(points)} contour points")
            with open("result_plot.png", "wb") as f:
                f.write(base64.b64decode(plot_b64))
            print("Plot saved → result_plot.png")

    if len(sys.argv) > 1:
        run_process_only(sys.argv[1])
    else:
        run_scan()