from rplidar import RPLidar, RPLidarException
import threading, time, math, statistics, csv, io, base64
import numpy as np
from circle_fit import taubinSVD
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

# ── Constants ─────────────────────────────────────────────────────────────────
BAUDRATE             = 115200
MOTOR_STARTUP_DELAY  = 3
MAX_POINTS           = 10000
SAFETY_MARGIN_DEG    = 5
CIRCLE_FIT_TOLERANCE = 50   # mm
MIN_POINTS_CIRCLE    = 10
CIRCLE_UPDATE_EVERY  = 5    # scans

DEFAULT_RAW_PATH = os.path.join("data", "lidar_scan")
DEFAULT_RAW_FILE = os.path.join(DEFAULT_RAW_PATH, "raw.csv")
DEFAULT_RAW_IMG  = os.path.join(DEFAULT_RAW_PATH, "processed_lidar.png")

os.makedirs(DEFAULT_RAW_PATH, exist_ok=True)


# ── Standalone helpers (ported directly from lidar_scanner.py) ────────────────

def polar_to_cartesian(angle_deg, distance_mm):
    rad = math.radians(angle_deg)
    return distance_mm * math.cos(rad), distance_mm * math.sin(rad)


def fit_circle(points):
    if len(points) < 3:
        return None
    try:
        pts  = np.array(points)
        x, y = pts[:, 0], pts[:, 1]
        x_m, y_m = np.mean(x), np.mean(y)
        u, v = x - x_m, y - y_m
        Suv  = np.sum(u * v)
        Suu  = np.sum(u ** 2);  Svv  = np.sum(v ** 2)
        Suuv = np.sum(u ** 2 * v); Suvv = np.sum(u * v ** 2)
        Suuu = np.sum(u ** 3);  Svvv = np.sum(v ** 3)
        A = np.array([[Suu, Suv], [Suv, Svv]])
        B = np.array([0.5 * (Suuu + Suvv), 0.5 * (Svvv + Suuv)])
        if np.linalg.det(A) == 0:
            return None
        uc, vc = np.linalg.solve(A, B)
        return uc + x_m, vc + y_m, np.sqrt(uc**2 + vc**2 + (Suu + Svv) / len(x))
    except Exception:
        return None


def distance_to_circle(point, circle_params):
    if circle_params is None:
        return 0
    cx, cy, r = circle_params
    x, y = point
    return abs(math.sqrt((x - cx) ** 2 + (y - cy) ** 2) - r)


def filter_by_circle(points, expected_radius_mm,
                     tolerance_mm=CIRCLE_FIT_TOLERANCE,
                     min_points=MIN_POINTS_CIRCLE):
    """Ported directly from lidar_scanner.py — no changes."""
    if len(points) < min_points:
        return points, None

    cart         = [polar_to_cartesian(a, d) for _, a, d in points]
    circle_params = fit_circle(cart)

    if circle_params is None:
        return points, None

    filtered = []
    for i, pt in enumerate(points):
        if distance_to_circle(cart[i], circle_params) <= tolerance_mm:
            filtered.append(pt)

    return filtered, circle_params


def filter_scan_data(scan, min_angle, max_angle):
    """Ported directly from lidar_scanner.py — no changes."""
    filtered     = []
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
                          safety_margin=SAFETY_MARGIN_DEG):
    """Ported directly from lidar_scanner.py — no changes."""
    half_deg   = math.degrees(math.atan(radius_cm / distance_cm))
    half_total = half_deg + safety_margin
    min_angle  = -half_total
    max_angle  =  half_total
    if min_angle < 0:
        min_angle += 360
    if max_angle < 0:
        max_angle += 360
    return min_angle, max_angle, half_deg, half_total * 2


# ── Thread class ──────────────────────────────────────────────────────────────

class LidarThread(threading.Thread):

    def __init__(self, name):
        super().__init__(name=name)
        self.daemon       = True
        self.running      = True
        self._lidar       = None
        self._scan_kwargs = None
        self._done_event  = threading.Event()

    # ------------------------------------------------------------------ #
    #  run() — thread entry point                                         #
    # ------------------------------------------------------------------ #
    def run(self):
        print(f"[{self.name}] Lidar controller active...")
        if self._scan_kwargs is not None:
            self.scan(**self._scan_kwargs)

    # ------------------------------------------------------------------ #
    #  submit_scan() — public API                                         #
    # ------------------------------------------------------------------ #
    def submit_scan(self, port, radius_cm, distance_cm=None,
                    safety_margin_deg=SAFETY_MARGIN_DEG,
                    scan_duration_sec=30,
                    circle_tolerance_mm=CIRCLE_FIT_TOLERANCE):
        """
        Configure and start a background scan.
        Returns a threading.Event — call .wait() to block until done.

            worker = LidarThread("LidarWorker-1")
            done   = worker.submit_scan(port="COM4", radius_cm=15.0)
            done.wait()
            points, r_exp, plot_b64 = worker.process_data()
        """
        if self.is_alive():
            raise RuntimeError(f"[{self.name}] Already running.")
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
    #  scan() — scan logic ported from lidar_scanner.py main()           #
    # ------------------------------------------------------------------ #
    def scan(self, port, radius_cm, distance_cm=None,
             safety_margin_deg=SAFETY_MARGIN_DEG,
             scan_duration_sec=30,
             circle_tolerance_mm=CIRCLE_FIT_TOLERANCE,
             done_event=None):
        try:
            radius_mm = radius_cm * 10

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

            # ── Angle window (same math as lidar_scanner.py) ─────────────
            min_angle, max_angle, half_angle, total_angle = calculate_scan_angle(
                radius_cm, distance_cm, safety_margin_deg)
            print(f"[{self.name}] Scan window: {min_angle:.2f}° → {max_angle:.2f}°")

            # ── Scan variables (same names as lidar_scanner.py) ───────────
            data_buffer            = []
            total_points_collected = 0
            points_rejected_angle  = 0
            points_rejected_circle = 0
            points_rejected_buffer = 0
            scan_count             = 0
            circle_params          = None
            last_circle_update     = 0
            start_time             = time.time()

            # ── Main scan loop (direct port of lidar_scanner.py) ─────────
            for scan in self._lidar.iter_scans(max_buf_meas=50000):
                elapsed_time = time.time() - start_time
                if elapsed_time >= scan_duration_sec:
                    break

                # LAYER 1: angle filter
                n_before          = len(scan)
                filtered_by_angle = filter_scan_data(scan, min_angle, max_angle)
                n_after           = len(filtered_by_angle)
                points_rejected_angle += (n_before - n_after)

                # Update circle fit periodically
                if (scan_count - last_circle_update >= CIRCLE_UPDATE_EVERY
                        and len(data_buffer) >= MIN_POINTS_CIRCLE):
                    _, circle_params = filter_by_circle(
                        data_buffer, radius_mm, circle_tolerance_mm)
                    last_circle_update = scan_count
                    if circle_params:
                        cx, cy, r = circle_params
                        print(f"[{self.name}] Circle (scan {scan_count}): "
                              f"centre=({cx:.1f}, {cy:.1f}) mm  r={r:.1f} mm  "
                              f"(expected {radius_mm:.1f} mm)")

                # LAYER 2: circle filter
                if circle_params is not None:
                    filtered_by_circle, _ = filter_by_circle(
                        filtered_by_angle, radius_mm, circle_tolerance_mm)
                    points_rejected_circle += (n_after - len(filtered_by_circle))
                    filtered_scan = filtered_by_circle
                else:
                    filtered_scan = filtered_by_angle

                # Quality-ranked buffer
                for quality, angle, distance in filtered_scan:
                    total_points_collected += 1
                    if len(data_buffer) < MAX_POINTS:
                        data_buffer.append((quality, angle, distance))
                    else:
                        worst_pt = min(data_buffer, key=lambda p: p[0])
                        if quality > worst_pt[0]:
                            data_buffer.remove(worst_pt)
                            data_buffer.append((quality, angle, distance))
                            points_rejected_buffer += 1
                        else:
                            points_rejected_buffer += 1

                # Progress
                circle_status = "Y" if circle_params else "N"
                print(f"\r[{self.name}] "
                      f"scan {scan_count:4d} | "
                      f"angle {n_after:4d}/{n_before:4d} | "
                      f"circle({circle_status}) {len(filtered_scan):4d} | "
                      f"buf {len(data_buffer):5d}/{MAX_POINTS} | "
                      f"{elapsed_time/scan_duration_sec*100:5.1f}%",
                      end='', flush=True)

                scan_count += 1

            print()  # newline after progress bar

            # ── Save CSV (same column order as lidar_scanner.py) ─────────
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
    def process_data(self, path=DEFAULT_RAW_FILE):
        print(f"[{self.name}] Processing: {path}")

        qualities, angles, distances = self._load_csv(path)
        if not angles:
            print(f"[{self.name}] No valid data found.")
            return None, None, None

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
        print(f"[{self.name}] Centre ({xc:.2f}, {yc:.2f}) mm | r={r:.2f} mm | σ={sigma:.3f}")

        expand_mm = 100.0
        n_points  = 36
        r_exp     = r + expand_mm

        sample_angles = np.array([
            math.radians(180 - i * (360.0 / n_points))
            for i in range(n_points)
        ])
        cx_s     = 0.0
        sample_x = cx_s + r_exp * np.cos(sample_angles)
        sample_y = yc   + r_exp * np.sin(sample_angles)
        sample_z = np.full(n_points, 10)

        print(f"[{self.name}] Expanded r={r_exp:.2f} mm | centre (0, {yc:.2f}) | {n_points} pts CCW")

        plot_b64 = self._plot(pts, xc, yc, r, sigma,
                              r_exp, cx_s, yc, sample_x, sample_y)
        points = [
            {"x": float(sample_x[i]), "y": float(sample_y[i]), "z": float(sample_z[i])}
            for i in range(n_points)
        ]
        return points, r_exp, plot_b64

    # ------------------------------------------------------------------ #
    #  stop()                                                             #
    # ------------------------------------------------------------------ #
    def stop(self):
        self.running = False

    # ------------------------------------------------------------------ #
    #  _plot() — unchanged from original worker                          #
    # ------------------------------------------------------------------ #
    def _plot(self, points, xc, yc, r, sigma,
              r_exp, cx_s, yc_s, sample_x, sample_y):
        xs = points[:, 0] - xc
        ys = points[:, 1]

        fig, ax = plt.subplots(figsize=(7, 7))
        ax.scatter(xs, ys, s=6, color='steelblue', alpha=0.55, label='Scan points')
        ax.add_patch(plt.Circle((0, yc), r, color='black', fill=False,
                                linewidth=1.8, label=f'Fit  r={r:.1f} mm'))
        ax.plot(0, yc, 'k+', markersize=10, label='Fit centre')
        ax.add_patch(plt.Circle((cx_s, yc_s), r_exp, color='green', fill=False,
                                linewidth=1.8, linestyle='--',
                                label=f'Expanded  r={r_exp:.1f} mm'))
        ax.plot(cx_s, yc_s, 'g+', markersize=10, label=f'Shifted centre (y={yc_s:.1f})')
        ax.scatter(sample_x, sample_y, s=55, color='red', edgecolors='black',
                   zorder=5, label=f'{len(sample_x)}-pt contour (0° CCW)')
        ax.scatter(sample_x[0], sample_y[0], s=120, color='yellow',
                   edgecolors='black', zorder=6, label='Start (0°)')
        ax.set_aspect('equal', adjustable='datalim')
        ax.axvline(0, color='grey', linewidth=0.6, alpha=0.4)
        ax.set_title(f'Fit r={r:.2f} mm  →  Expanded r={r_exp:.2f} mm   σ={sigma:.3f} mm')
        ax.set_xlabel('X (mm)');  ax.set_ylabel('Y (mm)')
        ax.legend(fontsize=8, loc='upper right')
        ax.grid(True, linestyle=':', alpha=0.5)
        plt.tight_layout()

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

                # get_info first (Bug 2 fix: avoids descriptor mismatch)
                info = self._lidar.get_info()
                print(f"[{self.name}] Connected (attempt {attempt+1}): {info}")
                self._lidar.start_motor()
                time.sleep(MOTOR_STARTUP_DELAY)
                return True

            except RPLidarException as e:
                print(f"[{self.name}] Connect attempt {attempt+1}/{max_retries} failed: {e}")
                time.sleep(2.0)
        return False

    def _detect_distance(self, num_scans=5, min_quality=10, front_angle_tolerance=1):
        def is_front(angle):
            return angle <= front_angle_tolerance or angle >= (360 - front_angle_tolerance)

        front_dist, front_angles, n = [], [], 0
        for scan in self._lidar.iter_scans(max_buf_meas=5000):
            n += 1
            valid = 0
            for quality, angle, distance in scan:
                if quality >= min_quality and is_front(angle):
                    front_dist.append(distance)
                    front_angles.append(angle)
                    valid += 1
            print(f"  Scan {n}/{num_scans}: {valid} valid front-facing points")
            if n >= num_scans:
                break

        if not front_dist:
            return None, None

        # avg_dist   = statistics.mean(front_dist)
        avg_dist = min(front_dist)
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
        PORT      = "COM4"    # adjust — Linux: "/dev/ttyUSB0"
        RADIUS_CM = 15.0

        worker = LidarThread("LidarWorker-1")
        done   = worker.submit_scan(
            port                = PORT,
            radius_cm           = RADIUS_CM,
            distance_cm         = None,       # None → auto-detect
            safety_margin_deg   = 5,
            scan_duration_sec   = 30,
            circle_tolerance_mm = 50,
        )
        print("Scanning in background — waiting...")
        done.wait()
        print("Done.")

        points, r_exp, plot_b64 = worker.process_data()
        if r_exp is not None:
            print(f"r_exp={r_exp:.2f} mm  |  {len(points)} contour points")
            with open("result_plot.png", "wb") as f:
                f.write(base64.b64decode(plot_b64))
            print("Plot saved → result_plot.png")

    def run_process_only(csv_path):
        worker = LidarThread("LidarWorker-2")
        points, r_exp, plot_b64 = worker.process_data(csv_path)
        if r_exp is not None:
            print(f"r_exp={r_exp:.2f} mm  |  {len(points)} contour points")
            with open("result_plot.png", "wb") as f:
                f.write(base64.b64decode(plot_b64))
            print("Plot saved → result_plot.png")

    if len(sys.argv) > 1:
        run_process_only(sys.argv[1])   # python lidar_worker_final.py scan.csv
    else:
        run_scan()                      # python lidar_worker_final.py