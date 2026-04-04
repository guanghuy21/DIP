from rplidar import RPLidar, RPLidarException
import threading, time, math, statistics, csv, io, base64, os, numpy as np
from circle_fit import taubinSVD
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

PORT, BAUDRATE, MOTOR_STARTUP_DELAY, MAX_POINTS = 'COM5', 115200, 3, 10000

DEFAULT_CONFIG = {
    'radius_cm':           5.0,
    'distance_cm':         None,
    'safety_margin_deg':   5,
    'scan_duration_sec':   30,
    'circle_tolerance_mm': 50,
    'port':                'COM5',
    'expand_mm':           100.0,   # how much to expand the fitted circle
    'n_points':            36,      # number of contour sample points
    'z_height_mm':         10.0,    # z value assigned to every contour point
}


class LidarThread(threading.Thread):

    def __init__(self, name, config=None):
        super().__init__(name=name)
        self.daemon = True
        cfg = {**DEFAULT_CONFIG, **(config or {})}

        if not float(cfg.get('radius_cm', 0)) > 0:
            raise ValueError("config must contain 'radius_cm' > 0")

        self.radius_cm           = float(cfg['radius_cm'])
        self.radius_mm           = self.radius_cm * 10
        self.distance_cm         = float(cfg['distance_cm']) if cfg.get('distance_cm') else None
        self.auto_detect         = self.distance_cm is None
        self.safety_margin_deg   = float(cfg.get('safety_margin_deg', 5))
        self.scan_duration_sec   = float(cfg.get('scan_duration_sec', 30))
        self.circle_tolerance_mm = float(cfg.get('circle_tolerance_mm', 50))
        self.port                = cfg.get('port', PORT)
        self.expand_mm           = float(cfg.get('expand_mm', 100.0))
        self.n_points            = int(cfg.get('n_points', 36))
        self.z_height_mm         = float(cfg.get('z_height_mm', 10.0))
        self.output_path         = 'raw_data.csv'
        self._lidar              = None

        # --- scan state (read by gateway to poll progress) ---
        self.scan_status  = 'idle'   # idle | running | done | error
        self.scan_error   = None
        self._scan_thread = None

    def run(self):
        pass  # Thread kept alive by the orchestrator; scanning triggered via scan_async()

    # ------------------------------------------------------------------ #
    #  PUBLIC API                                                          #
    # ------------------------------------------------------------------ #

    def scan_async(self):
        """
        Start a scan in a background thread and return immediately.
        The gateway can poll self.scan_status to know when it's done.
        """
        if self.scan_status == 'running':
            return  # already scanning, ignore duplicate calls
        self.scan_status = 'running'
        self.scan_error  = None
        self._scan_thread = threading.Thread(
            target=self._scan_worker, name=f"{self.name}-scan", daemon=True
        )
        self._scan_thread.start()

    def scan_sync(self):
        """
        Blocking scan — kept for standalone / CLI use.
        Returns True on success, False on failure.
        """
        self.scan_status = 'running'
        self.scan_error  = None
        result = self._scan_worker()
        return result

    def process_data(self, path=None):
        """
        Load a CSV of raw scan data, fit a circle, expand it, sample
        n_points contour points, generate a matplotlib plot.

        Returns (points, r_exp, plot_b64) on success.
        Returns (None, None, None) if there is not enough data.
        Raises ValueError with a descriptive message on processing failure
        so the gateway can return a proper 400/500 with context.
        """
        if path is None:
            path = self.output_path

        print(f"[{self.name}] Processing: {path}")
        angles, distances, _ = self._load_csv(path)

        if len(angles) < 10:
            raise ValueError(
                f"Too few data points ({len(angles)}) in {path}. "
                "Re-run the scan or lower circle_tolerance_mm."
            )

        # ── 1. Convert raw polar → Cartesian using RPLidar convention ──────
        # RPLidar: 0° = forward (+Y), angles increase clockwise.
        #   x =  distance * sin(angle)   (positive = right)
        #   y =  distance * cos(angle)   (positive = forward / away from sensor)
        #
        # No angle-bias correction before fitting — rotating the point cloud
        # before taubinSVD distorts the geometry and inflates the fitted radius.
        # taubinSVD finds the true centre directly from the raw Cartesian coords.
        # No Y offset either — adding an offset shifts the arc centre and makes
        # a small bottle (r≈35mm) fit to a much larger circle.
        xs, ys = [], []
        for a, dist in zip(angles, distances):
            rad = math.radians(a)
            xs.append(dist * math.sin(rad))
            ys.append(dist * math.cos(rad))

        # Sort by angle for clean arc plotting
        idx = sorted(range(len(angles)), key=lambda i: angles[i])
        xs = [xs[i] for i in idx]
        ys = [ys[i] for i in idx]

        pts = np.array(list(zip(xs, ys)))

        # ── 5. Fit circle (TaubinSVD) ────────────────────────────────────
        try:
            xc, yc, r, sigma = taubinSVD(pts)
        except Exception as e:
            raise ValueError(f"Circle fit failed: {e}. Check scan quality.")

        if r <= 0 or r > 5000:
            raise ValueError(
                f"Fitted radius {r:.1f} mm is implausible. "
                "Check scan data — possibly too much noise or wrong tolerance."
            )

        print(f"[{self.name}] Fit: centre=({xc:.2f}, {yc:.2f}) mm | r={r:.2f} mm | σ={sigma:.3f}")

        # ── 6. Expand circle & sample contour points ─────────────────────
        r_exp = r + self.expand_mm

        # Sample evenly CCW starting at 90° (top of circle, +Y direction).
        # 90° → cos=0, sin=1 → point lands at (cx_s, yc+r_exp) = top centre.
        sample_angles = np.array([
            math.radians(90 + i * (360.0 / self.n_points))
            for i in range(self.n_points)
        ])

        # X-centre: use the fitted xc so contour is truly centred on the trunk.
        # sample_x is in the SAME coordinate space as the raw scan (before
        # the display-only xc subtraction that happens inside _plot).
        cx_s     = xc
        sample_x = cx_s + r_exp * np.cos(sample_angles)
        sample_y = yc   + r_exp * np.sin(sample_angles)
        sample_z = np.full(self.n_points, self.z_height_mm)

        print(
            f"[{self.name}] Expanded r={r_exp:.2f} mm | "
            f"centre=(0, {yc:.2f}) | {self.n_points} pts"
        )

        # ── 7. Plot ──────────────────────────────────────────────────────
        plot_b64 = self._plot(
            pts, xc, yc, r, sigma,
            r_exp, cx_s, yc,
            sample_x, sample_y
        )

        # ── 8. Build point list ──────────────────────────────────────────
        points = [
            {"x": float(sample_x[i]),
             "y": float(sample_y[i]),
             "z": float(sample_z[i])}
            for i in range(self.n_points)
        ]

        # ── 9. Save red contour points to CSV ────────────────────────────
        abs_output   = os.path.abspath(self.output_path)
        contour_path = os.path.splitext(abs_output)[0] + '_contour_points.csv'

        # Close any existing file handle by writing to a temp file first,
        # then replacing — avoids PermissionError if the CSV is open in Excel.
        tmp_path = contour_path + '.tmp'
        with open(tmp_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['x', 'y', 'z'])
            for p in points:
                w.writerow([f"{p['x']:.4f}", f"{p['y']:.4f}", f"{p['z']:.4f}"])

        # Atomic replace (works on Windows too)
        if os.path.exists(contour_path):
            os.remove(contour_path)
        os.rename(tmp_path, contour_path)

        print(f"[{self.name}] Contour points saved → {contour_path} ({len(points)} pts)")

        return points, float(r_exp), plot_b64

    # ------------------------------------------------------------------ #
    #  INTERNAL                                                            #
    # ------------------------------------------------------------------ #

    def _scan_worker(self):
        """Runs the actual RPLidar scan. Called by both sync and async paths."""
        try:
            self._lidar = RPLidar(self.port, baudrate=BAUDRATE, timeout=3)
            self._lidar.start_motor()
            time.sleep(MOTOR_STARTUP_DELAY)

            # Auto-detect distance if not provided
            if self.auto_detect or self.distance_cm is None:
                dist_mm = self._detect_distance()
                if dist_mm is None:
                    raise RuntimeError(
                        "Auto-detection failed — no front points found. "
                        "Is the sensor pointing at the object?"
                    )
                self.distance_cm = dist_mm / 10
                self._lidar.stop()
                time.sleep(0.5)

            # Compute angular window
            half = (
                math.degrees(math.atan(self.radius_cm / self.distance_cm))
                + self.safety_margin_deg
            )
            min_ang = (360 - half) % 360
            max_ang = half % 360
            wraps   = min_ang > max_ang

            buf, circle_params, scan_count, last_upd = [], None, 0, 0
            t0 = time.time()

            for s in self._lidar.iter_scans(max_buf_meas=50000):
                if (time.time() - t0) >= self.scan_duration_sec:
                    break

                # Layer 1: angle filter
                if wraps:
                    pts = [(q, a, d) for q, a, d in s if d > 0 and (a >= min_ang or a <= max_ang)]
                else:
                    pts = [(q, a, d) for q, a, d in s if d > 0 and min_ang <= a <= max_ang]

                # Layer 2: circle filter (once we have a fit)
                if circle_params and pts:
                    cx, cy, cr = circle_params
                    pts = [
                        (q, a, d) for q, a, d in pts
                        if abs(
                            math.sqrt(
                                (d * math.cos(math.radians(a)) - cx) ** 2 +
                                (d * math.sin(math.radians(a)) - cy) ** 2
                            ) - cr
                        ) <= self.circle_tolerance_mm
                    ]

                # Buffer management — keep best-quality points
                for q, a, d in pts:
                    if len(buf) < MAX_POINTS:
                        buf.append((q, a, d))
                    else:
                        worst = min(buf, key=lambda p: p[0])
                        if q > worst[0]:
                            buf.remove(worst)
                            buf.append((q, a, d))

                # Refresh circle estimate every 5 scans
                if scan_count - last_upd >= 5 and len(buf) >= 10:
                    cart = [
                        (d * math.cos(math.radians(a)),
                         d * math.sin(math.radians(a)))
                        for _, a, d in buf
                    ]
                    cp = self._fit_circle_ls(cart)
                    if cp:
                        circle_params = cp
                    last_upd = scan_count

                scan_count += 1

            # Write CSV as (angle, distance, quality)
            buf.sort(key=lambda p: p[1])
            out_dir = os.path.dirname(self.output_path)
            if out_dir:
                os.makedirs(out_dir, exist_ok=True)

            with open(self.output_path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['angle', 'distance', 'quality'])
                w.writerows((a, d, q) for q, a, d in buf)

            print(f"[{self.name}] Scan saved → {self.output_path} ({len(buf)} pts)")
            self.scan_status = 'done'
            return True

        except RPLidarException as e:
            msg = f"RPLidar hardware error: {e}"
            print(f"[{self.name}] {msg}")
            self.scan_error  = msg
            self.scan_status = 'error'
            return False

        except Exception as e:
            import traceback
            msg = traceback.format_exc()
            print(f"[{self.name}] Scan failed:\n{msg}")
            self.scan_error  = str(e)
            self.scan_status = 'error'
            return False

        finally:
            self._cleanup()

    def _plot(self, points, xc, yc, r, sigma, r_exp, cx_s, yc_s, sample_x, sample_y):
        # Use raw Cartesian coordinates throughout — no artificial shifts.
        # Sensor is at origin (0,0); forward = +Y; right = +X.
        # xc/yc from taubinSVD are the true bottle centre in sensor space.
        xs_plot = points[:, 0]
        ys_plot = points[:, 1]

        fig, ax = plt.subplots(figsize=(7, 7))

        # Sensor origin marker
        ax.plot(0, 0, 'bs', markersize=7, label='Sensor origin')

        # Raw scan arc
        ax.scatter(xs_plot, ys_plot,
                   s=6, color='steelblue', alpha=0.7, label='Scan points')

        # Fitted circle at true centre (xc, yc)
        ax.add_patch(plt.Circle(
            (xc, yc), r,
            color='black', fill=False, linewidth=1.8,
            label=f'Fit  r={r:.1f} mm'
        ))
        ax.plot(xc, yc, 'k+', markersize=10, label=f'Fit centre ({xc:.0f}, {yc:.0f})')

        # Expanded circle at same centre
        ax.add_patch(plt.Circle(
            (cx_s, yc_s), r_exp,
            color='green', fill=False, linewidth=1.8, linestyle='--',
            label=f'Expanded  r={r_exp:.1f} mm'
        ))
        ax.plot(cx_s, yc_s, 'g+', markersize=10,
                label=f'Expanded centre ({cx_s:.0f}, {yc_s:.0f})')

        # 36 contour sample points
        ax.scatter(sample_x, sample_y,
                   s=55, color='red', edgecolors='black', zorder=5,
                   label=f'{len(sample_x)}-pt contour')
        # Start point at 90° = top of circle
        ax.scatter(sample_x[0], sample_y[0],
                   s=120, color='yellow', edgecolors='black', zorder=6,
                   label='Start pt (90°)')

        ax.set_aspect('equal', adjustable='datalim')
        ax.axvline(0, color='grey', linewidth=0.6, alpha=0.4)
        ax.set_title(
            f'Scan curve  →  Fit r={r:.2f} mm  →  '
            f'Expanded r={r_exp:.2f} mm   σ={sigma:.4f} mm'
        )
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.legend(fontsize=8, loc='upper right')
        ax.grid(True, linestyle=':', alpha=0.5)
        plt.tight_layout()

        buf = io.BytesIO()
        fig.savefig(buf, format='png', dpi=120)
        plt.close(fig)
        buf.seek(0)
        return base64.b64encode(buf.read()).decode('utf-8')

    def _detect_distance(self):
        tol, dists, count = self.safety_margin_deg, [], 0
        for s in self._lidar.iter_scans(max_buf_meas=5000):
            count += 1
            dists += [
                d for q, a, d in s
                if q >= 10 and d > 0 and (a <= tol or a >= 360 - tol)
            ]
            if count >= 5:
                break
        return statistics.median(dists) if dists else None

    def _load_csv(self, path):
        angles, distances, qualities = [], [], []
        with open(path, 'r', encoding='latin-1') as f:
            for row in csv.reader(f):
                # Skip blank, comment, and header rows
                if not row:
                    continue
                if row[0].startswith('#') or row[0].strip() in ('angle', 'quality'):
                    continue
                try:
                    # CSV written as (angle, distance, quality)
                    angles.append(float(row[0]))
                    distances.append(float(row[1]))
                    qualities.append(int(row[2]))
                except (ValueError, IndexError):
                    continue
        return angles, distances, qualities

    @staticmethod
    def _fit_circle_ls(cart_points):
        try:
            pts = np.array(cart_points)
            x, y = pts[:, 0], pts[:, 1]
            u, v = x - x.mean(), y - y.mean()
            Suu = (u ** 2).sum()
            Svv = (v ** 2).sum()
            Suv = (u * v).sum()
            A = np.array([[Suu, Suv], [Suv, Svv]])
            if abs(np.linalg.det(A)) < 1e-10:
                return None
            B  = np.array([
                0.5 * ((u ** 3).sum() + (u * v ** 2).sum()),
                0.5 * ((v ** 3).sum() + (u ** 2 * v).sum()),
            ])
            uc, vc = np.linalg.solve(A, B)
            r = math.sqrt(uc ** 2 + vc ** 2 + (Suu + Svv) / len(x))
            return uc + x.mean(), vc + y.mean(), r
        except Exception:
            return None

    def _cleanup(self):
        if self._lidar:
            try:
                self._lidar.stop()
                self._lidar.stop_motor()
                self._lidar.disconnect()
            except Exception:
                pass
            self._lidar = None


# ------------------------------------------------------------------ #
#  Standalone CLI                                                       #
# ------------------------------------------------------------------ #
if __name__ == "__main__":
    worker = LidarThread("LidarWorker-CLI", {
        'radius_cm':         5.0,
        'distance_cm':       None,
        'safety_margin_deg': 5,
        'scan_duration_sec': 30,
        'circle_tolerance_mm': 50,
        'expand_mm':         100.0,
        'n_points':          36,
        'z_height_mm':       10.0,
        'port':              'COM5',
    })

    import sys
    csv_path = sys.argv[1] if len(sys.argv) > 1 else 'raw_data.csv'

    try:
        points, r_exp, plot_b64 = worker.process_data(csv_path)
        print(f"r_exp = {r_exp:.2f} mm")
        print(f"Points ({len(points)} total):")
        for i, p in enumerate(points):
            print(f"  [{i:02d}]  x={p['x']:8.2f}  y={p['y']:8.2f}  z={p['z']:.1f}")
        with open('result_plot.png', 'wb') as f:
            f.write(base64.b64decode(plot_b64))
        print("Plot saved → result_plot.png")
    except ValueError as e:
        print(f"Processing error: {e}")
