from rplidar import RPLidar, RPLidarException
import threading, time, math, statistics, csv, io, base64
import numpy as np
from circle_fit import taubinSVD
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

BAUDRATE            = 115200
MOTOR_STARTUP_DELAY = 3
MAX_POINTS          = 10000

DEFAULT_RAW_PATH = os.path.join( "data", "lidar_scan")
DEFAULT_RAW_FILE = os.path.join(DEFAULT_RAW_PATH, "raw.csv")
DEFAULT_RAW_IMG  = os.path.join(DEFAULT_RAW_PATH, "processed_lidar.png")

os.makedirs(DEFAULT_RAW_PATH, exist_ok=True)


class LidarThread(threading.Thread):

    def __init__(self, name):
        super().__init__(name=name)
        self.daemon  = True
        self.running = True
        self._lidar  = None

    def run(self):
        print(f"[{self.name}] Lidar controller active...")
        while True:
            time.sleep(1)

    # ------------------------------------------------------------------ #
    #  scan()                                                             #
    # ------------------------------------------------------------------ #
    def scan(self, port, radius_cm, distance_cm=None, safety_margin_deg=3,
             scan_duration_sec=30, circle_toleration_mm=50, done_event=None):
        try:
            self.radius_cm           = radius_cm
            self.radius_mm           = self.radius_cm * 10
            self.distance_cm         = distance_cm
            self.auto_detect         = self.distance_cm is None
            self.safety_margin_deg   = safety_margin_deg
            self.scan_duration_sec   = scan_duration_sec
            self.circle_tolerance_mm = circle_toleration_mm
            self.port                = port
            print("111111111111111111111111111")

            if not self._connect_lidar(port):
                raise RuntimeError("Failed to connect to RPLidar after retries.")

            # Auto-detect sensor→object distance if not provided
            if self.auto_detect:
                dist_mm = self._detect_distance()
                if dist_mm is None:
                    raise RuntimeError("Auto-detection failed – no front points found.")
                self.distance_cm = dist_mm / 10
                self._lidar.stop()
                time.sleep(0.5)
            print("111111111111111111111111111")

            # Angle window around 0° (forward)
            half    = math.degrees(math.atan(self.radius_cm / self.distance_cm)) + self.safety_margin_deg
            min_ang = (360 - half) % 360
            max_ang = half % 360
            wraps   = min_ang > max_ang

            buf, circle_params = [], None
            scan_count, last_upd = 0, 0
            t0 = time.time()

            for attempt in range(3):
                try:
                    for scan in self._lidar.iter_scans(max_buf_meas=50000):
                        if not self.running or (time.time() - t0) >= self.scan_duration_sec:
                            break

                        # Layer 1 – angle filter
                        pts = [(q, a, d) for q, a, d in scan
                               if d > 0 and (a >= min_ang or a <= max_ang if wraps
                                             else min_ang <= a <= max_ang)]

                        # Layer 2 – circle filter
                        if circle_params and pts:
                            cx, cy, cr = circle_params
                            pts = [(q, a, d) for q, a, d in pts
                                   if abs(math.sqrt(
                                       (d * math.cos(math.radians(a)) - cx)**2 +
                                       (d * math.sin(math.radians(a)) - cy)**2
                                   ) - cr) <= self.circle_tolerance_mm]

                        # Quality-ranked buffer
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
                            cart = [(d * math.cos(math.radians(a)), d * math.sin(math.radians(a)))
                                    for _, a, d in buf]
                            cp = self._fit_circle_ls(cart)
                            if cp:
                                circle_params = cp
                            last_upd = scan_count

                        scan_count += 1

                    break  # clean exit — no retry needed

                except RPLidarException as e:
                    print(f"[{self.name}] Scan error (attempt {attempt + 1}/3): {e}")
                    if attempt < 2:
                        print(f"[{self.name}] Reconnecting...")
                        self._connect_lidar(port)
                        t0 = time.time()
                    else:
                        raise

            buf.sort(key=lambda p: p[1])

            # Save raw.csv — columns: quality, angle, distance
            os.makedirs(DEFAULT_RAW_PATH, exist_ok=True)
            with open(DEFAULT_RAW_FILE, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['quality', 'angle', 'distance'])
                w.writerows(buf)
            print(f"[{self.name}] Saved {len(buf)} points → {DEFAULT_RAW_FILE}")

        except RPLidarException as e:
            print(f"[{self.name}] RPLidar error: {e}")
        except Exception:
            import traceback; traceback.print_exc()
        finally:
            self._cleanup()
            if done_event:
                done_event.set()

    # ------------------------------------------------------------------ #
    #  process_data()                                                     #
    # ------------------------------------------------------------------ #
    def process_data(self, path=DEFAULT_RAW_FILE):
        print(f"[{self.name}] Processing file at: {path}")

        qualities, angles, distances = self._load_csv(path)   # matches CSV column order

        if not angles:
            print(f"[{self.name}] No valid data found.")
            return None, None, None

        # Normalize angles → (-180, 180]
        def norm(a):
            while a >  180: a -= 360
            while a <= -180: a += 360
            return a
        norm_ang = [norm(a) for a in angles]

        # Weighted center angle from closest points
        n   = max(5, len(distances) // 10)
        top = sorted(zip(distances, norm_ang))[:n]
        tw  = sum(1/d for d, _ in top)
        ca  = sum((1/d)*a for d, a in top) / tw

        # Polar → Cartesian  (0° = forward = +Y)
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

        # Apply y offset before fitting
        y_offset_mm = 50.0
        ys_shifted  = [y + y_offset_mm for y in ys]

        pts = np.array(list(zip(xs, ys_shifted)))
        xc, yc, r, sigma = taubinSVD(pts)

        print(f"[{self.name}] Centre ({xc:.2f}, {yc:.2f}) mm | r={r:.2f} mm | σ={sigma:.3f}")

        # Expand circle and sample contour
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
    #  Plot → base64 PNG                                                  #
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
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
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
                # Full teardown before each attempt
                if self._lidar:
                    try:
                        self._lidar.stop()
                        self._lidar.stop_motor()
                        self._lidar.disconnect()
                    except Exception:
                        pass
                    self._lidar = None
                    time.sleep(1.0)  # let serial port fully release

                self._lidar = RPLidar(port, baudrate=BAUDRATE, timeout=3)

                # Flush stale bytes that cause descriptor mismatch
                if hasattr(self._lidar, '_serial') and self._lidar._serial:
                    self._lidar._serial.reset_input_buffer()
                    self._lidar._serial.reset_output_buffer()

                self._lidar.stop()       # reset any in-progress command
                time.sleep(0.3)
                self._lidar.start_motor()
                time.sleep(MOTOR_STARTUP_DELAY)

                # Handshake — confirms clean comms before iter_scans
                info = self._lidar.get_info()
                print(f"[{self.name}] Connected (attempt {attempt+1}): {info}")
                return True

            except RPLidarException as e:
                print(f"[{self.name}] Connect attempt {attempt+1}/{max_retries} failed: {e}")
                time.sleep(2.0)

        return False

    def _detect_distance(self):
        tol, dists, count = self.safety_margin_deg, [], 0
        for scan in self._lidar.iter_scans(max_buf_meas=5000):
            count += 1
            dists += [d for q, a, d in scan
                      if q >= 10 and d > 0 and (a <= tol or a >= 360 - tol)]
            if count >= 5:
                break
        return statistics.median(dists) if dists else None

    def _load_csv(self, path):
        """
        Handles both original scanner format (angle, distance, quality)
        and lidar_worker format (quality, angle, distance).
        """
        qualities, angles, distances = [], [], []
        with open(path, 'r', encoding='latin-1') as f:
            reader = csv.reader(f)
            header = None
            for row in reader:
                if not row or row[0].startswith('#') or row[0].strip() == '':
                    continue

                # Detect header row — any row whose first cell is non-numeric
                try:
                    float(row[0])
                except ValueError:
                    header = [col.strip().lower() for col in row]
                    continue

                # Parse data row according to detected header
                try:
                    if header and 'angle' in header:
                        ai = header.index('angle')
                        di = header.index('distance')
                        qi = header.index('quality')
                        angles.append(float(row[ai]))
                        distances.append(float(row[di]))
                        qualities.append(int(float(row[qi])))
                    else:
                        # Fallback: assume lidar_worker order (quality, angle, distance)
                        qualities.append(int(float(row[0])))
                        angles.append(float(row[1]))
                        distances.append(float(row[2]))
                except (ValueError, IndexError):
                    continue

        return qualities, angles, distances

    @staticmethod
    def _fit_circle_ls(cart_points):
        try:
            pts = np.array(cart_points)
            x, y = pts[:, 0], pts[:, 1]
            u, v = x - x.mean(), y - y.mean()
            Suu, Svv, Suv = (u**2).sum(), (v**2).sum(), (u*v).sum()
            A = np.array([[Suu, Suv], [Suv, Svv]])
            if abs(np.linalg.det(A)) < 1e-10: return None
            B  = np.array([0.5*((u**3).sum() + (u*v**2).sum()),
                           0.5*((v**3).sum() + (u**2*v).sum())])
            uc, vc = np.linalg.solve(A, B)
            r = math.sqrt(uc**2 + vc**2 + (Suu + Svv) / len(x))
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


# ── Standalone test ───────────────────────────────────────────────────────────
if __name__ == "__main__":
    worker = LidarThread("LidarWorker-1")
    points, r_exp, plot_b64 = worker.process_data('data_scan_circle_filtered_6.csv')

    if r_exp is not None:
        print(f"r_exp={r_exp:.2f} mm  |  {len(points)} contour points")
        with open('result_plot.png', 'wb') as f:
            f.write(base64.b64decode(plot_b64))
        print("Plot saved → result_plot.png")