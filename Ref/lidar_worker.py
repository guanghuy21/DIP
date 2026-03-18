from rplidar import RPLidar, RPLidarException
import time
import csv
import os
import io
import base64
import math
import statistics
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from circle_fit import taubinSVD
from datetime import datetime

# Configuration
PORT = 'COM4'
BAUDRATE = 115200
TIMEOUT = 3
MOTOR_STARTUP_DELAY = 3
DEFAULT_SCAN_DURATION = 30
MAX_POINTS_TO_KEEP = 10000
SAFETY_MARGIN_DEGREES = 5

PRESCAN_NUM_SCANS = 5
PRESCAN_MIN_QUALITY = 10
PRESCAN_FRONT_ANGLE_TOLERANCE = 10

CIRCLE_FIT_TOLERANCE_MM = 50
MIN_POINTS_FOR_CIRCLE_FIT = 10
CIRCLE_UPDATE_INTERVAL = 5

DEFAULT_RAW_PATH = os.path.join("data", "lidar_scan")
DEFAULT_RAW_FILE = os.path.join(DEFAULT_RAW_PATH, "raw.csv")
DEFAULT_RAW_IMG  = os.path.join(DEFAULT_RAW_PATH, "processed_lidar.png")
os.makedirs(DEFAULT_RAW_PATH, exist_ok=True)

# ── Geometry ──────────────────────────────────────────────────────────────────

def polar_to_cartesian(angle_deg, distance_mm):
    angle_rad = math.radians(angle_deg)
    return distance_mm * math.cos(angle_rad), distance_mm * math.sin(angle_rad)

def fit_circle(points):
    if len(points) < 3:
        return None
    try:
        points = np.array(points)
        x, y   = points[:, 0], points[:, 1]
        x_m, y_m = np.mean(x), np.mean(y)
        u, v = x - x_m, y - y_m
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
        return uc + x_m, vc + y_m, np.sqrt(uc**2 + vc**2 + (Suu + Svv) / len(x))
    except Exception:
        return None

def distance_to_circle(point, circle_params):
    if circle_params is None:
        return 0
    cx, cy, r = circle_params
    x, y = point
    return abs(math.sqrt((x - cx)**2 + (y - cy)**2) - r)

def filter_by_circle(points, expected_radius_mm, tolerance_mm=CIRCLE_FIT_TOLERANCE_MM):
    if len(points) < MIN_POINTS_FOR_CIRCLE_FIT:
        return points, None
    cartesian = [polar_to_cartesian(a, d) for _, a, d in points]
    circle_params = fit_circle(cartesian)
    if circle_params is None:
        return points, None
    filtered = [
        pt for pt, (x, y) in zip(points, cartesian)
        if distance_to_circle((x, y), circle_params) <= tolerance_mm
    ]
    return filtered, circle_params

def calculate_scan_angle(radius_cm, distance_cm, safety_margin=SAFETY_MARGIN_DEGREES):
    half_angle_deg = math.degrees(math.atan(radius_cm / distance_cm))
    half = half_angle_deg + safety_margin
    min_angle = -half
    max_angle =  half
    if min_angle < 0: min_angle += 360
    if max_angle < 0: max_angle += 360
    return min_angle, max_angle, half_angle_deg, half * 2

def filter_scan_data(scan, min_angle, max_angle):
    wraps = min_angle > max_angle
    filtered = []
    for q, a, d in scan:
        if wraps:
            if a >= min_angle or a <= max_angle:
                filtered.append((q, a, d))
        else:
            if min_angle <= a <= max_angle:
                filtered.append((q, a, d))
    return filtered

# ── Plot ──────────────────────────────────────────────────────────────────────

def plot_result(points, xc, yc, r, sigma, r_exp, cx_s, yc_s, sample_x, sample_y):
    """Generate plot, save to file and return base64 PNG string."""
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
    ax.set_title(f'Fit r={r:.2f} mm  ->  Expanded r={r_exp:.2f} mm   sigma={sigma:.3f} mm')
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
    print(f"Plot saved → {DEFAULT_RAW_IMG}")
    return base64.b64encode(buf.read()).decode('utf-8')

# ── Process data ──────────────────────────────────────────────────────────────

def process_data(path=DEFAULT_RAW_FILE):
    """Read raw.csv, fit circle, expand, generate plot."""
    print(f"\nProcessing: {path}")
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
                header = [c.strip().lower() for c in row]
                continue
            try:
                if header and 'angle' in header:
                    ai, di, qi = header.index('angle'), header.index('distance'), header.index('quality')
                    angles.append(float(row[ai]))
                    distances.append(float(row[di]))
                    qualities.append(int(float(row[qi])))
                else:
                    qualities.append(int(float(row[0])))
                    angles.append(float(row[1]))
                    distances.append(float(row[2]))
            except (ValueError, IndexError):
                continue

    if not angles:
        print("No valid data found.")
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

    # Polar → Cartesian (0° = forward = +Y)
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

    ys_shifted = [y + 50.0 for y in ys]  # y offset
    pts = np.array(list(zip(xs, ys_shifted)))
    xc, yc, r, sigma = taubinSVD(pts)
    print(f"Centre ({xc:.2f}, {yc:.2f}) mm | r={r:.2f} mm | σ={sigma:.3f}")

    expand_mm = 100.0
    n_points  = 36
    r_exp     = r + expand_mm

    sample_angles = np.array([math.radians(180 - i * (360.0 / n_points)) for i in range(n_points)])
    cx_s     = 0.0
    sample_x = cx_s + r_exp * np.cos(sample_angles)
    sample_y = yc   + r_exp * np.sin(sample_angles)
    sample_z = np.full(n_points, 10)

    print(f"Expanded r={r_exp:.2f} mm | centre (0, {yc:.2f}) | {n_points} pts CCW")

    plot_b64 = plot_result(pts, xc, yc, r, sigma, r_exp, cx_s, yc, sample_x, sample_y)

    points_out = [
        {"x": float(sample_x[i]), "y": float(sample_y[i]), "z": float(sample_z[i])}
        for i in range(n_points)
    ]
    return points_out, r_exp, plot_b64

# ── CSV helpers ───────────────────────────────────────────────────────────────

def get_unique_filename(base_filename):
    if not os.path.exists(base_filename):
        return base_filename
    print(f"\nWARNING: File '{base_filename}' already exists!")
    print("  1. Overwrite   2. Timestamp   3. Counter   4. Custom   5. Cancel")
    while True:
        choice = input("Choice (1-5): ").strip()
        if choice == '1':
            if input(f"Overwrite '{base_filename}'? (y/n): ").strip().lower() == 'y':
                return base_filename
        elif choice == '2':
            name, ext = os.path.splitext(base_filename)
            return f"{name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}{ext}"
        elif choice == '3':
            name, ext = os.path.splitext(base_filename)
            i = 1
            while os.path.exists(f"{name}_{i}{ext}"): i += 1
            return f"{name}_{i}{ext}"
        elif choice == '4':
            custom = input("Filename: ").strip()
            if not custom.endswith('.csv'): custom += '.csv'
            if not os.path.exists(custom): return custom
        elif choice == '5':
            print("Cancelled."); exit(0)

def initialize_csv(filename, radius, distance, min_angle, max_angle, duration, auto_detected, tolerance):
    with open(filename, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['# RPLIDAR Scan Data with Circle Filtering'])
        w.writerow([f'# Timestamp: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}'])
        w.writerow([f'# Object Radius: {radius:.2f} cm'])
        w.writerow([f'# Sensor Distance: {distance:.2f} cm {"(Auto-detected)" if auto_detected else "(Manual)"}'])
        w.writerow([f'# Scan Range: {min_angle:.2f} to {max_angle:.2f}'])
        w.writerow([f'# Scan Duration: {duration:.1f} seconds'])
        w.writerow([f'# Circle Fit Tolerance: {tolerance:.1f} mm'])
        w.writerow([f'# Max Points: {MAX_POINTS_TO_KEEP}'])
        w.writerow([''])
        w.writerow(['angle', 'distance', 'quality'])   # ← string header

def write_final_data_to_csv(filename, data_buffer):
    with open(filename, 'a', newline='') as f:
        w = csv.writer(f)
        for quality, angle, distance in data_buffer:
            w.writerow([f'{angle:.2f}', f'{distance:.2f}', quality])

# ── Auto-detect ───────────────────────────────────────────────────────────────

def auto_detect_distance(lidar, num_scans=PRESCAN_NUM_SCANS,
                         min_quality=PRESCAN_MIN_QUALITY,
                         angle_tolerance=PRESCAN_FRONT_ANGLE_TOLERANCE):
    print(f"\nAuto-detecting distance (front ±{angle_tolerance}°)...")

    def is_front(a): return a <= angle_tolerance or a >= (360 - angle_tolerance)

    front_dist, front_angles, scan_count = [], [], 0
    for scan in lidar.iter_scans(max_buf_meas=5000):
        scan_count += 1
        valid = 0
        for q, a, d in scan:
            if q >= min_quality and is_front(a):
                front_dist.append(d); front_angles.append(a); valid += 1
        print(f"  Scan {scan_count}/{num_scans}: {valid} pts in front")
        if scan_count >= num_scans: break

    if not front_dist:
        return None, None

    avg_dist  = statistics.mean(front_dist)
    norm_angs = [a - 360 if a > 180 else a for a in front_angles]
    avg_angle = statistics.mean(norm_angs)
    if avg_angle < 0: avg_angle += 360

    print(f"Detected {avg_dist:.1f} mm ({avg_dist/10:.1f} cm) at {avg_angle:.1f}°")
    return avg_dist, avg_angle

# ── User input ────────────────────────────────────────────────────────────────

def get_user_input():
    print("=" * 60)
    print("RPLIDAR SCANNER WITH CIRCLE FILTERING")
    print("=" * 60)

    while True:
        try:
            r = float(input("Object radius (cm): "))
            if r > 0: break
        except ValueError: pass

    auto = input("Auto-detect distance? (y/n): ").strip().lower() == 'y'
    distance = None

    if not auto:
        while True:
            try:
                d = float(input("Distance to object center (cm): "))
                if d > 0 and d > r: break
            except ValueError: pass
        distance = d

    margin_in = input(f"Safety margin deg (default {SAFETY_MARGIN_DEGREES}): ").strip()
    margin    = float(margin_in) if margin_in else SAFETY_MARGIN_DEGREES

    dur_in    = input(f"Scan duration sec (default {DEFAULT_SCAN_DURATION}): ").strip()
    duration  = float(dur_in) if dur_in else DEFAULT_SCAN_DURATION

    tol_in    = input(f"Circle tolerance mm (default {CIRCLE_FIT_TOLERANCE_MM}): ").strip()
    tolerance = float(tol_in) if tol_in else CIRCLE_FIT_TOLERANCE_MM

    return r, distance, margin, duration, auto, tolerance

# ── Cleanup ───────────────────────────────────────────────────────────────────

def cleanup_lidar(lidar):
    try:
        lidar.stop(); lidar.stop_motor(); lidar.disconnect()
        print("LIDAR stopped.")
    except Exception: pass

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    lidar = None
    try:
        radius, distance, margin, scan_duration, auto_detect, circle_tolerance = get_user_input()

        print(f"\nConnecting to RPLIDAR on {PORT}...")
        lidar = RPLidar(PORT, baudrate=BAUDRATE, timeout=TIMEOUT)
        info  = lidar.get_info()
        print(f"Connected: model={info['model']}  fw={info['firmware']}")

        lidar.start_motor()
        time.sleep(MOTOR_STARTUP_DELAY)

        auto_detected = False
        if auto_detect:
            dist_mm, _ = auto_detect_distance(lidar)
            if dist_mm is None:
                if input("Auto-detect failed. Enter manually? (y/n): ").lower() == 'y':
                    while True:
                        try:
                            d = float(input("Distance (cm): "))
                            if d > 0 and d > radius: break
                        except ValueError: pass
                    distance = d
                else:
                    cleanup_lidar(lidar); return
            else:
                distance = dist_mm / 10
                auto_detected = True
                print(f"Using detected distance: {distance:.2f} cm")

        min_angle, max_angle, half_angle, total_angle = calculate_scan_angle(radius, distance, margin)
        expected_radius_mm = radius * 10

        print(f"\nScan range: {min_angle:.2f}° to {max_angle:.2f}°  |  total: {total_angle:.2f}°")

        proceed = input("Proceed? (y/n, default y): ").strip().lower()
        if proceed == 'n':
            cleanup_lidar(lidar); return

        output_file = get_unique_filename("data_scan_circle_filtered.csv")
        if auto_detect:
            lidar.stop(); time.sleep(0.5)

        initialize_csv(output_file, radius, distance, min_angle, max_angle,
                       scan_duration, auto_detected, circle_tolerance)

        data_buffer  = []
        circle_params = None
        scan_count, last_upd = 0, 0
        points_rejected_angle = points_rejected_circle = points_rejected_buffer = 0
        t0 = time.time()

        print(f"\nScanning for {scan_duration}s...")

        for scan in lidar.iter_scans(max_buf_meas=50000):
            elapsed = time.time() - t0
            if elapsed >= scan_duration:
                print(f"\nDuration reached."); break

            before        = len(scan)
            by_angle      = filter_scan_data(scan, min_angle, max_angle)
            points_rejected_angle += before - len(by_angle)

            if scan_count - last_upd >= CIRCLE_UPDATE_INTERVAL and len(data_buffer) >= MIN_POINTS_FOR_CIRCLE_FIT:
                _, circle_params = filter_by_circle(data_buffer, expected_radius_mm, circle_tolerance)
                last_upd = scan_count

            if circle_params:
                by_circle, _ = filter_by_circle(by_angle, expected_radius_mm, circle_tolerance)
                points_rejected_circle += len(by_angle) - len(by_circle)
                filtered = by_circle
            else:
                filtered = by_angle

            for quality, angle, dist in filtered:
                if len(data_buffer) < MAX_POINTS_TO_KEEP:
                    data_buffer.append((quality, angle, dist))
                else:
                    worst = min(data_buffer, key=lambda p: p[0])
                    if quality > worst[0]:
                        data_buffer.remove(worst)
                        data_buffer.append((quality, angle, dist))
                    points_rejected_buffer += 1

            print(f"\rScan #{scan_count:4d} | buf={len(data_buffer):5d} | "
                  f"{elapsed/scan_duration*100:5.1f}%", end='', flush=True)
            scan_count += 1

        data_buffer.sort(key=lambda p: p[1])
        print(f"\nWriting {len(data_buffer)} points to {output_file}...")
        write_final_data_to_csv(output_file, data_buffer)

        # ── Process and plot ─────────────────────────────────────────────
        points_out, r_exp, plot_b64 = process_data(output_file)

        if points_out:
            print(f"\nExpanded r = {r_exp:.2f} mm  |  {len(points_out)} contour points")
            img_path = DEFAULT_RAW_IMG.replace('.png', '_result.png')
            with open(img_path, 'wb') as f:
                f.write(base64.b64decode(plot_b64))
            print(f"Result plot saved → {img_path}")

    except RPLidarException as e:
        print(f"\nRPLIDAR error: {e}")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except Exception as e:
        import traceback; traceback.print_exc()
    finally:
        if lidar: cleanup_lidar(lidar)


if __name__ == "__main__":
    main()