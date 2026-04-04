import threading
import time
import math
import datetime
import csv
import io
import base64
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

try:
    import serial
    _SERIAL_AVAILABLE = True
except ImportError:
    _SERIAL_AVAILABLE = False
# ── Constants ─────────────────────────────────────────────────────────────────

TOF_FOV_DEG = 27.0

DEFAULT_TOF_PATH = os.path.join("data", "tof_logs")
DEFAULT_TOF_FILE = os.path.join(DEFAULT_TOF_PATH, "tof.csv")
DEFAULT_TOF_IMG  = os.path.join(DEFAULT_TOF_PATH, "tof_plot.png")

os.makedirs(DEFAULT_TOF_PATH, exist_ok=True)


# ── Module-level helpers ──────────────────────────────────────────────────────

def tof_to_cartesian(distances_mm, fov_deg=TOF_FOV_DEG):
    n = len(distances_mm)
    if n == 0:
        return [], []
    half = fov_deg / 2.0
    angles_deg = [(-half) + i * (fov_deg / (n - 1)) for i in range(n)] if n > 1 else [0.0]
    xs, ys = [], []
    for angle_deg, dist in zip(angles_deg, distances_mm):
        if dist is None or dist <= 0:
            continue
        rad = math.radians(angle_deg)
        xs.append(dist * math.sin(rad))
        ys.append(dist * math.cos(rad))
    return xs, ys


# ── Thread class ──────────────────────────────────────────────────────────────

class TOFThread(threading.Thread):
    """
    Background thread that manages TOF snapshot collection into a CSV.

    record(port) — opens serial, sends START, reads one DATA: line,
                   appends [timestamp, millis, distance_mm] to CSV,
                   closes serial, returns distance in mm.

    reset()      — truncates the CSV so a fresh run starts clean.

    process_data() — reads the accumulated CSV and returns
                     (x_pts, y_pts, image64).
    """

    def __init__(self, name, baud=115200, path=DEFAULT_TOF_FILE):
        super().__init__(name=name)
        self.daemon          = True
        self._baud           = baud
        self._path           = path
        self._lock           = threading.Lock()
        self._conv_window    = []
        self._conv_tolerance = 5.0
        self.converged       = False
        self.last_distance   = None

    # ------------------------------------------------------------------ #
    #  run()                                                              #
    # ------------------------------------------------------------------ #
    def run(self):
        print(f"[{self.name}] TOF worker active, path: {self._path}")

    # ------------------------------------------------------------------ #
    #  record(port) — open serial, get one snapshot, close               #
    # ------------------------------------------------------------------ #
    def record(self, port, done_event=None):
        """
        Open the serial port, send START, wait for one DATA: line,
        append [timestamp, millis, distance_mm] to CSV, then close.

        Parameters
        ----------
        port : str   e.g. "COM5" or "/dev/ttyACM0"

        Returns
        -------
        distance_mm : float | None
        """
        distance_mm = None

        try:
            if not _SERIAL_AVAILABLE:
                raise RuntimeError("pyserial not installed.")

            # Bug fix 1: pass baud rate, set a read timeout
            ser = serial.Serial(port, self._baud, timeout=2)
            time.sleep(1) # Necessary for delaying arduino-python
            print(f"[{self.name}] Connected to {port} for snapshot")

            try:
                # Send START — Arduino will reply with exactly one DATA: line
                ser.reset_input_buffer()
                ser.write("START\n".encode("utf-8"))
                ser.flush()

                for _ in range(5):
                    raw = ser.readline()
                    if not raw:
                        print(f"[{self.name}] Timeout waiting for DATA:")
                        break

                    line = raw.decode("utf-8", errors="ignore").strip()

                    if line.startswith("DATA:"):
                        parts = line[5:].split(",")
                        if len(parts) == 2:
                            ts          = datetime.datetime.now().isoformat()
                            millis      = parts[0].strip()
                            distance_mm = float(parts[1].strip())
                            self.last_distance = distance_mm 

                            with self._lock:
                                folder = os.path.dirname(self._path)
                                if folder:
                                    os.makedirs(folder, exist_ok=True)
                                file_exists = os.path.exists(self._path)
                                is_empty    = file_exists and os.path.getsize(self._path) == 0
                                with open(self._path, 'a', newline='') as f:
                                    writer = csv.writer(f)
                                    if not file_exists or is_empty:
                                        writer.writerow(["timestamp", "millis", "distance_mm"])
                                    writer.writerow([ts, millis, distance_mm])
                                    f.flush()

                            print(f"[{self.name}] Recorded {distance_mm:.0f} mm @ {ts}")
                            self._check_convergence([distance_mm])
                        break  # got DATA:, stop looping regardless

                    elif line.startswith("MSG:"):
                        print(f"[{self.name}] Arduino: {line[4:]}")
                        # keep looping — MSG: lines are informational, not the data

            finally:
                ser.close()
                print(f"[{self.name}] Port {port} closed")

        except Exception:
            import traceback; traceback.print_exc()

        finally:
            if done_event:
                done_event.set()

        return distance_mm

    # ------------------------------------------------------------------ #
    #  reset()                                                            #
    # ------------------------------------------------------------------ #
    def reset(self):
        with self._lock:
            folder = os.path.dirname(self._path)
            if folder:
                os.makedirs(folder, exist_ok=True)
            with open(self._path, 'w', newline='') as f:
                f.truncate(0)
        self._conv_window = []
        self.converged    = False
        print(f"[{self.name}] CSV reset → {self._path}")

    # ------------------------------------------------------------------ #
    #  process_data()                                                     #
    # ------------------------------------------------------------------ #
    def process_data(self, raw_data=None, path=DEFAULT_TOF_FILE, r_trajectory=40, arc_span=360):
        """
        Convert accumulated TOF data to circular-orbit contour points and a plot.

        Parameters
        ----------
        r_trajectory : float   radius of the sensor orbit in mm (default 40)
        arc_span     : float   total arc covered in degrees (default 360)

        Returns
        -------
        x_trunk, y_trunk : list[float]   mapped bark contour points
        image64          : str           base64-encoded PNG
        """
        target = path or self._path

        if raw_data is not None:
            all_rows = [raw_data] if isinstance(raw_data[0], (int, float)) else raw_data
        else:
            all_rows = self._load_csv(target)
            if not all_rows:
                print(f"[{self.name}] No valid data in {target}")
                return [], [], None

        # Flatten to a simple list of distances: [[76.0], [75.0], ...] → [76.0, 75.0, ...]
        A = [row[0] for row in all_rows]
        print(f"[{self.name}] Processing {len(A)} reading(s)")

        num_points  = len(A)
        angles_deg  = np.linspace(0, arc_span, num_points, endpoint=False)
        angles_rad  = np.radians(angles_deg)

        x_sensor, y_sensor = [], []
        x_trunk,  y_trunk  = [], []

        for i in range(num_points):
            # Sensor orbit position
            x_sensor.append(r_trajectory * np.cos(angles_rad[i]))
            y_sensor.append(r_trajectory * np.sin(angles_rad[i]))

            # Mapped contour point (sensor position minus measured distance)
            r_trunk = r_trajectory - A[i]
            x_trunk.append(r_trunk * np.cos(angles_rad[i]))
            y_trunk.append(r_trunk * np.sin(angles_rad[i]))

        image64 = self._plot(A, x_sensor, y_sensor, x_trunk, y_trunk)
        return x_trunk, y_trunk, image64


    # ------------------------------------------------------------------ #
    #  stop()                                                             #
    # ------------------------------------------------------------------ #
    def stop(self):
        pass

    # ------------------------------------------------------------------ #
    #  _check_convergence()                                               #
    # ------------------------------------------------------------------ #
    def _check_convergence(self, distances_mm):
        self._conv_window.append(list(distances_mm))
        if len(self._conv_window) > 3:
            self._conv_window.pop(0)

        if len(self._conv_window) < 3:
            self.converged = False
            print(f"[{self.name}] Convergence: waiting ({len(self._conv_window)}/3 records)")
            return

        matrix      = np.array(self._conv_window, dtype=float)
        overall_mad = float(np.abs(matrix - matrix.mean(axis=0)).mean())

        self.converged = overall_mad <= self._conv_tolerance
        status = "CONVERGED" if self.converged else "not converged"
        print(f"[{self.name}] Convergence: {status} "
              f"(MAD={overall_mad:.2f} mm  tol={self._conv_tolerance:.1f} mm)")

    # ------------------------------------------------------------------ #
    #  _plot()                                                            #
    # ------------------------------------------------------------------ #
    def _plot(self, A, x_sensor, y_sensor, x_trunk, y_trunk):
        fig, ax = plt.subplots(figsize=(12, 12))

        # Origin
        ax.plot(0, 0, 'kx', markersize=12, markeredgewidth=2, label='Orbit center (0,0)')

        # Sensor orbit path
        ax.plot(x_sensor, y_sensor, 'r--', alpha=0.6, label=f'ToF orbit (R={round(np.hypot(x_sensor[0], y_sensor[0]))})')

        # Start / end markers
        ax.plot(x_sensor[0],  y_sensor[0],  'b^', markersize=12, label='Start position')
        ax.plot(x_sensor[-1], y_sensor[-1], 'rs', markersize=8,  fillstyle='none', label='End position')

        # Direction arrow
        ax.annotate('', xy=(x_sensor[min(5, len(x_sensor)-1)], y_sensor[min(5, len(y_sensor)-1)]),
                    xytext=(x_sensor[0], y_sensor[0]),
                    arrowprops=dict(arrowstyle='->', color='blue', lw=2, connectionstyle='arc3,rad=.2'))

        # Bark contour (closed loop)
        ax.plot(np.append(x_trunk, x_trunk[0]),
                np.append(y_trunk, y_trunk[0]),
                'go-', markersize=4, label='Mapped contour')
        ax.fill(x_trunk, y_trunk, 'g', alpha=0.15)

        # TOF beams
        for i in range(len(A)):
            ax.annotate('', xy=(x_trunk[i], y_trunk[i]),
                        xytext=(x_sensor[i], y_sensor[i]),
                        arrowprops=dict(arrowstyle='<->', color='blue', lw=1, alpha=0.5))

        ax.set_aspect('equal')
        ax.set_title(f'360° B-scan — {len(A)} reading(s) | bark contour from circular ToF orbit')
        ax.set_xlabel('X position (mm)')
        ax.set_ylabel('Y position (mm)')
        ax.grid(True, linestyle=':', alpha=0.5)
        ax.legend(loc='upper right')
        plt.tight_layout()

        buf = io.BytesIO()
        fig.savefig(buf, format='png', dpi=120)
        fig.savefig(DEFAULT_TOF_IMG, format='png', dpi=120)
        plt.close(fig)
        buf.seek(0)
        return base64.b64encode(buf.read()).decode('utf-8')
    # ------------------------------------------------------------------ #
    #  _load_csv()                                                        #
    # ------------------------------------------------------------------ #
    def _load_csv(self, path):
        rows = []
        try:
            with open(path, 'r', encoding='latin-1') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    try:
                        rows.append([float(row['distance_mm'])])
                    except (ValueError, KeyError):
                        continue
        except FileNotFoundError:
            print(f"[{self.name}] CSV not found: {path}")
        return rows


# ── Standalone usage ──────────────────────────────────────────────────────────
if __name__ == "__main__":
    worker = TOFThread("TOF-Worker-1", baud=115200)
    worker.start()

    worker.reset()

    for i in range(3):
        dist = worker.record(port="COM5")
        print(f"Position {i+1}: {dist} mm  |  converged={worker.converged}")

    x_pts, y_pts, img = worker.process_data()
    print(f"Total points: {len(x_pts)}")
    with open("tof_result.png", "wb") as f:
        f.write(base64.b64decode(img))
    print("Plot saved → tof_result.png")