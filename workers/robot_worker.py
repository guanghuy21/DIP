import threading
import time
import serial
import numpy as np
import matplotlib.pyplot as plt
import io
import base64
import requests
import os
import csv
import datetime

try:
    import serial
    _SERIAL_AVAILABLE = True
except ImportError:
    _SERIAL_AVAILABLE = False

BAUD_RATE   = 115200
TIMEOUT_SEC = 30
MM_TO_CM    = 0.1

DEFAULT_TOF_PATH = os.path.join("data", "tof_logs")
DEFAULT_TOF_FILE = os.path.join(DEFAULT_TOF_PATH, "tof.csv")
DEFAULT_TOF_IMG  = os.path.join(DEFAULT_TOF_PATH, "tof_plot.png")
os.makedirs(DEFAULT_TOF_PATH, exist_ok=True)


class RobotThread(threading.Thread):
    """
    Unified worker for SCARA arm + VL53L0X TOF sensor.
    Both are connected to the same Arduino Mega — one shared serial port.

    Serial Commands sent to Arduino:
        x,y,z            → Move arm (IK solved on Arduino)
        TOF_START         → Take one TOF reading
        SET_OFFSET:<val>  → Set LiDAR-to-J1 offset in cm (sent once at /init)
        SET_TRUNK:<x>,<y> → Set trunk centre in J1 frame in cm (sent once at /init)
        STATUS            → Get motor positions + TOF state

    Arduino replies:
        MOVING t1,t2,t3   → IK solved, motors starting
        DONE              → All motors stopped
        UNREACHABLE       → IK failed
        DATA:millis,mm    → TOF reading result
        ACK_OFFSET:<val>  → Offset confirmed
        ACK_TRUNK:<x>,<y> → Trunk centre confirmed
        MSG:...           → Informational, ignored by worker
    """

    def __init__(self, name, port=None, baud=BAUD_RATE, tof_path=DEFAULT_TOF_FILE):
        super().__init__(name=name)
        self.daemon       = True

        # ── SCARA state ───────────────────────────────────
        self.R_trajectory = 40
        self.position     = {"x": 0, "y": 0, "z": 0}
        self.is_moving    = False

        # ── Serial ────────────────────────────────────────
        self._serial      = None
        self._serial_lock = threading.Lock()
        self._baud        = baud
        self.port         = port

        # ── TOF state ─────────────────────────────────────
        self._tof_path       = tof_path
        self._tof_lock       = threading.Lock()
        self._conv_window    = []
        self._conv_tolerance = 5.0
        self.converged       = False
        self.last_distance   = None   # last reading in mm

        # ── Logging ───────────────────────────────────────
        self.log_path = "data/tof_logs/tof.log"
        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)

    # ──────────────────────────────────────────────────────
    # LIFECYCLE
    # ──────────────────────────────────────────────────────

    def run(self):
        print(f"[{self.name}] Robot controller active...")

        if self.port:
            print(f"[{self.name}] Auto-connecting to {self.port}...")
            self._connect(self.port)

        while True:
            if self.port and (self._serial is None or not self._serial.is_open):
                print(f"[{self.name}] Serial lost — reconnecting...")
                self._connect(self.port)
            time.sleep(2)

    def connect(self, port):
        """Open serial to Arduino. Must be called before any commands."""
        self.port = port
        self._connect(port)

    def _connect(self, port):
        try:
            self._serial = serial.Serial(port, self._baud, timeout=TIMEOUT_SEC)
            time.sleep(2)  # Arduino resets on serial open

            deadline = time.time() + 4
            while time.time() < deadline:
                if self._serial.in_waiting:
                    line = self._serial.readline().decode('utf-8', errors='ignore').strip()
                    print(f"[{self.name}] Boot: {line}")
                    if line == "MSG:READY":
                        break

            print(f"[{self.name}] Connected on {port} @ {self._baud}")
        except serial.SerialException as e:
            self._serial = None
            print(f"[{self.name}] ERROR connecting: {e}")

    # ──────────────────────────────────────────────────────
    # LOW-LEVEL SERIAL
    # ──────────────────────────────────────────────────────

    def _send_command(self, cmd: str) -> str:
        """
        Send one command, skip MSG: lines, return first meaningful response.
        Uses _serial_lock — do NOT call from inside another locked block.
        """
        if self._serial is None or not self._serial.is_open:
            raise RuntimeError("Serial not open — call connect(port) first")

        with self._serial_lock:
            self._serial.reset_input_buffer()
            self._serial.write(f"{cmd}\n".encode('utf-8'))
            self._serial.flush()
            print(f"[{self.name}] >> {cmd}")

            deadline = time.time() + TIMEOUT_SEC
            while time.time() < deadline:
                raw = self._serial.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='ignore').strip()
                if not line or line.startswith("MSG:"):
                    continue
                print(f"[{self.name}] << {line}")
                return line

            raise RuntimeError(f"Timeout waiting for response to: {cmd}")

    # ──────────────────────────────────────────────────────
    # INIT COMMANDS  (sent once by /init before pipeline)
    # ──────────────────────────────────────────────────────

    def set_lidar_offset(self, offset_cm: float, timeout: float = 3.0) -> bool:
        """
        Send SET_OFFSET:<val> to Arduino and wait for ACK_OFFSET.

        offset_cm : distance from LiDAR to J1 axis in cm.
                    Positive = LiDAR is closer to tree than J1.
                    The lidar_worker already applies this when generating
                    waypoints, so this call is informational for the Arduino
                    (used in STATUS replies).

        Must be called after connect() and before the pipeline starts.
        """
        if self._serial is None or not self._serial.is_open:
            raise RuntimeError("Serial not open — call connect(port) first")

        cmd = f"SET_OFFSET:{offset_cm:.2f}\n"
        with self._serial_lock:
            self._serial.write(cmd.encode('utf-8'))
            self._serial.flush()
            print(f"[{self.name}] >> SET_OFFSET:{offset_cm:.2f}")

            deadline = time.time() + timeout
            while time.time() < deadline:
                line = self._serial.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                print(f"[{self.name}] << {line}")
                if line.startswith("ACK_OFFSET:"):
                    confirmed = float(line.split(":")[1])
                    print(f"[{self.name}] Offset confirmed: {confirmed:.2f} cm")
                    return True

        print(f"[{self.name}] WARNING: No ACK_OFFSET received")
        return False

    def set_trunk_centre(self, cx_cm: float, cy_cm: float, timeout: float = 3.0) -> bool:
        """
        Send SET_TRUNK:<cx>,<cy> to Arduino and wait for ACK_TRUNK.

        cx_cm, cy_cm : trunk centre position in J1 frame, cm.
                       Obtained from lidar_worker.process_data() which returns
                       trunk_centre = {"cx_mm": ..., "cy_mm": ...}.
                       Divide by 10 to convert mm → cm before passing here.

        This MUST be sent before any move commands — the Arduino rejects
        moves with ERR:TRUNK_NOT_SET if this has not been received.
        """
        if self._serial is None or not self._serial.is_open:
            raise RuntimeError("Serial not open — call connect(port) first")

        cmd = f"SET_TRUNK:{cx_cm:.2f},{cy_cm:.2f}\n"
        with self._serial_lock:
            self._serial.write(cmd.encode('utf-8'))
            self._serial.flush()
            print(f"[{self.name}] >> SET_TRUNK:{cx_cm:.2f},{cy_cm:.2f}")

            deadline = time.time() + timeout
            while time.time() < deadline:
                line = self._serial.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                print(f"[{self.name}] << {line}")
                if line.startswith("ACK_TRUNK:"):
                    parts = line.split(":")[1].split(",")
                    print(f"[{self.name}] Trunk centre confirmed: "
                          f"({float(parts[0]):.2f}, {float(parts[1]):.2f}) cm")
                    return True

        print(f"[{self.name}] WARNING: No ACK_TRUNK received")
        return False

    # ──────────────────────────────────────────────────────
    # SCARA COMMANDS
    # ──────────────────────────────────────────────────────

    def _move_arm(self, x, y, z):
        """
        Send x,y,z to Arduino.
        Phase 1 — wait for MOVING (IK solved, motors starting)
        Phase 2 — wait for DONE  (motors physically stopped)
        """
        if self._serial is None or not self._serial.is_open:
            raise RuntimeError("Serial not open — call connect(port) first")

        cmd = f"{x:.2f},{y:.2f},{z:.2f}"

        with self._serial_lock:
            self._serial.reset_input_buffer()
            self._serial.write(f"{cmd}\n".encode('utf-8'))
            self._serial.flush()
            print(f"[{self.name}] >> {cmd}")

            # Phase 1: MOVING or UNREACHABLE
            deadline = time.time() + 10
            while time.time() < deadline:
                line = self._serial.readline().decode('utf-8', errors='ignore').strip()
                if not line or line.startswith("MSG:"):
                    continue
                print(f"[{self.name}] << {line}")
                if line == "UNREACHABLE":
                    raise RuntimeError(f"IK UNREACHABLE for ({x},{y},{z})")
                if line.startswith("MOVING"):
                    break
            else:
                raise RuntimeError("Timeout waiting for MOVING ack")

            # Phase 2: DONE
            deadline = time.time() + TIMEOUT_SEC
            while time.time() < deadline:
                line = self._serial.readline().decode('utf-8', errors='ignore').strip()
                if not line or line.startswith("MSG:"):
                    continue
                print(f"[{self.name}] << {line}")
                if line == "DONE":
                    return

            raise RuntimeError("Timeout waiting for DONE after MOVING")

    def get_arduino_status(self):
        """Send STATUS to Arduino, return raw response."""
        response = self._send_command("STATUS")
        return {"raw": response}

    # ──────────────────────────────────────────────────────
    # TOF COMMANDS
    # ──────────────────────────────────────────────────────

    def _read_tof_serial(self) -> float:
        """
        Send TOF_START over the shared serial port.
        Parses DATA:millis,mm and returns distance in cm.
        Uses _serial_lock — do NOT call from inside another locked block.
        """
        if self._serial is None or not self._serial.is_open:
            raise RuntimeError("Serial not open — call connect(port) first")

        with self._serial_lock:
            self._serial.reset_input_buffer()
            self._serial.write("TOF_START\n".encode('utf-8'))
            self._serial.flush()
            print(f"[{self.name}] >> TOF_START")

            deadline = time.time() + 5
            while time.time() < deadline:
                raw  = self._serial.readline()
                line = raw.decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                print(f"[{self.name}] << {line}")

                if line.startswith("DATA:"):
                    parts = line[5:].split(",")
                    if len(parts) == 2:
                        distance_mm        = float(parts[1].strip())
                        self.last_distance = distance_mm
                        self._log_tof_csv(distance_mm)
                        self._check_convergence([distance_mm])
                        return distance_mm * MM_TO_CM

                if line == "MSG:OUT_OF_RANGE":
                    raise RuntimeError("TOF out of range")
                if "TOF not available" in line:
                    raise RuntimeError("TOF sensor not initialized on Arduino")

            raise RuntimeError("Timeout waiting for TOF reading")

    def record(self, port=None, done_event=None):
        """
        Public TOF snapshot method — keeps compatibility with old tof_worker API.
        Uses the already-open shared serial connection.
        If port differs from current, reconnects first.
        Returns distance_mm or None on failure.
        """
        distance_mm = None
        try:
            if port and port != self.port:
                self.connect(port)

            if self._serial is None or not self._serial.is_open:
                raise RuntimeError("Serial not open — call connect(port) first")

            distance_cm = self._read_tof_serial()
            distance_mm = distance_cm / MM_TO_CM
            print(f"[{self.name}] TOF snapshot: {distance_mm:.0f} mm")

        except Exception:
            import traceback; traceback.print_exc()
        finally:
            if done_event:
                done_event.set()

        return distance_mm

    def reset_tof(self):
        """Truncate TOF CSV log and reset convergence state."""
        with self._tof_lock:
            folder = os.path.dirname(self._tof_path)
            if folder:
                os.makedirs(folder, exist_ok=True)
            with open(self._tof_path, 'w', newline='') as f:
                f.truncate(0)
        self._conv_window  = []
        self.converged     = False
        self.last_distance = None
        print(f"[{self.name}] TOF CSV reset → {self._tof_path}")

    # ──────────────────────────────────────────────────────
    # PUBLIC MOVE + TOF
    # ──────────────────────────────────────────────────────

    def move_to(self, x, y, z, path_tof=DEFAULT_TOF_FILE, done_event=None):
        """
        Move arm to (x, y, z) in cm, then take a TOF reading.
        Blocks until DONE received from Arduino.
        Appends (index, tof_cm) to path_tof CSV.
        Called by Flask route /robot/move.
        """
        self.is_moving = True
        try:
            self._move_arm(x, y, z)
            self.position = {"x": x, "y": y, "z": z}

            tof_cm = self._read_tof_serial()

            idx = self._get_next_tof_index(path_tof)
            os.makedirs(
                os.path.dirname(path_tof) if os.path.dirname(path_tof) else '.',
                exist_ok=True
            )
            with open(path_tof, 'a', newline='') as f:
                csv.writer(f).writerow([idx, tof_cm])
            print(f"[{self.name}] TOF {tof_cm:.2f} cm at index {idx}")

        except Exception as e:
            print(f"[{self.name}] ERROR in move_to: {e}")
        finally:
            self.is_moving = False
            if done_event:
                done_event.set()

        return self.position

    # ──────────────────────────────────────────────────────
    # CONTOUR TRAVERSAL
    # ──────────────────────────────────────────────────────

    def submit_contour(self, points: list, path_tof: str = DEFAULT_TOF_FILE) -> threading.Event:
        """
        Run full contour in background.
        Returns threading.Event — call .wait() to block until complete.
        """
        done_event = threading.Event()
        threading.Thread(
            target=self._contour_worker,
            args=(points, path_tof, done_event),
            name=f"{self.name}-contour",
            daemon=True,
        ).start()
        return done_event

    def run_contour(self, points: list, path_tof: str = DEFAULT_TOF_FILE) -> None:
        """Blocking version of submit_contour()."""
        self._contour_worker(points, path_tof, done_event=None)

    def _contour_worker(self, points, path_tof, done_event):
        """
        Visit every lidar contour point:
          1. Convert mm → cm  (lidar returns mm, Arduino expects cm)
          2. _move_arm()      blocks until DONE
          3. _read_tof_serial() reads distance on same serial port
          4. Append to CSV
        """
        if not points:
            print(f"[{self.name}] Empty points list.")
            if done_event:
                done_event.set()
            return

        self.is_moving = True
        total   = len(points)
        skipped = 0

        os.makedirs(
            os.path.dirname(path_tof) if os.path.dirname(path_tof) else '.',
            exist_ok=True
        )
        print(f"[{self.name}] Contour start — {total} points → {path_tof}")

        try:
            for i, pt in enumerate(points):
                x_cm = pt["x"] * MM_TO_CM
                y_cm = pt["y"] * MM_TO_CM
                z_cm = pt.get("z", 0) * MM_TO_CM

                print(f"[{self.name}] [{i+1}/{total}] "
                      f"x={x_cm:.2f} y={y_cm:.2f} z={z_cm:.2f} cm", end="  ")

                try:
                    self._move_arm(x_cm, y_cm, z_cm)
                    self.position = {"x": x_cm, "y": y_cm, "z": z_cm}
                    print("✓ DONE", end="  ")
                except RuntimeError as e:
                    print(f"⚠ SKIP ({e})")
                    skipped += 1
                    continue

                try:
                    tof_cm = self._read_tof_serial()
                    idx = self._get_next_tof_index(path_tof)
                    with open(path_tof, 'a', newline='') as f:
                        csv.writer(f).writerow([idx, tof_cm])
                    print(f"TOF={tof_cm:.2f} cm")
                except Exception as e:
                    print(f"TOF ERROR: {e}")

            print(f"[{self.name}] Contour complete — "
                  f"{total - skipped}/{total} done, {skipped} skipped.")

        except Exception as e:
            print(f"[{self.name}] ERROR in contour: {e}")
            import traceback; traceback.print_exc()
        finally:
            self.is_moving = False
            if done_event:
                done_event.set()

    # ──────────────────────────────────────────────────────
    # DATA PROCESSING  (B-Scan plot)
    # ──────────────────────────────────────────────────────

    def process_data(self, raw_data=None, path=None, r_trajectory=None, arc_span=360):
        """
        Generate B-Scan image from raw TOF list or CSV file.

        raw_data : flat list of distances in mm  e.g. [20, 19, 18, ...]
        path     : CSV file path (reads distance_mm column)
        Returns  : x_trunk, y_trunk, base64_png
        """
        R = r_trajectory if r_trajectory else self.R_trajectory

        if raw_data is not None:
            if raw_data and isinstance(raw_data[0], (int, float)):
                A = [float(v) for v in raw_data]
            else:
                A = [float(row[0]) for row in raw_data]
        elif path is not None:
            A = self._load_tof_csv(path)
        else:
            A = self._load_tof_csv(self._tof_path)

        if not A:
            print(f"[{self.name}] process_data: no data available")
            return [], [], None

        num_points = len(A)
        print(f"[{self.name}] Processing {num_points} TOF reading(s)")

        angles_rad = np.radians(np.linspace(0, arc_span, num_points, endpoint=False))
        x_sensor, y_sensor, x_trunk, y_trunk = [], [], [], []

        for i in range(num_points):
            xs = R * np.cos(angles_rad[i])
            ys = R * np.sin(angles_rad[i])
            x_sensor.append(xs)
            y_sensor.append(ys)
            r = R - A[i]
            x_trunk.append(r * np.cos(angles_rad[i]))
            y_trunk.append(r * np.sin(angles_rad[i]))

        fig, ax = plt.subplots(figsize=(12, 12))
        ax.plot(0, 0, 'kx', markersize=12, markeredgewidth=2, label='Orbit center (0,0)')
        ax.plot(x_sensor, y_sensor, 'r--', alpha=0.6, label=f'TOF orbit (R={R})')
        ax.plot(x_sensor[0], y_sensor[0], 'b^', markersize=12, label='Start')
        ax.plot(x_sensor[-1], y_sensor[-1], 'rs', markersize=8, fillstyle='none', label='End')
        ax.annotate('',
                    xy=(x_sensor[min(5, num_points-1)], y_sensor[min(5, num_points-1)]),
                    xytext=(x_sensor[0], y_sensor[0]),
                    arrowprops=dict(arrowstyle='->', color='blue', lw=2,
                                    connectionstyle='arc3,rad=.2'))
        ax.plot(np.append(x_trunk, x_trunk[0]),
                np.append(y_trunk, y_trunk[0]),
                'go-', markersize=4, label='Mapped contour')
        ax.fill(x_trunk, y_trunk, 'g', alpha=0.15)
        for i in range(num_points):
            ax.annotate('', xy=(x_trunk[i], y_trunk[i]),
                        xytext=(x_sensor[i], y_sensor[i]),
                        arrowprops=dict(arrowstyle='<->', color='blue', lw=1, alpha=0.5))
        ax.set_aspect('equal')
        ax.set_title(f'360° B-Scan — {num_points} readings')
        ax.set_xlabel('X (mm)'); ax.set_ylabel('Y (mm)')
        ax.grid(True, linestyle=':', alpha=0.5)
        ax.legend(loc='upper right')
        plt.tight_layout()

        buf = io.BytesIO()
        fig.savefig(buf, format='png', dpi=120)
        fig.savefig(DEFAULT_TOF_IMG, format='png', dpi=120)
        plt.close(fig)
        buf.seek(0)
        image64 = base64.b64encode(buf.read()).decode('utf-8')

        self._log_raw_data(A)
        return x_trunk, y_trunk, image64

    def fetch_lidar_points(self, target_url, data_path):
        """Fetch processed LiDAR contour points from Flask API endpoint."""
        try:
            response = requests.post(target_url, json={"path": data_path}, timeout=10)
            response.raise_for_status()
            return response.json().get("points", {})
        except requests.exceptions.RequestException as e:
            print(f"[{self.name}] fetch_lidar_points error: {e}")
            return None

    # ──────────────────────────────────────────────────────
    # HELPERS
    # ──────────────────────────────────────────────────────

    def _log_tof_csv(self, distance_mm):
        """Append one TOF reading to the persistent CSV log."""
        with self._tof_lock:
            folder = os.path.dirname(self._tof_path)
            if folder:
                os.makedirs(folder, exist_ok=True)
            file_exists = os.path.exists(self._tof_path)
            is_empty    = file_exists and os.path.getsize(self._tof_path) == 0
            with open(self._tof_path, 'a', newline='') as f:
                writer = csv.writer(f)
                if not file_exists or is_empty:
                    writer.writerow(["timestamp", "millis", "distance_mm"])
                writer.writerow([
                    datetime.datetime.now().isoformat(),
                    int(time.time() * 1000),
                    distance_mm
                ])
                f.flush()

    def _load_tof_csv(self, path):
        """Read distance_mm column from TOF CSV. Returns list of floats."""
        rows = []
        try:
            with open(path, 'r', encoding='latin-1') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    try:
                        rows.append(float(row['distance_mm']))
                    except (ValueError, KeyError):
                        continue
        except FileNotFoundError:
            print(f"[{self.name}] CSV not found: {path}")
        return rows

    def _log_raw_data(self, raw_A):
        """Append raw TOF array to human-readable log file."""
        try:
            with open(self.log_path, "a") as f:
                f.write(f"{time.strftime('%Y%m%d-%H%M%S')},"
                        f"{','.join(map(str, raw_A))}\n")
        except Exception as e:
            print(f"[{self.name}] Log error: {e}")

    def _get_next_tof_index(self, path_tof):
        """Count existing rows in CSV to get next index."""
        if not os.path.exists(path_tof):
            return 0
        with open(path_tof, 'r') as f:
            return sum(1 for row in csv.reader(f) if row)

    def _check_convergence(self, distances_mm):
        """Check if last 3 readings are within tolerance."""
        self._conv_window.append(list(distances_mm))
        if len(self._conv_window) > 3:
            self._conv_window.pop(0)
        if len(self._conv_window) < 3:
            self.converged = False
            return
        matrix      = np.array(self._conv_window, dtype=float)
        overall_mad = float(np.abs(matrix - matrix.mean(axis=0)).mean())
        self.converged = overall_mad <= self._conv_tolerance
        status = "CONVERGED" if self.converged else "not converged"
        print(f"[{self.name}] Convergence: {status} (MAD={overall_mad:.2f} mm)")

    def get_status(self):
        return {
            "position":      self.position,
            "moving":        self.is_moving,
            "last_distance": self.last_distance,
            "converged":     self.converged,
        }