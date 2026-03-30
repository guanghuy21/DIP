import threading
import time
import serial
import numpy as np
import matplotlib.pyplot as plt
import io
import base64
import os
import csv

BAUD_RATE   = 9600   # must match Serial.begin(9600) in Arduino
TIMEOUT_SEC = 10     # seconds to wait for Arduino reply (movement can be slow)
MM_TO_CM = 0.1       # points from LidarThread.process_data() are in mm; Arduino expects cm

class RobotThread(threading.Thread):
    def __init__(self, name):
        super().__init__(name=name)
        self.daemon       = True
        self.R_trajectory = 40
        self.position     = {"x": 0, "y": 0, "z": 0}
        self.is_moving    = False

        self._serial      = None
        self._serial_lock = threading.Lock()
        self.port         = None

        self.log_path = "data/tof_logs/tof.log"
        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)

    # ──────────────────────────────────────────────
    # LIFECYCLE
    # ──────────────────────────────────────────────

    def run(self):
        print(f"[{self.name}] Robot controller active...")
        while True:
            if self.port and (self._serial is None or not self._serial.is_open):
                print(f"[{self.name}] Serial lost — reconnecting...")
                self._connect(self.port)
            time.sleep(2)

    def connect(self, port):
        """Open serial to Arduino. Call this before any move_to() calls."""
        self.port = port
        self._connect(port)

    def _connect(self, port):
        try:
            self._serial = serial.Serial(port, BAUD_RATE, timeout=TIMEOUT_SEC)
            time.sleep(2)  # Arduino resets on serial open — wait for READY

            # Drain the READY message Arduino sends on boot
            if self._serial.in_waiting:
                boot_msg = self._serial.readline().decode('utf-8').strip()
                print(f"[{self.name}] Arduino boot: {boot_msg}")

            print(f"[{self.name}] Connected to Arduino on {port} @ {BAUD_RATE}")
        except serial.SerialException as e:
            self._serial = None
            print(f"[{self.name}] ERROR connecting to Arduino: {e}")

    # ──────────────────────────────────────────────
    # SERIAL PRIMITIVES
    # ──────────────────────────────────────────────

    def _send_command(self, cmd: str) -> str:
        """Send one command line to Arduino, return its response line.
        
        Thread-safe — _serial_lock prevents concurrent writes.
        Raises RuntimeError if serial is not open or command fails.
        """
        if self._serial is None or not self._serial.is_open:
            raise RuntimeError("Serial port not open — call connect(port) first")

        with self._serial_lock:
            self._serial.reset_input_buffer()
            self._serial.write(f"{cmd}\n".encode('utf-8'))
            response = self._serial.readline().decode('utf-8').strip()
            print(f"[{self.name}]  >> {cmd}")
            print(f"[{self.name}]  << {response}")
            return response

    # ──────────────────────────────────────────────
    # ARDUINO COMMANDS
    # ──────────────────────────────────────────────

    def _move_arm(self, x, y, z):
        """Send MOVE command, expect ACK from Arduino.
        
        Arduino runs IK internally, drives all 3 steppers, then replies ACK.
        This call blocks until movement is physically complete.
        """
        # Format: "MOVE 360.00 200.00 100.00"
        response = self._send_command(f"MOVE {x:.2f} {y:.2f} {z:.2f}")
        if response == "ACK":
            return
        raise RuntimeError(f"Arduino rejected MOVE: {response}")

    def _read_tof(self) -> float:
        """Placeholder — wire in your real ToF sensor read here.
        
        If ToF is connected to Arduino:
            response = self._send_command("READ_TOF")
            return float(response.split(":")[1])   # expects "TOF:18.4"
        
        If ToF is connected directly to Pi/PC via I2C or serial,
        replace with your sensor library call instead.
        """
        raise NotImplementedError("Implement real ToF sensor read here")

    def get_arduino_status(self):
        """Send STATUS, returns parsed position dict from Arduino."""
        response = self._send_command("STATUS")
        # Expects "POS:360,200,100"
        if response.startswith("POS:"):
            x, y, z = response.replace("POS:", "").split(",")
            return {"x": float(x), "y": float(y), "z": float(z)}
        raise RuntimeError(f"Unexpected STATUS response: {response}")

    # ──────────────────────────────────────────────
    # PUBLIC WORKER METHODS
    # ──────────────────────────────────────────────

    def move_to(self, x, y, z, path_tof='tof.csv', done_event=None):
        """Move SCARA to (x,y,z), read ToF, append to CSV."""
        self.is_moving = True
        try:
            # 1. Send to Arduino — blocks until ACK (movement done)
            self._move_arm(x, y, z)
            self.position = {"x": x, "y": y, "z": z}

            # 2. Read ToF distance
            tof_cm = self._read_tof()

            # 3. Log to CSV
            point_index = self._get_next_tof_index(path_tof)
            os.makedirs(os.path.dirname(path_tof) if os.path.dirname(path_tof) else '.', exist_ok=True)
            with open(path_tof, 'a', newline='') as f:
                csv.writer(f).writerow([point_index, tof_cm])
            print(f"[{self.name}] ToF {tof_cm} cm at index {point_index}")

        except NotImplementedError:
            print(f"[{self.name}] WARNING: ToF not implemented, skipping log.")
        except Exception as e:
            print(f"[{self.name}] ERROR in move_to: {e}")
        finally:
            self.is_moving = False
            if done_event:
                done_event.set()

        return self.position
    
    # ------------------------------------------------------------------ #
    #  submit_contour() — public API                                      #
    # ------------------------------------------------------------------ #
    def submit_contour(self, points: list, path_tof: str = 'tof.csv') -> threading.Event:
        """
        Run the full contour traversal in the background.
        Returns a threading.Event — call .wait() to block until all points
        are visited and the END signal has been sent.
 
        Usage:
            robot = RobotThread("Robot-1")
            robot.connect("COM4")
            robot.start()
 
            done = robot.submit_contour(points)
            done.wait()
            x_trunk, y_trunk, plot_b64 = robot.process_data(path='tof.csv')
        """
        done_event = threading.Event()
        t = threading.Thread(
            target=self._contour_worker,
            args=(points, path_tof, done_event),
            name=f"{self.name}-contour",
            daemon=True,
        )
        t.start()
        return done_event
 
    # ------------------------------------------------------------------ #
    #  run_contour() — blocking version                                   #
    # ------------------------------------------------------------------ #
    def run_contour(self, points: list, path_tof: str = 'tof.csv') -> None:
        """
        Blocking version of submit_contour().
        Iterates through every point from LidarThread.process_data(),
        moves the arm to each one, reads ToF, and logs results.
        Sends END to Arduino when all points are covered.
 
        points  — list of dicts: [{"x": mm, "y": mm, "z": mm}, ...]
        path_tof — CSV file to append ToF readings to
        """
        self._contour_worker(points, path_tof, done_event=None)
 
    # ------------------------------------------------------------------ #
    #  _contour_worker() — internal traversal logic                      #
    # ------------------------------------------------------------------ #
    def _contour_worker(self, points: list, path_tof: str,
                        done_event: threading.Event | None) -> None:
        """
        Core point-by-point traversal.
 
        For each point in `points`:
          1. Convert x, y, z from mm → cm  (Arduino expects cm)
          2. Send MOVE command via _move_arm() — blocks until ACK
          3. Read ToF sensor
          4. Append (index, tof_cm) to path_tof CSV
 
        After all points:
          5. Send END command so Arduino can de-energise motors
 
        Skips unreachable points gracefully (Arduino returns non-ACK);
        execution continues with the next point.
        """
        if not points:
            print(f"[{self.name}] run_contour: empty points list, nothing to do.")
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
 
        print(f"[{self.name}] Starting contour traversal — {total} points → {path_tof}")
 
        try:
            for i, pt in enumerate(points):
                # ── Unit conversion: mm → cm ──────────────────────────────
                x_cm = pt["x"] * MM_TO_CM
                y_cm = pt["y"] * MM_TO_CM
                z_cm = pt["z"] * MM_TO_CM
 
                print(f"[{self.name}] Point {i+1:>2}/{total}  "
                      f"x={x_cm:.3f} y={y_cm:.3f} z={z_cm:.3f} cm", end="  ")
 
                # ── Move arm ──────────────────────────────────────────────
                try:
                    self._move_arm(x_cm, y_cm, z_cm)
                    self.position = {"x": x_cm, "y": y_cm, "z": z_cm}
                    print("✓ ACK", end="  ")
                except RuntimeError as e:
                    print(f"⚠ SKIPPED ({e})")
                    skipped += 1
                    continue    # skip ToF read for unreachable points
 
                # ── Read ToF ──────────────────────────────────────────────
                try:
                    tof_cm = self._read_tof()
                    point_index = self._get_next_tof_index(path_tof)
                    with open(path_tof, 'a', newline='') as f:
                        csv.writer(f).writerow([point_index, tof_cm])
                    print(f"ToF={tof_cm:.2f} cm  idx={point_index}")
                except NotImplementedError:
                    print("ToF=N/A (not implemented)")
                except Exception as e:
                    print(f"ToF ERROR: {e}")
 
            # ── Signal end of sequence to Arduino ─────────────────────────
            try:
                response = self._send_command("END")
                print(f"[{self.name}] END sent → Arduino: {response}")
            except Exception as e:
                print(f"[{self.name}] WARNING: END command failed: {e}")
 
            print(f"[{self.name}] Contour complete. "
                  f"{total - skipped}/{total} points executed, "
                  f"{skipped} skipped.")
 
        except Exception as e:
            print(f"[{self.name}] ERROR during contour traversal: {e}")
            import traceback; traceback.print_exc()
        finally:
            self.is_moving = False
            if done_event:
                done_event.set()

    def process_data(self, raw_A=None, path=None):
        """Generate B-Scan image from raw ToF list or CSV file."""
        if raw_A is None:
            if path is None:
                print(f"[{self.name}] ERROR: provide raw_A or path.")
                return [], [], None
            raw_A = []
            try:
                with open(path, 'r') as f:
                    for row in csv.reader(f):
                        if row:
                            raw_A.append(float(row[1]))
            except Exception as e:
                print(f"[{self.name}] ERROR reading CSV: {e}")
                return [], [], None

        self._log_raw_data(raw_A)
        num_points = len(raw_A)
        if num_points < 2:
            return [], [], None

        angles_rad = np.radians(np.linspace(0, 360, num_points, endpoint=False))
        x_trunk, y_trunk, x_sensor, y_sensor = [], [], [], []

        for i in range(num_points):
            xs = self.R_trajectory * np.cos(angles_rad[i])
            ys = self.R_trajectory * np.sin(angles_rad[i])
            x_sensor.append(xs); y_sensor.append(ys)
            r = self.R_trajectory - raw_A[i]
            x_trunk.append(r * np.cos(angles_rad[i]))
            y_trunk.append(r * np.sin(angles_rad[i]))

        fig = plt.figure(figsize=(12, 12))
        plt.plot(0, 0, 'kx', markersize=12, markeredgewidth=2, label='Orbit Center (0,0)')
        plt.plot(x_sensor, y_sensor, 'r--', alpha=0.6, label=f'Circular ToF Path (R={self.R_trajectory})')
        plt.plot(x_sensor[0], y_sensor[0], 'b^', markersize=12, label='START Position')
        mid_idx = num_points // 2
        plt.plot(x_sensor[mid_idx], y_sensor[mid_idx], 'rs', markersize=8, fillstyle='none', label='MID Position')
        arrow_idx = min(5, num_points - 1)
        plt.annotate('', xy=(x_sensor[arrow_idx], y_sensor[arrow_idx]), xytext=(x_sensor[0], y_sensor[0]),
                     arrowprops=dict(arrowstyle='->', color='blue', lw=2, connectionstyle="arc3,rad=.2"))
        plt.plot(np.append(x_trunk, x_trunk[0]), np.append(y_trunk, y_trunk[0]),
                 'go-', markersize=4, label='Mapped Bark Contour')
        plt.fill(x_trunk, y_trunk, 'g', alpha=0.15)
        for i in range(num_points):
            plt.annotate('', xy=(x_trunk[i], y_trunk[i]), xytext=(x_sensor[i], y_sensor[i]),
                         arrowprops=dict(arrowstyle='<->', color='blue', lw=1, alpha=0.5))
        plt.axis('equal')
        plt.title("Full 360° B-Scan: Bark Contour from Circular ToF Orbit")
        plt.xlabel("X position (cm)")
        plt.ylabel("Y position (cm)")
        plt.grid(True, linestyle=':', alpha=0.5)
        plt.legend(loc='upper right')

        buf = io.BytesIO()
        plt.savefig(buf, format='png', dpi=400, bbox_inches='tight')
        plt.close(fig)
        return x_trunk, y_trunk, base64.b64encode(buf.getvalue()).decode('utf-8')

    # ──────────────────────────────────────────────
    # HELPERS
    # ──────────────────────────────────────────────

    def _log_raw_data(self, raw_A):
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        try:
            with open(self.log_path, "a") as f:
                f.write(f"{timestamp},{','.join(map(str, raw_A))}\n")
        except Exception as e:
            print(f"[{self.name}] ERROR logging: {e}")

    def _get_next_tof_index(self, path_tof):
        if not os.path.exists(path_tof):
            return 0
        with open(path_tof, 'r') as f:
            return sum(1 for row in csv.reader(f) if row)

    def get_status(self):
        return {"position": self.position, "moving": self.is_moving}
