import threading
import time
import serial
import numpy as np
import matplotlib.pyplot as plt
import io
import base64
import os
import csv

BAUD_RATE    = 115200
TIMEOUT_SEC  = 5      # how long to wait for Arduino reply

class RobotThread(threading.Thread):
    def __init__(self, name):
        super().__init__(name=name)
        self.daemon      = True
        self.R_trajectory = 40
        self.position    = {"x": 0, "y": 0, "z": 0}
        self.is_moving   = False

        # Serial state — configured by connect() before use
        self._serial     = None
        self._serial_lock = threading.Lock()  # prevents two threads writing at the same time
        self.port        = None

        self.log_path = "data/tof_logs/tof.log"
        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)


    # LIFECYCLE

    def run(self):
        """Keep-alive loop. Serial is opened once via connect()."""
        print(f"[{self.name}] Robot controller active...")
        while True:
            # Watchdog: attempt reconnect if connection dropped
            if self.port and (self._serial is None or not self._serial.is_open):
                print(f"[{self.name}] Serial lost — reconnecting...")
                self._connect(self.port)
            time.sleep(2)

    def connect(self, port):
        """Called from the route or main.py to open serial before first use."""
        self.port = port
        self._connect(port)

    def _connect(self, port):
        """Opens serial connection to Arduino."""
        try:
            self._serial = serial.Serial(port, BAUD_RATE, timeout=TIMEOUT_SEC)
            time.sleep(2)  # wait for Arduino to reset after serial open
            print(f"[{self.name}] Connected to Arduino on {port}")
        except serial.SerialException as e:
            self._serial = None
            print(f"[{self.name}] ERROR connecting to Arduino: {e}")

    # ──────────────────────────────────────────────
    # SERIAL PRIMITIVES
    # ──────────────────────────────────────────────

    def _send_command(self, cmd: str) -> str:
        """Sends a command string to Arduino and returns the response line.
        
        Thread-safe via _serial_lock — only one command in flight at a time.
        
        Returns response string, or raises RuntimeError on failure.
        """
        if self._serial is None or not self._serial.is_open:
            raise RuntimeError("Serial port not open")

        with self._serial_lock:
            try:
                self._serial.reset_input_buffer()
                self._serial.write(f"{cmd}\n".encode('utf-8'))
                response = self._serial.readline().decode('utf-8').strip()
                print(f"[{self.name}] >> {cmd}  |  << {response}")
                return response
            except serial.SerialException as e:
                raise RuntimeError(f"Serial error: {e}")

    # ARDUINO COMMANDS

    def _read_tof(self) -> float:
        """Sends READ_TOF to Arduino, parses 'TOF:18.4' response → float cm."""
        response = self._send_command("READ_TOF")
        if response.startswith("TOF:"):
            return float(response.split(":")[1])
        raise RuntimeError(f"Unexpected ToF response: {response}")

    def _move_arm(self, x, y, z) -> bool:
        """Sends MOVE command, waits for ACK from Arduino.
        
        Arduino expected response: 'ACK' on success, 'ERR:<reason>' on failure.
        """
        response = self._send_command(f"MOVE {x:.2f} {y:.2f} {z:.2f}")
        if response == "ACK":
            return True
        raise RuntimeError(f"Move rejected by Arduino: {response}")

    # ──────────────────────────────────────────────
    # PUBLIC WORKER METHODS
    # ──────────────────────────────────────────────

    def move_to(self, x, y, z, path_tof='tof.csv', done_event=None):
        """Moves SCARA arm to (x, y, z), reads ToF, appends to CSV."""
        self.is_moving = True
        try:
            # 1. Send move command — blocks until Arduino ACKs
            self._move_arm(x, y, z)
            self.position = {"x": x, "y": y, "z": z}

            # 2. Read ToF from Arduino
            tof_cm = self._read_tof()

            # 3. Append to CSV
            point_index = self._get_next_tof_index(path_tof)
            os.makedirs(os.path.dirname(path_tof) if os.path.dirname(path_tof) else '.', exist_ok=True)
            with open(path_tof, 'a', newline='') as f:
                csv.writer(f).writerow([point_index, tof_cm])
            print(f"[{self.name}] ToF {tof_cm} cm logged at index {point_index}")

        except Exception as e:
            print(f"[{self.name}] ERROR in move_to: {e}")
        finally:
            self.is_moving = False
            if done_event:
                done_event.set()

        return self.position

    def process_data(self, raw_A=None, path=None):
        """Maps ToF data and generates a B-Scan image.

        Accepts either:
          - raw_A: list of ToF readings directly (for testing)
          - path:  path to tof.csv (for production)
        """
        if raw_A is None:
            if path is None:
                print(f"[{self.name}] ERROR: provide either raw_A or path.")
                return [], [], None
            raw_A = []
            try:
                with open(path, 'r') as f:
                    for row in csv.reader(f):
                        if row:
                            raw_A.append(float(row[1]))
            except Exception as e:
                print(f"[{self.name}] ERROR reading ToF CSV: {e}")
                return [], [], None

        self._log_raw_data(raw_A)

        num_points = len(raw_A)
        if num_points < 2:
            print(f"[{self.name}] Not enough ToF points.")
            return [], [], None

        angles_rad = np.radians(np.linspace(0, 360, num_points, endpoint=False))
        x_trunk, y_trunk, x_sensor, y_sensor = [], [], [], []

        for i in range(num_points):
            xs = self.R_trajectory * np.cos(angles_rad[i])
            ys = self.R_trajectory * np.sin(angles_rad[i])
            x_sensor.append(xs)
            y_sensor.append(ys)
            r_trunk = self.R_trajectory - raw_A[i]
            x_trunk.append(r_trunk * np.cos(angles_rad[i]))
            y_trunk.append(r_trunk * np.sin(angles_rad[i]))

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
        timestamp   = time.strftime("%Y%m%d-%H%M%S")
        data_string = ",".join(map(str, raw_A))
        try:
            with open(self.log_path, "a") as f:
                f.write(f"{timestamp},{data_string}\n")
        except Exception as e:
            print(f"[{self.name}] ERROR Logging Data: {e}")

    def _get_next_tof_index(self, path_tof):
        if not os.path.exists(path_tof):
            return 0
        with open(path_tof, 'r') as f:
            return sum(1 for row in csv.reader(f) if row)

    def get_status(self):
        return {"position": self.position, "moving": self.is_moving}