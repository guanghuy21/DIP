import threading
import time
import numpy as np
import matplotlib.pyplot as plt
import io
import base64
import os
import csv

class RobotThread(threading.Thread):
    def __init__(self, name):
        super().__init__(name=name)
        self.daemon = True
        self.R_trajectory = 40  # cm
        self.position = {"x": 0, "y": 0, "z": 0}
        self.is_moving = False

        # Define log path
        self.log_path = "data/tof_logs/tof.log"
        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)

    def run(self):
        print(f"[{self.name}] Robot controller active...")
        # while True:
        #     time.sleep(1)

    def _log_raw_data(self, raw_A):
        """Logs raw data with a timestamp to data/tof_logs/tof.log."""
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        data_string = ",".join(map(str, raw_A))
        try:
            with open(self.log_path, "a") as f:
                f.write(f"{timestamp},{data_string}\n")
        except Exception as e:
            print(f"[{self.name}] ERROR Logging Data: {e}")

    def _read_tof(self):
        """Reads a single ToF distance measurement from the sensor. Returns float (cm)."""
        # TODO: replace with real sensor read, e.g. serial/I2C call
        raise NotImplementedError("Implement real ToF sensor read here")

    def process_data(self, raw_A=None, path=None):
        """Maps ToF data and generates a B-Scan image.
        
        Accepts either:
          - raw_A: list of ToF readings directly (for testing)
          - path:  path to tof.csv (for production)
        """

        # 1. Load data — raw list takes priority over file path
        if raw_A is None:
            if path is None:
                print(f"[{self.name}] ERROR: provide either raw_A or path.")
                return [], [], None
            raw_A = []
            try:
                with open(path, 'r') as f:
                    reader = csv.reader(f)
                    for row in reader:
                        if row:
                            raw_A.append(float(row[1]))  # col 0 = index, col 1 = tof_cm
            except Exception as e:
                print(f"[{self.name}] ERROR reading ToF CSV: {e}")
                return [], [], None

        self._log_raw_data(raw_A)

        num_points = len(raw_A)
        if num_points < 2:
            print(f"[{self.name}] Not enough ToF points to plot.")
            return [], [], None

        # 1. Assign angles for full 360° rotation
        angles_rad = np.radians(np.linspace(0, 360, num_points, endpoint=False))

        # 2. Map coordinates
        x_trunk, y_trunk = [], []
        x_sensor, y_sensor = [], []

        for i in range(num_points):
            xs = self.R_trajectory * np.cos(angles_rad[i])
            ys = self.R_trajectory * np.sin(angles_rad[i])
            x_sensor.append(xs)
            y_sensor.append(ys)

            r_trunk = self.R_trajectory - raw_A[i]
            x_trunk.append(r_trunk * np.cos(angles_rad[i]))
            y_trunk.append(r_trunk * np.sin(angles_rad[i]))

        # 3. Plot
        fig = plt.figure(figsize=(12, 12))

        plt.plot(0, 0, 'kx', markersize=12, markeredgewidth=2, label='Orbit Center (0,0)')
        plt.plot(x_sensor, y_sensor, 'r--', alpha=0.6, label=f'Circular ToF Path (R={self.R_trajectory})')
        plt.plot(x_sensor[0], y_sensor[0], 'b^', markersize=12, label='START Position')

        mid_idx = num_points // 2
        plt.plot(x_sensor[mid_idx], y_sensor[mid_idx], 'rs', markersize=8, fillstyle='none', label='MID Position')

        # Direction arrow
        arrow_idx = min(5, num_points - 1)
        plt.annotate('', xy=(x_sensor[arrow_idx], y_sensor[arrow_idx]), xytext=(x_sensor[0], y_sensor[0]),
                     arrowprops=dict(arrowstyle='->', color='blue', lw=2, connectionstyle="arc3,rad=.2"))

        # Bark contour (closed loop)
        plt.plot(np.append(x_trunk, x_trunk[0]),
                 np.append(y_trunk, y_trunk[0]),
                 'go-', markersize=4, label='Mapped Bark Contour')
        plt.fill(x_trunk, y_trunk, 'g', alpha=0.15)

        # ToF beams
        for i in range(num_points):
            plt.annotate('', xy=(x_trunk[i], y_trunk[i]), xytext=(x_sensor[i], y_sensor[i]),
                         arrowprops=dict(arrowstyle='<->', color='blue', lw=1, alpha=0.5))

        plt.axis('equal')
        plt.title("Full 360° B-Scan: Bark Contour from Circular ToF Orbit")
        plt.xlabel("X position (cm)")
        plt.ylabel("Y position (cm)")
        plt.grid(True, linestyle=':', alpha=0.5)
        plt.legend(loc='upper right')

        # 4. Save — bbox_inches='tight' prevents cropping
        buf = io.BytesIO()
        plt.savefig(buf, format='png', dpi=400, bbox_inches='tight')
        plt.close(fig)

        image_base64 = base64.b64encode(buf.getvalue()).decode('utf-8')
        return x_trunk, y_trunk, image_base64

    def move_to(self, x, y, z, path_tof='tof.csv', done_event=None):
        """Moves the SCARA arm to (x, y, z), takes a ToF reading, and appends it to CSV."""
        self.is_moving = True
        print(f"[{self.name}] Moving to X:{x}, Y:{y}, Z:{z}")
        time.sleep(1)  # Simulate physical movement
        self.position = {"x": x, "y": y, "z": z}
        self.is_moving = False

        # Take ToF reading and append to CSV
        try:
            tof_cm = self._read_tof()
            point_index = self._get_next_tof_index(path_tof)
            os.makedirs(os.path.dirname(path_tof) if os.path.dirname(path_tof) else '.', exist_ok=True)
            with open(path_tof, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([point_index, tof_cm])
            print(f"[{self.name}] ToF reading {tof_cm} cm logged at index {point_index}")
        except NotImplementedError:
            print(f"[{self.name}] WARNING: ToF sensor not implemented, skipping log.")
        except Exception as e:
            print(f"[{self.name}] ERROR logging ToF data: {e}")
        finally:
            if done_event:
                done_event.set()
        return self.position

    def _get_next_tof_index(self, path_tof):
        """Returns the next row index for the ToF CSV."""
        if not os.path.exists(path_tof):
            return 0
        with open(path_tof, 'r') as f:
            return sum(1 for row in csv.reader(f) if row)

    def get_status(self):
        return {"position": self.position, "moving": self.is_moving}