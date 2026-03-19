import threading
import time
import os
import csv
import numpy as np
from itertools import zip_longest

try:
    import pyvisa
    _PYVISA_AVAILABLE = True
except ImportError:
    _PYVISA_AVAILABLE = False

try:
    import winsound
    _WINSOUND_AVAILABLE = True
except ImportError:
    _WINSOUND_AVAILABLE = False


# ── Module-level helpers ──────────────────────────────────────────────────────

def bscan(count):
    """Return evenly-spaced angle array for a B-scan with `count` steps."""
    step = 360 / count
    return np.array([i * step for i in range(1, count + 1)])


def env_creation(env_name, start=1, retry_flag=False):
    """Create and return a uniquely-numbered output folder."""
    idx, last_folder = start, None
    while True:
        folder = f'{env_name}_{idx}'
        if not os.path.exists(folder):
            if retry_flag:
                return last_folder
            os.mkdir(folder)
            break
        idx += 1
        last_folder = folder
    return folder


def get_last_existing_number(folder_name):
    """Return the highest numeric CSV index in `folder_name`, or None."""
    try:
        files   = os.listdir(folder_name)
        numbers = [int(f.split('.')[0]) for f in files
                   if f.endswith('.csv') and f != 'air.csv']
        return max(numbers) if numbers else None
    except Exception as e:
        print(f"[get_last_existing_number] {e}")
        return None


# ── Thread class ──────────────────────────────────────────────────────────────

class VNAThread(threading.Thread):
    """
    Background thread that owns one VNA connection.

    run()          — connects to the VNA and idles.
    collect_data() — called externally (from the rotator loop or API)
                     to capture one A-scan sweep and write it to a CSV.
    get_data()     — returns connection status or last sweep result.

    Usage
    -----
        worker = VNAThread(
            name      = "VNA-Worker-1",
            addr      = 'TCPIP0::localhost::hislip_PXI10...',
            vna_model = 'P5021A',
        )
        worker.start()          # connects to VNA in background

        # Caller drives the rotator and triggers each A-scan:
        for i, angle in enumerate(bscan(72)):
            move_rotator(angle)
            ev = threading.Event()
            worker.collect_data(f'scan_1/{i+1}.csv', done_event=ev)
            ev.wait()

        worker.get_data()       # {'vna_status': 'connected', ...}
    """

    def __init__(self, name,
                 addr='TCPIP0::localhost::hislip_PXI10_CHASSIS1_SLOT1_INDEX0::INSTR',
                 vna_model= 'P5021A',
                 tolerances=None, max_attempts=100,
                 test_mode=False):
        super().__init__(name=name)
        self.daemon = True

        self._addr         = addr
        self._vna_model    = vna_model
        self._tolerances   = tolerances if tolerances is not None else [0.5, 99, 99, 99]
        self._max_attempts = max_attempts
        self._test_mode    = test_mode

        self._vna          = None
        self._last_result  = None

    # ------------------------------------------------------------------ #
    #  run() — connects to VNA then idles                                 #
    # ------------------------------------------------------------------ #
    def run(self):
        print(f"[{self.name}] VNA thread active...")
        if not self._test_mode:
            self._vna = self._connect_vna(self._addr, self._vna_model)
            if self._vna is None:
                print(f"[{self.name}] Could not connect to VNA. Thread exiting.")
                return
        else:
            print(f"[{self.name}] test_mode=True — skipping VNA connection.")

    # ------------------------------------------------------------------ #
    #  get_data() — return connection status or last sweep result         #
    # ------------------------------------------------------------------ #
    def get_data(self):
        """Return the last collect_data() result, or a live status dict."""
        if self._last_result is not None:
            return self._last_result
        return {"vna_status": "connected" if self._vna else "disconnected",
                "frequency": "5GHz"}

    # ------------------------------------------------------------------ #
    #  collect_data() — single A-scan sweep → CSV                        #
    # ------------------------------------------------------------------ #
    def collect_data(self, filename, done_event=None):
        """
        Capture one A-scan sweep and write it to `filename`.

        Called externally — by the rotator control loop, by the REST API,
        or anywhere else that manages the measurement sequence.
        Always calls done_event.set() in finally so the caller can block on it.
        """
        try:
            real, imag = self._run_until_convergence(
                numtrace     = 4,
                tolerances   = self._tolerances,
                max_attempts = self._max_attempts,
            )
            if real is None or imag is None:
                print(f"[{self.name}] collect_data: convergence failed — {filename} not written.")
                return

            # CSV write — zip_longest pattern from reference script
            data        = [x for pair in zip(real, imag) for x in pair]
            export_data = zip_longest(*data, fillvalue='')
            with open(filename, 'w', encoding='ISO-8859-1', newline='') as f:
                csv.writer(f).writerows(export_data)

            print(f"[{self.name}] Saved → {filename}")
            self._last_result = {"filename": filename, "vna_status": "connected"}

        except Exception:
            import traceback; traceback.print_exc()
        finally:
            if done_event:
                done_event.set()

    # ------------------------------------------------------------------ #
    #  Private — VNA communication                                        #
    # ------------------------------------------------------------------ #
    def _connect_vna(self, addr, vna_model):
        if not _PYVISA_AVAILABLE:
            print(f"[{self.name}] pyvisa not installed.")
            return None
        try:
            print(f"[{self.name}] Connecting to: {addr}")
            rm  = pyvisa.ResourceManager()
            vna = rm.open_resource(addr, timeout=5000)
            resp = vna.query('*IDN?')
            if vna_model in resp:
                print(f"[{self.name}] Connected to Keysight VNA.")
                vna.write('FORMat REAL,64')
                vna.write('FORMat:BORDer SWAP')
                vna.write('SENS:SWE:MODE HOLD')
                time.sleep(0.1)
                return vna
            else:
                print(f"[{self.name}] Model mismatch. Got: {resp}")
                self._alarm()
                return None
        except Exception as e:
            print(f"[{self.name}] VNA connect error: {e}")
            self._alarm()
            return None

    def _capture_and_process_data(self, numtrace):
        trace_indices = ",".join(str(i) for i in range(1, numtrace + 1))
        self._vna.write("SENS:SWE:MODE SING")
        self._vna.query("*OPC?")

        data       = self._vna.query_binary_values(
                         f'CALC:DATA:MSD? "{trace_indices}"', datatype='d')
        traces     = [data[len(data)//numtrace*i : len(data)//numtrace*(i+1)]
                      for i in range(numtrace)]
        real_parts = np.array([t[::2]  for t in traces])
        imag_parts = np.array([t[1::2] for t in traces])
        magnitudes = np.array([
            np.abs(np.array(r) + 1j * np.array(im))
            for r, im in zip(real_parts, imag_parts)
        ])
        return real_parts, imag_parts, magnitudes

    def _calc_rel_error(self, previous_magnitudes, magnitudes):
        return [
            np.mean(np.abs(curr - prev) / np.array(curr)) * 100
            for prev, curr in zip(previous_magnitudes, magnitudes)
        ]

    def _calc_rmse(self, previous_magnitudes, magnitudes):
        return [
            100 * np.mean(np.square(curr - prev)) / np.mean(prev)
            for prev, curr in zip(previous_magnitudes, magnitudes)
        ]

    def _run_until_convergence(self, numtrace=4, tolerances=None, max_attempts=20):
        if tolerances is None:
            tolerances = [1] * numtrace

        previous_magnitudes = previous_real = previous_imag = None

        for attempt in range(max_attempts):
            print(f"[{self.name}] Starting Attempt: {attempt + 1}")
            real, imag, magnitudes = self._capture_and_process_data(numtrace)

            if previous_magnitudes is not None:
                e_percentages = self._calc_rel_error(previous_magnitudes, magnitudes)

                for i, e in enumerate(e_percentages):
                    print(f"[{self.name}] Trace {i+1}: Error% = {e:.2f}%")

                converged = all(e <= tol for e, tol in zip(e_percentages, tolerances))

                if converged:
                    print(f"[{self.name}] Converged after {attempt + 1} attempts.")
                    mean_real = [(r + pr) / 2 for r, pr in zip(real, previous_real)]
                    mean_imag = [(im + pi) / 2 for im, pi in zip(imag, previous_imag)]
                    return mean_real, mean_imag

            if attempt > 0:
                print(f"[{self.name}] Attempt {attempt + 1}: Magnitudes not converged.")

            previous_magnitudes = magnitudes
            previous_real       = real
            previous_imag       = imag

        print(f"[{self.name}] Maximum attempts reached. Convergence not achieved.")
        print(f"[{self.name}] ERROR: Not enough data collected for B-scan")
        self._alarm()
        return None, None

    def _alarm(self):
        if _WINSOUND_AVAILABLE:
            for _ in range(5):
                winsound.Beep(1000, 300)
                time.sleep(0.1)
        else:
            print(f"[{self.name}] [ALARM]")

    def _cleanup(self):
        if self._vna is not None:
            try:
                self._vna.close()
            except Exception:
                pass
            self._vna = None


# ── Standalone usage ──────────────────────────────────────────────────────────
if __name__ == "__main__":
    ADDR      = 'TCPIP0::localhost::hislip_PXI10_CHASSIS1_SLOT1_INDEX0::INSTR'
    MODEL     = 'P5021A'
    NAME      = 'minus_30deg_cross'
    COUNT     = 72
    RETRY     = False

    # Start VNA worker — connects in background
    worker = VNAThread(
        name         = "VNA-Worker-1",
        addr         = ADDR,
        vna_model    = MODEL,
        tolerances   = [0.5, 99, 99, 99],
        max_attempts = 100,
    )
    worker.start()

    # Caller creates folder and drives the rotator loop externally
    folder      = env_creation(NAME, start=1, retry_flag=RETRY)
    last_number = get_last_existing_number(folder) or 0 if RETRY else 0
    angles      = bscan(COUNT)

    start = time.time()
    for i in range(last_number, COUNT):
        # move_rotator(angles[i])   ← your rotator call goes here
        print(f"\nInterval: {i + 1}  angle: {angles[i]:.2f} deg")
        starttime = time.time()

        ev = threading.Event()
        worker.collect_data(os.path.join(folder, f'{i + 1}.csv'), done_event=ev)
        ev.wait()

        print(f"A-scan time: {time.time() - starttime:.2f}s")

    print(f"\nB-scan complete in {time.time() - start:.2f}s")
    print(worker.get_data())
    worker._cleanup()