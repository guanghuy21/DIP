import threading
import time

class VNAThread(threading.Thread):
    def __init__(self, name):
        super().__init__(name=name)
        self.daemon = True

    def run(self):
        print(f"[{self.name}] Dummy VNA thread is idling...")
        while True:
            time.sleep(10)

    def get_data(self):
        # Mock method for the API to call later
        return {"vna_status": "connected", "frequency": "5GHz"}
    def collect_data(self, filename, done_event=None):
        try:
            ...  # collect vna sweep
        finally:
            if done_event:
                done_event.set()