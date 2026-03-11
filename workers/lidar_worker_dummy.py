import threading

class LidarThread(threading.Thread):
    def __init__(self, name):
        super().__init__(name=name)
        self.daemon = True

    def run(self):
        # Continuous scanning logic here
        pass

    def process_data(self, path):
        """
        Logic to process the scan file at the given path.
        Returns the calculated xc, yc, and r.
        """
        print(f"[LidarWorker] Processing file at: {path}")
        
        # --- Mock Calculation ---
        # In reality, you'd read the file and do math here
        xc, yc, r = 10.5, 20.2, 5.0 
        
        return xc, yc, r