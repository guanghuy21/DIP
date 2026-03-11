import threading
import subprocess
import os
import platform

class GUIThread(threading.Thread):
    def __init__(self, name):
        super().__init__(name=name)
        self.daemon = True
        self.process = None
        # Path to your React project folder
        self.gui_path = os.path.join(os.getcwd(), "gui_frontend") 

    def run(self):
        print(f"[{self.name}] Initializing React Frontend on port 5000...")
        
        # Set the port for React
        env = os.environ.copy()
        env["PORT"] = "5000"
        env["BROWSER"] = "none"  # Prevents auto-opening browser on every boot

        try:
            # Check if directory exists before running
            if not os.path.exists(self.gui_path):
                print(f"[{self.name}] ERROR: GUI folder '{self.gui_path}' not found.")
                return

            # Use shell=True for Windows to handle npm command resolution
            cmd = "npm start" if platform.system() == "Windows" else ["npm", "start"]
            
            self.process = subprocess.Popen(
                cmd,
                cwd=self.gui_path,
                env=env,
                shell=(platform.system() == "Windows")
            )
            print(f"[{self.name}] React process started (PID: {self.process.pid})")
            
            # Wait for the process to finish (it won't unless crashed/stopped)
            self.process.wait()

        except Exception as e:
            print(f"[{self.name}] Failed to start GUI: {e}")

    def stop(self):
        if self.process:
            self.process.terminate()
            print(f"[{self.name}] React process terminated.")