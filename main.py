import sys
import time
import argparse
import configparser
import importlib
import subprocess
import webbrowser
import os
# from core.initialize import initsubsystems
from api.gateway import start_api_gateway, app

WORKER_DEFINITIONS = {
    "lidar": {"module": "workers.lidar_worker", "class": "LidarThread"},
    "vna":   {"module": "workers.vna_worker",   "class": "VNAThread"},
    "robot": {"module": "workers.robot_worker", "class": "RobotThread"},
    "gui":   {"module": "workers.gui_worker",   "class": "GUIThread"},
    "tof":   {"module": "workers.tof_worker",   "class": "TOFThread"}
}

GUI_DIR  = os.path.join(os.path.dirname(__file__), "gui")
GUI_URL  = "http://localhost:5173"
GUI_PORT = 5173

def launch_gui():
    """Starts 'npm run dev' in /gui as a background subprocess.
    Returns the Popen handle so main can kill it on shutdown.
    """
    # Check gui/ folder exists
    if not os.path.isdir(GUI_DIR):
        print("Boss: /gui folder not found — skipping GUI launch.")
        return None

    # Pick the right npm command for Windows vs Unix
    npm_cmd = "npm.cmd" if sys.platform == "win32" else "npm"

    print(f"Boss: Starting React dev server in {GUI_DIR} ...")
    proc = subprocess.Popen(
        [npm_cmd, "run", "dev"],
        cwd=GUI_DIR,
        stdout=subprocess.DEVNULL,   # suppress npm output from main console
        stderr=subprocess.DEVNULL,
    )
    print(f"Boss: React dev server started (pid {proc.pid})")

    # Give Vite ~2s to boot, then open browser automatically
    time.sleep(2)
    webbrowser.open(GUI_URL)
    print(f"Boss: Browser opened at {GUI_URL}")

    return proc


def main():
    parser = argparse.ArgumentParser(description="Boss Thread Orchestrator")
    parser.add_argument("--config", required=True, help="Path to worker.cfg")
    args = parser.parse_args()

    config = configparser.ConfigParser()
    config.read(args.config)

    # 1. Init Subsystems
    print("Boss: Initializing hardware subsystems...")
    # initsubsystems()

    # 2. Launch React GUI (non-blocking subprocess)
    gui_proc = launch_gui()

    # 3. Dynamic Worker Loading
    active_threads = []

    for key, info in WORKER_DEFINITIONS.items():
        if config.getboolean('WORKERS', key, fallback=False):
            try:
                module = importlib.import_module(info["module"])
                worker_class = getattr(module, info["class"])

                thread_instance = worker_class(name=f"{key.upper()}-Worker")
                thread_instance.daemon = True
                thread_instance.start()

                active_threads.append(thread_instance)
                print(f"Boss: {key.upper()} thread imported and started.")

            except (ImportError, AttributeError) as e:
                print(f"Boss: Failed to load {key.upper()}! Error: {e}")

    # 4. Link Workers to Flask app.config
    for worker in active_threads:
        config_key = worker.name.replace("-", "_").upper()
        app.config[config_key] = worker
        print(f"Boss: Linked {worker.name} to app.config[{config_key}]")

    # 5. Load [PATHS] into Flask app.config
    for key, value in config.items('PATHS'):
        app.config[key.upper()] = value
        print(f"Boss: Path loaded app.config[{key.upper()}] = {value}")

    # 6. Start Flask API Gateway (blocking — stays here until killed)
    try:
        start_api_gateway(host="127.0.0.1", port=3000)
    except Exception as e:
        print(f"Boss: API Gateway failed: {e}")
    finally:
        # 7. Kill React dev server when Flask exits
        if gui_proc and gui_proc.poll() is None:
            print("Boss: Stopping React dev server...")
            gui_proc.terminate()
        print("Boss: Shutting down all systems.")


if __name__ == "__main__":
    main()