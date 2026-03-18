import sys
import argparse
import configparser
import importlib 
from core.initialize import initsubsystems
from api.gateway import start_api_gateway, app

# Map config keys to their file locations and class names
WORKER_DEFINITIONS = {
    "lidar": {"module": "workers.lidar_worker", "class": "LidarThread"},
    "vna":   {"module": "workers.vna_worker",   "class": "VNAThread"},
    "robot": {"module": "workers.robot_worker", "class": "RobotThread"},
    "gui":   {"module": "workers.gui_worker",   "class": "GUIThread"}
}

def main():
    parser = argparse.ArgumentParser(description="Boss Thread Orchestrator")
    parser.add_argument("--config", required=True, help="Path to worker.cfg")
    args = parser.parse_args()

    config = configparser.ConfigParser()
    config.read(args.config)

    # 1. Init Subsystems (Blocking)
    print("Boss: Initializing hardware subsystems...")
    # initsubsystems()

    # 2. Dynamic Worker Loading
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
    
    # 3. Link Workers to Flask app.config
    for worker in active_threads:
        config_key = worker.name.replace("-", "_").upper()  # e.g. LIDAR_WORKER
        app.config[config_key] = worker
        print(f"Boss: Linked {worker.name} to app.config[{config_key}]")

    # 4. Load [PATHS] from ini into Flask app.config with UPPERCASE keys
    #    configparser lowercases keys by default, so .upper() is required
    #    e.g. lidar_data -> LIDAR_DATA, tof_record -> TOF_RECORD
    for key, value in config.items('PATHS'):
        app.config[key.upper()] = value
        print(f"Boss: Path loaded app.config[{key.upper()}] = {value}")

    # 5. Start API Gateway (Blocking Backbone)
    try:
        start_api_gateway(host="127.0.0.1", port=3000)
    except Exception as e:
        print(f"Boss: API Gateway failed: {e}")
    finally:
        print("Boss: Shutting down all systems.")

if __name__ == "__main__":
    main()