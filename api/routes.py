from flask import Blueprint, jsonify, request, current_app
import os
import threading

# Create the blueprint
api_blueprint = Blueprint('api', __name__)



@api_blueprint.route('/hello', methods=['GET'])
def hello():
    return jsonify({"answer": "how are you"}), 200

# You can add more routes here easily
@api_blueprint.route('/status', methods=['GET'])
def status():
    return jsonify({"status": "All systems nominal"}), 200

@api_blueprint.route('/lidar/process_data', methods=['POST'])
def lidar_process_data():
    data = request.get_json()
    path = data.get("path")
    path = os.path.join(current_app.config.get('LIDAR_DATA') , path) if path else "raw.csv"
    
    if not path:
        return jsonify({"error": "No path provided"}), 400

    # 2. Access the Lidar worker instance (passed from the Boss)
    lidar_worker = current_app.config.get('LIDAR_WORKER')
    
    if not lidar_worker:
        return jsonify({"error": "Lidar worker not initialized"}), 500

    # 3. Call the worker method
    try:
        points, r, plot_b64 = lidar_worker.process_data(path=path)
        return jsonify({"points": points, "r": r, "image": plot_b64}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    

@api_blueprint.route('/lidar/scan', methods=['POST'])
def lidar_scan_data():
    lidar_worker = current_app.config.get('LIDAR_WORKER')
    if not lidar_worker:
        return jsonify({"error": "Lidar worker not initialized"}), 500

    lidar_data_path = current_app.config.get('LIDAR_DATA')
    if not lidar_data_path:
        return jsonify({"error": "LIDAR_DATA path not configured"}), 500

    path = os.path.join(lidar_data_path, 'raw.csv')

    data                = request.get_json()
    port                = data.get("port")
    radius_cm           = data.get("radius_cm")
    distance_cm         = data.get("distance_cm")       # None → triggers auto-detect in scan()
    safety_margin_deg   = data.get("safety_margin_deg")
    scan_duration_sec   = data.get("scan_duration_sec")
    circle_toleration_mm = data.get("circle_toleration_mm")

    # Block until scan() finishes (done_event.set() called in scan()'s finally block)
    done = threading.Event()
    lidar_worker.scan(
        port,
        radius_cm,
        distance_cm,
        safety_margin_deg,
        scan_duration_sec,
        circle_toleration_mm,
        done_event=done
    )
    done.wait()

    points, r_exp, img = lidar_worker.process_data(path=path)

    return jsonify({
        # "port":                 port,
        # "distance_cm":          distance_cm,
        # "safety_margin_deg":    safety_margin_deg,
        # "scan_duration_sec":    scan_duration_sec,
        # "circle_toleration_mm": circle_toleration_mm,
        "r_exp":                r_exp,
        "points":               points,
        "msg":                  "Scan complete",
        "bscan_image":          img
    }), 200

    
@api_blueprint.route('/robot/status', methods=['GET'])
def robot_status():
    robot = current_app.config.get('ROBOT_WORKER')
    if not robot:
        return jsonify({"error": "exitcode:404", "msg": "Robot thread not running"}), 404
    return jsonify(robot.get_status()), 200

@api_blueprint.route('/robot/tof/process_data', methods=['GET'])
def receive_tof_data():
    # Access the shared robot worker thread
    robot = current_app.config.get('ROBOT_WORKER')
    
    if not robot:
        return jsonify({"error": "Robot Worker not initialized"}), 500

    # In production, use: data = request.json.get('data')
    # For now, using your provided test data
    raw_data = [20, 19, 18, 18, 17, 16, 16, 18, 18, 18, 16, 16, 17, 20, 22, 21, 21, 21, 18, 15, 13, 12 , 13 , 18, 19 ,21, 21, 21, 20, 18, 17, 16, 16, 20, 21]

    try:
        x_pts, y_pts, image64 = robot.process_data(raw_data)
        
        # Prepare points as a list of dictionaries for the JSON response
        points = [{"x": round(x, 2), "y": round(y, 2)} for x, y in zip(x_pts, y_pts)]

        return jsonify({
            "status": "success",
            "points": points, 
            "image": image64
        }), 200

    except Exception as e:
        return jsonify({"error": str(e)}), 500
    
@api_blueprint.route('/robot/move', methods=['POST'])
def robot_move():
    robot = current_app.config.get('ROBOT_WORKER')
    # lidar_worker = current_app.config.get('LIDAR_WORKER')
    
    # points, rcs, img = lidar_worker.process_data(path)
    if not robot:
        return jsonify({"error": "exitcode:404"}), 404
    
    data = request.get_json()
    x = data.get('x', 0)
    y = data.get('y', 0)
    z = data.get('z', 0)
    
    new_pos = robot.move_to(x, y, z)
    return jsonify({"message": "Move successful", "position": new_pos}), 200



@api_blueprint.route('/start_run', methods=['POST'])
def start_run():
    lidar_worker = current_app.config.get('LIDAR_WORKER')
    robot_worker = current_app.config.get('ROBOT_WORKER')
    vna_worker   = current_app.config.get('VNA_WORKER')

    data   = request.get_json()
    port   = data.get("port")
    radius = data.get("radius")

    if radius:
        robot_worker.R_trajectory = float(radius)

    # 1. LiDAR scan — block until done
    done = threading.Event()
    lidar_worker.scan(port=port, done_event=done)
    done.wait()

    path_to_lidar = os.path.join(current_app.config.get('LIDAR_DATA'), 'raw.csv')
    points, r_exp, _ = lidar_worker.process_data(path=path_to_lidar)

    # 2. Robot + VNA — block on every move then every VNA collect
    path_to_tof = os.path.join(current_app.config.get('TOF_RECORD'), 'tof.csv')
    vna_results = []

    # Reset tof.csv before each run
    os.makedirs(os.path.dirname(path_to_tof), exist_ok=True)
    open(path_to_tof, 'w').close()

    for i, point in enumerate(points):
        x, y, z = point["x"], point["y"], point["z"]

        # Block until arm has moved AND tof is logged
        done = threading.Event()
        robot_worker.move_to(x, y, z, path_tof=path_to_tof, done_event=done)
        done.wait()

        # Block until VNA collection is done
        done = threading.Event()
        vna_worker.collect_data(f'{i + 1}.csv', done_event=done)
        done.wait()

        vna_results.append(f'{i + 1}.csv')

    # 3. Generate B-Scan image
    x_trunk, y_trunk, img = robot_worker.process_data(path=path_to_tof)

    return jsonify({
        "msg": "Run complete",
        "bscan_image": img,
        "vna_files": vna_results
    }), 200

# --- VNA TESTING ---
@api_blueprint.route('/vna/data', methods=['GET'])
def vna_data():
    vna = current_app.config.get('VNA_WORKER')
    if not vna:
        return jsonify({"error": "exitcode:404"}), 404
    return jsonify(vna.get_data()), 200

# --- GUI TESTING ---
@api_blueprint.route('/gui/health', methods=['GET'])
def gui_health():
    gui = current_app.config.get('GUI_WORKER')
    if not gui:
        return jsonify({"error": "exitcode:404"}), 404
    
    # Check if the subprocess is actually running
    is_alive = gui.process is not None and gui.process.poll() is None
    return jsonify({
        "gui_service": "React",
        "port": 5000,
        "process_running": is_alive
    }), 200

@api_blueprint.route('/debug/config', methods=['GET'])
def debug_config():
    # List all keys in config that end with _WORKER
    worker_keys = [k for k in current_app.config.keys() if "_WORKER" in k]
    return jsonify({"active_workers_in_config": worker_keys})

# ── /init — stores config into app.config, connects robot serial ────
@api_blueprint.route('/init', methods=['POST'])
def init():
    data = request.get_json()
 
    lidar_worker = current_app.config.get('LIDAR_WORKER')
    robot_worker = current_app.config.get('ROBOT_WORKER')
 
    if not lidar_worker or not robot_worker:
        return jsonify({"error": "Workers not initialized"}), 500
 
    # Store scan config for use in /run and /process
    current_app.config['SCAN_CONFIG'] = {
        "port":                data.get("port"),
        "radius_mm":           data.get("radius_cm", 5.0) * 10,   # GUI sends cm, worker expects mm
        "distance_cm":         data.get("distance_cm"),            # None → auto-detect
        "safety_margin_deg":   data.get("safety_margin_deg", 5),
        "scan_duration_sec":   data.get("scan_duration_sec", 30),
        "circle_toleration_mm": data.get("circle_tolerance_mm", 50),
    }
 
    # Connect robot serial port
    robot_worker.connect(data.get("port"))
 
    # Update robot trajectory radius
    if data.get("radius_cm"):
        robot_worker.R_trajectory = float(data.get("radius_cm"))
 
    return jsonify({"msg": "Initialized"}), 200
 
 
# ── /run — triggers LiDAR scan ──────────────────────────────────────
@api_blueprint.route('/run', methods=['POST'])
def run():
    lidar_worker = current_app.config.get('LIDAR_WORKER')
    if not lidar_worker:
        return jsonify({"error": "Lidar worker not initialized"}), 500
 
    cfg = current_app.config.get('SCAN_CONFIG')
    if not cfg:
        return jsonify({"error": "Call /init first"}), 400
 
    lidar_data_path = current_app.config.get('LIDAR_DATA')
    if not lidar_data_path:
        return jsonify({"error": "LIDAR_DATA path not configured"}), 500
 
    # Block until scan finishes
    done = threading.Event()
    lidar_worker.scan(
        cfg["port"],
        cfg["radius_mm"],
        cfg["distance_cm"],
        cfg["safety_margin_deg"],
        cfg["scan_duration_sec"],
        cfg["circle_toleration_mm"],
        done_event=done
    )
    done.wait()
 
    return jsonify({"msg": "Scan complete"}), 200
 
 
# ── /process — processes LiDAR data, returns plot + points ─────────
@api_blueprint.route('/process', methods=['POST'])
def process():
    lidar_worker = current_app.config.get('LIDAR_WORKER')
    if not lidar_worker:
        return jsonify({"error": "Lidar worker not initialized"}), 500
 
    path = os.path.join(current_app.config.get('LIDAR_DATA'), 'raw.csv')
 
    points, r_exp, plot_b64 = lidar_worker.process_data(path=path)
 
    return jsonify({
        "plot_b64": plot_b64,   # GUI reads this as data.plot_b64
        "r_exp":    r_exp,      # GUI reads this as data.r_exp
        "points":   points      # GUI reads this as data.points
    }), 200
 