from flask import Blueprint, jsonify, request, current_app
import os
import threading

# Create the blueprint
api_blueprint = Blueprint('api', __name__)

@api_blueprint.route('/init', methods=['POST'])
def init():
    """
    Configure ports and scan parameters.
    Must be called before any robot or scan operations.
    Body: {
        "robot_port": "COM5",     ← Arduino Mega (motors + TOF)
        "lidar_port": "COM3",     ← LiDAR scanner
        "radius_cm": 5,
        "distance_cm": null,
        "safety_margin_deg": 5,
        "scan_duration_sec": 30,
        "circle_tolerance_mm": 50
    }
    """
    data         = request.get_json()
    lidar_worker = current_app.config.get('LIDAR_WORKER')
    robot_worker = current_app.config.get('ROBOT_WORKER')

    if not lidar_worker or not robot_worker:
        return jsonify({"error": "Workers not initialized"}), 500

    # ── Connect robot serial (motors + TOF) ───────────────
    robot_port = data.get("robot_port")
    if robot_port:
        robot_worker.connect(robot_port)
        print(f"[Boss] Robot connected on {robot_port}")
    elif not robot_worker.port:
        return jsonify({"error": "robot_port required — not connected yet"}), 400

    # ── Connect lidar serial ──────────────────────────────
    lidar_port = data.get("lidar_port")
    if lidar_port:
        lidar_worker.port = lidar_port
        print(f"[Boss] Lidar port set to {lidar_port}")

    # ── Store scan config ─────────────────────────────────
    current_app.config['SCAN_CONFIG'] = {
        "port":                 lidar_port or lidar_worker.port,
        "radius_mm":            data.get("radius_cm", 5.0) * 10,
        "distance_cm":          data.get("distance_cm"),
        "safety_margin_deg":    data.get("safety_margin_deg", 5),
        "scan_duration_sec":    data.get("scan_duration_sec", 30),
        "circle_toleration_mm": data.get("circle_tolerance_mm", 50),
    }

    if data.get("radius_cm"):
        robot_worker.R_trajectory = float(data.get("radius_cm"))

    return jsonify({
        "msg":        "Initialized",
        "robot_port": robot_worker.port,
        "lidar_port": lidar_port or lidar_worker.port
    }), 200

@api_blueprint.route('/hello', methods=['GET'])
def hello():
    return jsonify({"answer": "hello, system is good"}), 200

# You can add more routes here easily
@api_blueprint.route('/status', methods=['GET'])
def status():
    return jsonify({"status": "All systems nominal"}), 200

#======Lidar routes===========================================================

@api_blueprint.route('/lidar/scan', methods=['POST'])
def lidar_scan_data():
    '''
    Def:
    Input:
    JSON
    Output:
    '''
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
    
#==================================================================================================    
#======ROBOT ROUTES================================================================================
#==================================================================================================

@api_blueprint.route('/robot/status', methods=['GET'])
def robot_status():
    robot = current_app.config.get('ROBOT_WORKER')
    if not robot:
        return jsonify({"error": "exitcode:404", "msg": "Robot thread not running"}), 404
    return jsonify(robot.get_status()), 200


    
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
    
    done = threading.Event()
    threading.Thread(
        target=robot.move_to,
        kwargs={"x": x, "y": y, "z": z, "done_event": done},
        daemon=True
    ).start()
 
    if not done.wait(timeout=60):
        return jsonify({"error": "Timeout waiting for arm movement"}), 504
 
    return jsonify({
        "message":       "Move complete",
        "position":      robot.position,
        "last_distance": robot.last_distance,
        "distance_cm":   round(robot.last_distance * 0.1, 3) if robot.last_distance else None
    }), 200

#==============================================================================
#======ROBOT/TOF ROUTES========================================================
#==============================================================================
 
@api_blueprint.route('/robot/tof/scan', methods=['POST'])
def tof_scan():
    """
    Take one TOF snapshot.
    Robot worker handles serial — no separate TOF_WORKER needed.
    Request body: { "port": "COM5" }   ← port used only if not yet connected
    """
    robot = current_app.config.get('ROBOT_WORKER')
    if not robot:
        return jsonify({"error": "Robot worker not initialized"}), 500
 
    port = (request.get_json() or {}).get("port", None)
    done = threading.Event()
 
    threading.Thread(
        target=robot.record,
        kwargs={"port": port, "done_event": done},
        daemon=True
    ).start()
 
    if not done.wait(timeout=15):
        return jsonify({"error": "Timed out waiting for TOF reading"}), 504
 
    distance = robot.last_distance
    if distance is None:
        return jsonify({"error": "No reading received from sensor"}), 500
 
    return jsonify({
        "msg":         "Recorded",
        "distance_mm": distance,
        "distance_cm": round(distance * 0.1, 3)
    }), 200
 
 
@api_blueprint.route('/robot/tof/reset', methods=['GET'])
def tof_reset():
    """Clear the TOF CSV log and reset convergence state."""
    robot = current_app.config.get('ROBOT_WORKER')
    if not robot:
        return jsonify({"error": "Robot worker not initialized"}), 500
    try:
        robot.reset_tof()
        return jsonify({"msg": "TOF log reset"}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500
 
 
@api_blueprint.route('/robot/tof/process_data', methods=['POST'])
def tof_process_data():
    """
    Generate B-Scan plot from logged TOF data.
    Request body (all optional):
      { "path": "tof.csv", "r_trajectory": 40, "arc_span": 360 }
    """
    robot = current_app.config.get('ROBOT_WORKER')
    if not robot:
        return jsonify({"error": "Robot worker not initialized"}), 500
 
    data         = request.get_json() or {}
    path         = data.get("path")
    r_trajectory = data.get("r_trajectory", None)
    arc_span     = data.get("arc_span", 360)
 
    if path:
        path = os.path.join(current_app.config.get('TOF_RECORD', ''), path)
 
    try:
        x_trunk, y_trunk, plot_b64 = robot.process_data(
            path=path,
            r_trajectory=r_trajectory,
            arc_span=arc_span
        )
        return jsonify({
            "x_trunk": x_trunk,
            "y_trunk": y_trunk,
            "image":   plot_b64
        }), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    
#=============================================================================
#======Test motors, TOF, and both independently before a full run=============
#=============================================================================
 
@api_blueprint.route('/test/motors', methods=['POST'])
def test_motors():
    """
    TEST MOTORS ONLY — moves arm to given position, no TOF reading.
    Use this to verify IK, motor wiring, and movement without sensor.
    Body: { "x": float, "y": float, "z": float }
    Arduino sequence:  x,y,z → MOVING t1,t2,t3 → DONE
    Response: { position, duration_sec }
    """
    robot = current_app.config.get('ROBOT_WORKER')
    if not robot:
        return jsonify({"error": "Robot worker not initialized"}), 500

    data = request.get_json() or {}
    x = data.get('x', 0)
    y = data.get('y', 0)
    z = data.get('z', 0)

    import time
    start_time = time.time()
    try:
        # _move_arm() only — completely bypasses TOF
        # Any TOF connection errors are irrelevant here
        robot._move_arm(x, y, z)
        robot.position = {"x": x, "y": y, "z": z}
        duration = round(time.time() - start_time, 2)

        return jsonify({
            "msg":          "Motors test complete",
            "target":       {"x": x, "y": y, "z": z},
            "position":     robot.position,
            "duration_sec": duration,
            "tof":          "skipped — motor test only"
        }), 200

    except RuntimeError as e:
        # Catches UNREACHABLE and serial timeout
        # Does NOT catch TOF errors since TOF is never called
        return jsonify({
            "error":  str(e),
            "target": {"x": x, "y": y, "z": z},
            "tip":    "UNREACHABLE means IK failed — try coordinates closer to the arm"
        }), 400

    except Exception as e:
        return jsonify({"error": str(e)}), 500
 
 
@api_blueprint.route('/test/tof', methods=['POST'])
def test_tof():
    """
    TEST TOF ONLY — takes one reading, arm does NOT move.
    Use this to verify VL53L0X wiring and sensor readings.
    Body: {} or { "port": "COM5" }
    Arduino sequence:  TOF_START → DATA:millis,mm  (or MSG:OUT_OF_RANGE)
    Response: { distance_mm, distance_cm, converged }
    """
    robot = current_app.config.get('ROBOT_WORKER')
    if not robot:
        return jsonify({"error": "Robot worker not initialized"}), 500
 
    port = (request.get_json() or {}).get("port", None)
    done = threading.Event()
 
    threading.Thread(
        target=robot.record,
        kwargs={"port": port, "done_event": done},
        daemon=True
    ).start()
 
    if not done.wait(timeout=10):
        return jsonify({"error": "Timeout — check TOF sensor wiring and COM port"}), 504
 
    distance = robot.last_distance
    if distance is None:
        return jsonify({
            "error":   "No reading — sensor may be out of range or not connected",
            "tip":     "Check VL53L0X wiring: SDA→pin20, SCL→pin21, VCC→3.3V"
        }), 500
 
    return jsonify({
        "msg":          "TOF test complete",
        "distance_mm":  distance,
        "distance_cm":  round(distance * 0.1, 3),
        "converged":    robot.converged
    }), 200
 
 
@api_blueprint.route('/test/motors_and_tof', methods=['POST'])
def test_motors_and_tof():
    """
    TEST MOTORS + TOF TOGETHER — moves arm then reads TOF.
    Use this to verify the full move_to() pipeline before a real run.
    Body: { "x": float, "y": float, "z": float }
    Arduino sequence:  x,y,z → MOVING → DONE → TOF_START → DATA:millis,mm
    Response: { position, distance_mm, distance_cm, duration_sec }
    """
    robot = current_app.config.get('ROBOT_WORKER')
    if not robot:
        return jsonify({"error": "Robot worker not initialized"}), 500
 
    data = request.get_json() or {}
    x = data.get('x', 0)
    y = data.get('y', 0)
    z = data.get('z', 0)
 
    done  = threading.Event()
    start = __import__('time').time()
 
    threading.Thread(
        target=robot.move_to,
        kwargs={"x": x, "y": y, "z": z, "done_event": done},
        daemon=True
    ).start()
 
    if not done.wait(timeout=60):
        return jsonify({"error": "Timeout — arm may be stuck or TOF not responding"}), 504
 
    duration = round(__import__('time').time() - start, 2)
    distance = robot.last_distance
 
    if distance is None:
        return jsonify({
            "msg":          "Arm moved but TOF reading failed",
            "position":     robot.position,
            "duration_sec": duration,
            "tip":          "Check TOF sensor — arm movement worked correctly"
        }), 207   # 207 = partial success
 
    return jsonify({
        "msg":          "Motors + TOF test complete",
        "position":     robot.position,
        "distance_mm":  distance,
        "distance_cm":  round(distance * 0.1, 3),
        "duration_sec": duration
    }), 200
    
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
# @api_blueprint.route('/init', methods=['POST'])
# def init():
#     data = request.get_json()
 
#     lidar_worker = current_app.config.get('LIDAR_WORKER')
#     robot_worker = current_app.config.get('ROBOT_WORKER')
 
#     if not lidar_worker or not robot_worker:
#         return jsonify({"error": "Workers not initialized"}), 500
 
#     # Store scan config for use in /run and /process
#     current_app.config['SCAN_CONFIG'] = {
#         "port":                data.get("port"),
#         "radius_mm":           data.get("radius_cm", 5.0) * 10,   # GUI sends cm, worker expects mm
#         "distance_cm":         data.get("distance_cm"),            # None → auto-detect
#         "safety_margin_deg":   data.get("safety_margin_deg", 5),
#         "scan_duration_sec":   data.get("scan_duration_sec", 30),
#         "circle_toleration_mm": data.get("circle_tolerance_mm", 50),
#     }
 
#     # Connect robot serial port
#     robot_worker.connect(data.get("port"))
 
#     # Update robot trajectory radius
#     if data.get("radius_cm"):
#         robot_worker.R_trajectory = float(data.get("radius_cm"))
 
#     return jsonify({"msg": "Initialized"}), 200
 
 
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
 
 
#==============================================================================
#======VNA ROUTES==============================================================
#==============================================================================
 
@api_blueprint.route('/vna/data', methods=['GET'])
def vna_data():
    """Return the result of the last collect_data() call, or connection status."""
    vna = current_app.config.get('VNA_WORKER')
    if not vna:
        return jsonify({"error": "exitcode:404"}), 404
    return jsonify(vna.get_data()), 200
 
 
@api_blueprint.route('/vna/status', methods=['GET'])
def vna_status():
    """
    Report thread health and VNA connection state.
 
    Response
    --------
    {
        "thread_alive": true,
        "vna_connected": true,
        "last_result": { ... } | null
    }
    """
    vna = current_app.config.get('VNA_WORKER')
    if not vna:
        return jsonify({"error": "VNA worker not initialized"}), 500
 
    return jsonify({
        "thread_alive": vna.is_alive(),
        "vna_connected": vna._vna is not None,
        "last_result":   vna._last_result,
    }), 200
 
 
@api_blueprint.route('/vna/collect', methods=['POST'])
def vna_collect():
    """
    Trigger a single A-scan sweep and write it to a CSV file.
    Blocks until collect_data() finishes (done_event.set() in finally).
 
    Request body (all optional)
    ----------------------------
    {
        "filename": "1.csv"   // relative to VNA_DATA folder; default: "1.csv"
    }
 
    Response
    --------
    {
        "msg": "A-scan saved",
        "filename": "data/vna/1.csv"
    }
    """
    vna = current_app.config.get('VNA_WORKER')
    if not vna:
        return jsonify({"error": "VNA worker not initialized"}), 500
 
    vna_data_path = current_app.config.get('VNA_LOGS')
    if not vna_data_path:
        return jsonify({"error": "VNA_DATA path not configured"}), 500
 
    data     = request.get_json() or {}
    filename = data.get("filename", "1.csv")

    filepath = os.path.join(vna_data_path, filename)
 
    os.makedirs(vna_data_path, exist_ok=True)
 
    done = threading.Event()
    vna.collect_data(filepath, done_event=done)
    done.wait()
 
    if not os.path.exists(filepath):
        return jsonify({"error": "collect_data completed but file was not written — check convergence logs"}), 500
 
    return jsonify({
        "msg":      "A-scan saved",
        "filename": filepath,
    }), 200
 
 
# @api_blueprint.route('/vna/bscan', methods=['POST'])
# def vna_bscan():
#     """
#     Drive a full B-scan from this route — the VNA worker is called once per
#     angle step while the rotator is moved externally via /robot/move.
 
#     This route owns the loop; VNAThread.collect_data() is called for each step.
 
#     Request body
#     ------------
#     {
#         "folder_name":  "my_scan",   // base name — will become my_scan_1, my_scan_2, …
#         "b_scan_count": 72,          // number of angular steps (default 72)
#         "retry":        false        // if true, resume the last existing folder
#     }
 
#     Response
#     --------
#     {
#         "msg":        "B-scan complete",
#         "folder":     "my_scan_1",
#         "scans_done": 72
#     }
#     """
#     vna = current_app.config.get('VNA_WORKER')
#     if not vna:
#         return jsonify({"error": "VNA worker not initialized"}), 500
 
#     robot = current_app.config.get('ROBOT_WORKER')
 
#     data          = request.get_json() or {}
#     folder_name   = data.get("folder_name", "bscan")
#     b_scan_count  = int(data.get("b_scan_count", 72))
#     retry         = bool(data.get("retry", False))
 
#     # Build angle array and output folder using module-level helpers
#     angles = bscan(b_scan_count)
#     folder = env_creation(folder_name, start=1, retry_flag=retry)
 
#     # Resume from last completed index if retrying
#     last_number = get_last_existing_number(folder) or 0 if retry else 0
 
#     scans_done = 0
 
#     for i in range(last_number, b_scan_count):
#         angle = float(angles[i])
 
#         # Move rotator if robot worker is available
#         if robot:
#             move_done = threading.Event()
#             robot.move_to(0, 0, angle, done_event=move_done)
#             move_done.wait()
 
#         # Collect one A-scan — blocks until done
#         filepath = os.path.join(folder, f'{i + 1}.csv')
#         done = threading.Event()
#         vna.collect_data(filepath, done_event=done)
#         done.wait()
 
#         if not os.path.exists(filepath):
#             return jsonify({
#                 "error":      f"A-scan {i + 1} failed — file not written. Aborting.",
#                 "scans_done": scans_done,
#                 "folder":     folder,
#             }), 500
 
#         scans_done += 1
 
#     return jsonify({
#         "msg":        "B-scan complete",
#         "folder":     folder,
#         "scans_done": scans_done,
#     }), 200
 
 
# @api_blueprint.route('/vna/bscan/status', methods=['GET'])
# def vna_bscan_status():
#     """
#     Report how many A-scan CSVs have been written into an existing folder.
#     Useful for polling progress during a long B-scan.
 
#     Query params
#     ------------
#     folder : str — folder to check (e.g. "my_scan_1")
 
#     Response
#     --------
#     {
#         "folder":      "my_scan_1",
#         "scans_done":  12,
#         "last_file":   "12.csv"   | null
#     }
#     """
#     folder = request.args.get("folder")
#     if not folder:
#         return jsonify({"error": "Provide ?folder=<folder_name>"}), 400
 
#     if not os.path.exists(folder):
#         return jsonify({"error": f"Folder '{folder}' does not exist"}), 404
 
#     last = get_last_existing_number(folder)
#     return jsonify({
#         "folder":     folder,
#         "scans_done": last if last is not None else 0,
#         "last_file":  f"{last}.csv" if last is not None else None,
#     }), 200

# =============================================================================
# TOF ROUTES
# =============================================================================


# @api_blueprint.route('/tof/scan', methods=['POST'])
# def tof_scan():
#     tof = current_app.config.get('TOF_WORKER')
#     if not tof:
#         return jsonify({"error": "TOF worker not initialized"}), 500
 
#     port = (request.get_json() or {}).get("port", "COM5")
#     done = threading.Event()
#     distances = None
 
#     try:
#         print(f"[{tof.name}]: Connecting to port {port}")
#         # Fix 1: run record() in a background thread so Flask is not blocked
#         threading.Thread(
#             target=tof.record,
#             kwargs={"port": port, "done_event": done},
#             daemon=True
#         ).start()
 
#         # Fix 2: done.wait() now actually blocks until the background thread
#         # finishes; timeout prevents an infinite hang if something goes wrong
#         if not done.wait(timeout=15):
#             return jsonify({"error": "Timed out waiting for TOF reading"}), 504
 
#         distances = tof.last_distance   # read result written by record()
 
#     except Exception as e:
#         # Fix 3: named exception, return proper error response
#         print(f"[{tof.name}] Error: {e}")
#         return jsonify({"error": str(e)}), 500
 
#     return jsonify({"msg": "Recorded", "zones": distances}), 200

# @api_blueprint.route('/tof/reset', methods=['GET'])
# def tof_reset():
#     tof = current_app.config.get('TOF_WORKER')
#     if not tof:
#         return jsonify({"error": "TOF worker not initialized"}), 500

#     try:
#         tof.reset()
#         return jsonify({"msg": "Reset log"}), 200
#     except Exception as e:
#         print(f"[{tof.name}] Error: {e}")
#         return jsonify({"error": str(e)}), 500  # str(e) so jsonify can serialize it

# @api_blueprint.route('/tof/process_data', methods=['POST'])
# def tof_process_data():
#     data         = request.get_json() or {}
#     path         = data.get("path")
#     r_trajectory = data.get("r_trajectory", 40)
#     arc_span     = data.get("arc_span", 360)

#     if path:
#         path = os.path.join(current_app.config.get('TOF_RECORD'), path)

#     tof = current_app.config.get('TOF_WORKER')
#     if not tof:
#         return jsonify({"error": "TOF worker not initialized"}), 500

#     try:
#         x_trunk, y_trunk, plot_b64 = tof.process_data(
#             path=path,
#             r_trajectory=r_trajectory,
#             arc_span=arc_span
#         )
#         return jsonify({
#             "x_trunk":  x_trunk,
#             "y_trunk":  y_trunk,
#             "image":    plot_b64
#         }), 200
#     except Exception as e:
#         return jsonify({"error": str(e)}), 500
    
@api_blueprint.route('/robot/test_fetch', methods=['POST'])
def robot_test_fetch():
    # 1. Get the URL and Path from the request sent to THIS route
    data = request.get_json()
    target_url = data.get("url")  # e.g., "http://127.0.0.1:5000/lidar/process_data"
    file_path = data.get("path")   # e.g., "scan_001.csv"

    if not target_url or not file_path:
        return jsonify({"error": "Both 'url' and 'path' are required in the request body"}), 400

    # 2. Access the Robot worker instance
    robot_worker = current_app.config.get('ROBOT_WORKER')
    
    if not robot_worker:
        return jsonify({"error": "Robot worker not initialized"}), 500

    # 3. Instruct the robot to fetch the data
    points_dict = robot_worker.fetch_lidar_points(target_url, file_path)

    if points_dict is not None:
        return jsonify({
            "status": "success",
            "received_from": target_url,
            "points": points_dict
        }), 200
    else:
        return jsonify({"error": "Robot failed to fetch data from the specified URL"}), 500
    
#======Individual tests======================================================

@api_blueprint.route('/test/tof/process_data', methods=['GET'])
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

# @api_blueprint.route('/start_run', methods=['POST'])
# def start_run():
#     lidar_worker = current_app.config.get('LIDAR_WORKER')
#     robot_worker = current_app.config.get('ROBOT_WORKER')
#     vna_worker   = current_app.config.get('VNA_WORKER')

#     data   = request.get_json()
#     port   = data.get("port")
#     radius = data.get("radius")

#     if radius:
#         robot_worker.R_trajectory = float(radius)

#     # 1. LiDAR scan — block until done
#     done = threading.Event()
#     lidar_worker.scan(port=port, done_event=done)
#     done.wait()

#     path_to_lidar = os.path.join(current_app.config.get('LIDAR_DATA'), 'raw.csv')
#     points, r_exp, _ = lidar_worker.process_data(path=path_to_lidar)

#     # 2. Robot + VNA — block on every move then every VNA collect
#     path_to_tof = os.path.join(current_app.config.get('TOF_RECORD'), 'tof.csv')
#     vna_results = []

#     # Reset tof.csv before each run
#     os.makedirs(os.path.dirname(path_to_tof), exist_ok=True)
#     open(path_to_tof, 'w').close()

#     for i, point in enumerate(points):
#         x, y, z = point["x"], point["y"], point["z"]

#         # Block until arm has moved AND tof is logged
#         done = threading.Event()
#         robot_worker.move_to(x, y, z, path_tof=path_to_tof, done_event=done)
#         done.wait()

#         # Block until VNA collection is done
#         done = threading.Event()
#         vna_worker.collect_data(f'{i + 1}.csv', done_event=done)
#         done.wait()

#         vna_results.append(f'{i + 1}.csv')

#     # 3. Generate B-Scan image
#     x_trunk, y_trunk, img = robot_worker.process_data(path=path_to_tof)

#     return x