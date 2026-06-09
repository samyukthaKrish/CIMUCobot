"""
xArm5 Dashboard Server — Studio WebSocket Edition
===================================================
Bridges your browser dashboard to UFACTORY Studio's WebSocket API (port 18333).
All motion commands go through Studio exactly as if clicked in the UI.
Live data is read from the xArm SDK alongside Studio without conflict.
 
Architecture:
  Browser <──WebSocket──> This server (port 5000)
                               │
                               ├── Studio WebSocket (port 18333) ← controls
                               └── xArm SDK (port 30003)         ← reads data
 
Requirements:
    pip install flask flask-socketio xArm-Python-SDK websocket-client
 
Usage:
    python xarm5_dashboard_server.py --ip 192.168.1.x
    Then open http://localhost:5000
 
Adding/removing scripts:
    Edit scripts_config.json — no code changes needed.
"""
 
import argparse
import csv
import json
import math
import os
import threading
import time
from datetime import datetime
 
from flask import Flask, send_from_directory, jsonify, request
from flask_socketio import SocketIO
 
try:
    import websocket as ws_client
    WS_AVAILABLE = True
except ImportError:
    print("[WARN] websocket-client not found. Install: pip install websocket-client")
    WS_AVAILABLE = False
 
try:
    from xarm.wrapper import XArmAPI
    XARM_AVAILABLE = True
except ImportError:
    print("[WARN] xArm SDK not found. Running in DEMO mode.")
    XARM_AVAILABLE = False
 
 
 
# ── xArm5 IK / FK (hidden from users, runs server-side) ──────────────────────
 
import math as _math
 
_SAFE_POSE = [0, -30.0, 0, -10.0, 175.0]
_D1     = 267.0
_A2     = 289.48866
_A3     = 351.158796
_D5     = 97.0
_A5     = 76.0
_T2_OFF = _math.radians(-79.34995)
_T3_OFF = _math.radians(156.599924)
_T4_OFF = _math.radians(-77.249974)
 
_DH_PARAMS = [
    (0,            0,   _D1,  0       ),
    (-_math.pi/2,  0,   0,    _T2_OFF ),
    (0,            _A2, 0,    _T3_OFF ),
    (0,            _A3, 0,    _T4_OFF ),
    (-_math.pi/2,  _A5, _D5,  0       ),
]
 
def _mat_mul(A, B):
    C = [[0]*4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            for k in range(4):
                C[i][j] += A[i][k] * B[k][j]
    return C
 
def _dh_matrix(alpha, a, d, theta):
    ct, st = _math.cos(theta), _math.sin(theta)
    ca, sa = _math.cos(alpha), _math.sin(alpha)
    return [
        [ct,    -st,    0,    a     ],
        [st*ca,  ct*ca, -sa,  -sa*d ],
        [st*sa,  ct*sa,  ca,   ca*d ],
        [0,      0,      0,    1    ]
    ]
 
def _forward_kinematics(joint_angles_deg):
    T = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
    for i, (alpha, a, d, offset) in enumerate(_DH_PARAMS):
        theta = _math.radians(joint_angles_deg[i]) + offset
        T = _mat_mul(T, _dh_matrix(alpha, a, d, theta))
    return [T[0][3], T[1][3], T[2][3]]
 
def _norm3(v):
    return _math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
 
def _gaussian_solve(A, b):
    n = 3
    M = [A[i][:] + [b[i]] for i in range(n)]
    for col in range(n):
        max_row = max(range(col, n), key=lambda r: abs(M[r][col]))
        M[col], M[max_row] = M[max_row], M[col]
        if abs(M[col][col]) < 1e-12:
            return None
        for row in range(col+1, n):
            f = M[row][col] / M[col][col]
            for k in range(col, n+1):
                M[row][k] -= f * M[col][k]
    x = [0.0]*n
    for i in range(n-1, -1, -1):
        x[i] = M[i][n]
        for j in range(i+1, n):
            x[i] -= M[i][j] * x[j]
        x[i] /= M[i][i]
    return x
 
def _inverse_kinematics(target_xyz, seed=None, tol=1.0, max_iter=300, step=0.5):
    if seed is None:
        seed = _SAFE_POSE[:]
    joints = list(seed)
    limits = [(-360,360),(-118,120),(-225,11),(-97,180),(-360,360)]
    eps = 0.1
    for _ in range(max_iter):
        curr = _forward_kinematics(joints)
        err  = [target_xyz[i] - curr[i] for i in range(3)]
        dist = _norm3(err)
        if dist < tol:
            return joints
        J = [[0.0]*5 for _ in range(3)]
        for j in range(5):
            jp = list(joints)
            jp[j] += eps
            pp = _forward_kinematics(jp)
            for r in range(3):
                J[r][j] = (pp[r] - curr[r]) / eps
        JJT = [[0.0]*3 for _ in range(3)]
        for r in range(3):
            for c in range(3):
                for k in range(5):
                    JJT[r][c] += J[r][k] * J[c][k]
        lam = 0.01 * dist
        for i in range(3):
            JJT[i][i] += lam
        x = _gaussian_solve(JJT, err)
        if x is None:
            return None
        delta = [0.0]*5
        for j in range(5):
            for r in range(3):
                delta[j] += J[r][j] * x[r]
        dn = _norm3(delta[:3]) + abs(delta[3]) + abs(delta[4])
        if dn > step:
            delta = [d * step/dn for d in delta]
        for j in range(5):
            joints[j] = max(limits[j][0], min(limits[j][1], joints[j] + delta[j]))
    return None
 
def general_move_func(x, y, z, speed=50):
    if arm is None or not connected:
        return False, "Arm not connected"
    try:
        joints = _inverse_kinematics([x, y, z], seed=_SAFE_POSE[:])
        if joints is None:
            return False, "No IK solution for ({}, {}, {})".format(x, y, z)
        code = arm.set_servo_angle(angle=joints, speed=speed, mvacc=500, wait=False)
        if code != 0:
            return False, "Move failed, code={}".format(code)
        return True, "Moving to X:{} Y:{} Z:{}".format(x, y, z)
    except Exception as e:
        return False, str(e)
 
# ── Config ────────────────────────────────────────────────────────────────────
 
MIN_CONTACT_FORCE_N = 2.0
MAX_CONTACT_FORCE_N = 10.0
CONTACT_THRESHOLD_N = 0.5
TOOL_WEIGHT_N       = 0.0
POLL_INTERVAL       = 0.1   # 10 Hz
STUDIO_PORT         = 18333
SCRIPTS_CONFIG_FILE = "scripts_config.json"
 
LOG_DIR = "logs"
os.makedirs(LOG_DIR, exist_ok=True)
 
 
# ── Scripts config ────────────────────────────────────────────────────────────
 
def load_scripts():
    """
    Load script list from scripts_config.json.
    To add or remove scripts, edit that file — no code changes needed.
 
    Format:
    [
      {
        "label":       "Human-readable name shown on dashboard button",
        "description": "Short description shown as tooltip",
        "project":     "Studio Python IDE project name",
        "file":        "filename.py"
      }
    ]
    """
    if not os.path.exists(SCRIPTS_CONFIG_FILE):
        print(f"[WARN] {SCRIPTS_CONFIG_FILE} not found. No scripts available.")
        return []
    try:
        with open(SCRIPTS_CONFIG_FILE) as f:
            scripts = json.load(f)
        print(f"[OK] Loaded {len(scripts)} script(s) from {SCRIPTS_CONFIG_FILE}")
        return scripts
    except Exception as e:
        print(f"[ERROR] Failed to load {SCRIPTS_CONFIG_FILE}: {e}")
        return []
 
 
# ── App setup ─────────────────────────────────────────────────────────────────
 
app = Flask(__name__, static_folder=".", template_folder=".")
app.config["SECRET_KEY"] = "xarm5-dashboard"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")
 
arm              = None
studio_ws        = None
arm_ip           = None
connected        = False
studio_connected = False
demo_t           = 0.0
running_script   = None   # currently running script label
 
log_file   = None
log_writer = None
log_active = False
 
 
# ── Studio WebSocket ──────────────────────────────────────────────────────────
 
def connect_studio(ip):
    """Connect to UFACTORY Studio WebSocket on port 18333."""
    global studio_ws, studio_connected
 
    if not WS_AVAILABLE:
        return
 
    def on_open(ws):
        global studio_connected
        studio_connected = True
        print(f"[OK] Studio WebSocket connected at {ip}:{STUDIO_PORT}")
        socketio.emit("studio_status", {"connected": True})
 
    def on_message(ws, message):
        try:
            data = json.loads(message)
            # Forward all Studio responses to the browser
            socketio.emit("studio_response", data)
            # Detect program stop from Studio's response
            if data.get("cmd") in ("stop_python_program", "xarm_urgent_stop"):
                global running_script
                running_script = None
                socketio.emit("script_status", {"running": None})
        except Exception:
            pass
 
    def on_error(ws, error):
        global studio_connected
        studio_connected = False
        print(f"[WARN] Studio WebSocket error: {error}")
        socketio.emit("studio_status", {"connected": False})
 
    def on_close(ws, code, msg):
        global studio_connected
        studio_connected = False
        print("[INFO] Studio WebSocket closed. Reconnecting in 5s...")
        socketio.emit("studio_status", {"connected": False})
        time.sleep(5)
        connect_studio(ip)
 
    def run():
        global studio_ws
        studio_ws = ws_client.WebSocketApp(
            f"ws://{ip}:{STUDIO_PORT}",
            on_open=on_open,
            on_message=on_message,
            on_error=on_error,
            on_close=on_close,
        )
        studio_ws.run_forever(reconnect=5)
 
    threading.Thread(target=run, daemon=True).start()
 
 
def studio_send(cmd: str, data: dict = None):
    """
    Send a command to UFACTORY Studio via WebSocket.
    Equivalent to clicking the corresponding button in the Studio UI.
    Message format: {"cmd": "command_name", "data": {...}, "id": "1"}
    """
    if studio_ws is None or not studio_connected:
        return {"ok": False, "error": "Studio WebSocket not connected."}
    try:
        msg = {"cmd": cmd, "id": "1"}
        if data:
            msg["data"] = data
        studio_ws.send(json.dumps(msg))
        return {"ok": True}
    except Exception as e:
        return {"ok": False, "error": str(e)}
 
 
# ── xArm SDK — read-only data ─────────────────────────────────────────────────
 
def connect_arm_readonly(ip):
    """Connect to xArm SDK for reading live data only."""
    global arm, connected
    try:
        arm = XArmAPI(ip, is_radian=False)
        connected = True
        print(f"[OK] xArm SDK connected (read-only) at {ip}")
    except Exception as e:
        print(f"[WARN] xArm SDK connection failed: {e}")
        connected = False
 
 
def get_force_status(force_n):
    if force_n < CONTACT_THRESHOLD_N:    return "NO CONTACT"
    elif force_n < MIN_CONTACT_FORCE_N:  return "TOO LIGHT"
    elif force_n <= MAX_CONTACT_FORCE_N: return "GOOD"
    else:                                return "TOO HARD"
 
 
def read_arm_data():
    """Read live state from xArm. Returns a data dict."""
    global demo_t
 
    if not XARM_AVAILABLE or not connected or arm is None:
        demo_t += POLL_INTERVAL
        fz_raw = 5.0 + 3.0 * math.sin(demo_t * 0.5) + 0.5 * math.sin(demo_t * 3.0)
        contact_force = max(0.0, abs(fz_raw) - TOOL_WEIGHT_N)
        return {
            "connected":       False,
            "studio_connected": studio_connected,
            "demo":            True,
            "timestamp":       datetime.now().isoformat(),
            "running_script":  running_script,
            "force": {
                "fx": round(0.3 * math.sin(demo_t * 0.3), 3),
                "fy": round(0.2 * math.cos(demo_t * 0.4), 3),
                "fz_raw":        round(fz_raw, 3),
                "contact_force": round(contact_force, 3),
                "status":        get_force_status(contact_force),
                "in_window":     MIN_CONTACT_FORCE_N <= contact_force <= MAX_CONTACT_FORCE_N,
            },
            "position": {
                "x":     round(300 + 5 * math.sin(demo_t * 0.1), 2),
                "y":     round(10 * math.cos(demo_t * 0.15), 2),
                "z":     round(250 + 3 * math.sin(demo_t * 0.2), 2),
                "roll":  round(179.5 + 0.5 * math.sin(demo_t * 0.1), 2),
                "pitch": round(0.3 * math.cos(demo_t * 0.2), 2),
                "yaw":   round(1.0 * math.sin(demo_t * 0.05), 2),
            },
            "joints": [round(10 * math.sin(demo_t * 0.1 + i), 2) for i in range(5)],
            "sensors": {
                "o2":          {"connected": False, "value": None, "unit": "%"},
                "temperature": {"connected": False, "value": None, "unit": "°C"},
            },
            "config": {
                "min_force":   MIN_CONTACT_FORCE_N,
                "max_force":   MAX_CONTACT_FORCE_N,
                "tool_weight": TOOL_WEIGHT_N,
            }
        }
 
    try:
        fx = fy = fz_raw = 0.0
        code, fdata = arm.get_ft_sensor_data()
        if code == 0 and fdata:
            fx, fy, fz_raw = float(fdata[0]), float(fdata[1]), float(fdata[2])
        contact_force = max(0.0, abs(fz_raw) - TOOL_WEIGHT_N)
 
        pos = {"x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 0}
        code2, pdata = arm.get_position()
        if code2 == 0 and pdata:
            pos = {k: round(v, 2) for k, v in zip(
                ["x","y","z","roll","pitch","yaw"], pdata[:6])}
 
        joints = [0.0] * 5
        code3, jdata = arm.get_servo_angle()
        if code3 == 0 and jdata:
            joints = [round(j, 2) for j in jdata[:5]]
 
        return {
            "connected":        True,
            "studio_connected": studio_connected,
            "demo":             False,
            "timestamp":        datetime.now().isoformat(),
            "running_script":   running_script,
            "force": {
                "fx": round(fx, 3), "fy": round(fy, 3),
                "fz_raw":        round(fz_raw, 3),
                "contact_force": round(contact_force, 3),
                "status":        get_force_status(contact_force),
                "in_window":     MIN_CONTACT_FORCE_N <= contact_force <= MAX_CONTACT_FORCE_N,
            },
            "position": pos,
            "joints":   joints,
            "sensors": {
                "o2":          {"connected": False, "value": None, "unit": "%"},
                "temperature": {"connected": False, "value": None, "unit": "°C"},
            },
            "config": {
                "min_force":   MIN_CONTACT_FORCE_N,
                "max_force":   MAX_CONTACT_FORCE_N,
                "tool_weight": TOOL_WEIGHT_N,
            }
        }
    except Exception as e:
        print(f"[ERROR] Data read failed: {e}")
        return None
 
 
# ── CSV logging ───────────────────────────────────────────────────────────────
 
def start_log():
    global log_file, log_writer, log_active
    ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(LOG_DIR, f"xarm_log_{ts}.csv")
    log_file = open(path, "w", newline="")
    log_writer = csv.writer(log_file)
    log_writer.writerow([
        "timestamp", "Fx_N", "Fy_N", "Fz_raw_N", "contact_force_N",
        "status", "in_window",
        "pos_x", "pos_y", "pos_z", "roll", "pitch", "yaw",
        "j1", "j2", "j3", "j4", "j5"
    ])
    log_active = True
    return path
 
 
def stop_log():
    global log_file, log_active
    log_active = False
    if log_file:
        log_file.close()
        log_file = None
 
 
def write_log(data):
    if not log_active or log_writer is None:
        return
    f = data["force"]
    p = data["position"]
    j = data["joints"]
    log_writer.writerow([
        data["timestamp"],
        f["fx"], f["fy"], f["fz_raw"], f["contact_force"],
        f["status"], int(f["in_window"]),
        p["x"], p["y"], p["z"], p["roll"], p["pitch"], p["yaw"],
        *j
    ])
    log_file.flush()
 
 
# ── Background polling ────────────────────────────────────────────────────────
 
def poll_loop():
    while True:
        data = read_arm_data()
        if data:
            write_log(data)
            socketio.emit("data", data)
        time.sleep(POLL_INTERVAL)
 
 
# ── Flask routes ──────────────────────────────────────────────────────────────
 
@app.route("/")
def index():
    return send_from_directory(".", "xarm5_dashboard.html")
 
 
@app.route("/api/scripts")
def api_scripts():
    """Return the list of available scripts from scripts_config.json."""
    return jsonify({"scripts": load_scripts()})
 
 
@app.route("/api/scripts/run", methods=["POST"])
def api_run_script():
    """
    Run a Python program saved in Studio's Python IDE.
    Equivalent to opening it in Studio and clicking Run.
    POST body: {"project": "CIMUCobot", "file": "epoxy_run.py", "label": "Epoxy Application"}
    """
    global running_script
    data = request.json or {}
    project = data.get("project", "")
    file    = data.get("file", "")
    label   = data.get("label", file)
 
    if not project or not file:
        return jsonify({"ok": False, "error": "project and file are required"})
 
    result = studio_send("run_python_program", {
        "project_name": project,
        "file_name":    file,
    })
 
    if result.get("ok"):
        running_script = label
        socketio.emit("script_status", {"running": label})
 
    return jsonify({**result, "label": label})
 
 
@app.route("/api/scripts/stop", methods=["POST"])
def api_stop_script():
    """
    Stop the currently running Python program in Studio.
    Equivalent to clicking Stop in Studio's Python IDE.
    """
    global running_script
    result = studio_send("stop_python_program")
    running_script = None
    socketio.emit("script_status", {"running": None})
    return jsonify(result)
 
 
@app.route("/api/estop", methods=["POST"])
def api_estop():
    """Emergency stop — same as Studio's E-Stop button."""
    global running_script
    result = studio_send("xarm_urgent_stop")
    running_script = None
    socketio.emit("estop",       {"active": True})
    socketio.emit("script_status", {"running": None})
    return jsonify(result)
 
 
@app.route("/api/estop/reset", methods=["POST"])
def api_estop_reset():
    """Re-enable motion after E-Stop — same as Studio's Enable button."""
    result = studio_send("xarm_set_state", {"state": 0})
    socketio.emit("estop", {"active": False})
    return jsonify(result)
 
 
@app.route("/api/jog", methods=["POST"])
def api_jog():
    """
    Jog via Studio's live control API — same as the jog arrows in Studio.
    axis:      0=X 1=Y 2=Z 3=Roll 4=Pitch 5=Yaw
    direction: 1=positive -1=negative
    """
    data = request.json or {}
    axis_map = {
        "X+":  (0, 1),  "X-":  (0, -1),
        "Y+":  (1, 1),  "Y-":  (1, -1),
        "Z+":  (2, 1),  "Z-":  (2, -1),
        "RX+": (3, 1),  "RX-": (3, -1),
        "RY+": (4, 1),  "RY-": (4, -1),
        "RZ+": (5, 1),  "RZ-": (5, -1),
    }
    axis_key = data.get("axis", "")
    if axis_key not in axis_map:
        return jsonify({"ok": False, "error": f"Unknown axis: {axis_key}"})
    axis_idx, direction = axis_map[axis_key]
    is_rotation = axis_idx >= 3
    result = studio_send("xarm_move_step", {
        "axis":      axis_idx,
        "direction": direction,
        "step":      5.0 if is_rotation else 10.0,
        "speed":     30.0 if is_rotation else 50.0,
    })
    return jsonify(result)
 
 
@app.route("/api/move/coords", methods=["POST"])
def api_move_coords():
    data  = request.json or {}
    x     = float(data.get("x",     300))
    y     = float(data.get("y",     0))
    z     = float(data.get("z",     300))
    speed = float(data.get("speed", 50))
    ok, msg = general_move_func(x, y, z, speed=speed)
    return jsonify({"ok": ok, "message": msg, "error": msg if not ok else ""})
 
 
@app.route("/api/move/home", methods=["POST"])
def api_home():
    result = studio_send("xarm_move_arc_line", {
        "pose": [300, 0, 400, 180, 0, 0],
        "speed": 80, "acc": 500, "mvtime": 0, "wait": False,
    })
    return jsonify(result)
 
 
@app.route("/api/move/maintenance", methods=["POST"])
def api_maintenance():
    result = studio_send("xarm_move_arc_line", {
        "pose": [300, 0, 500, 180, 0, 0],
        "speed": 60, "acc": 500, "mvtime": 0, "wait": False,
    })
    return jsonify(result)
 
 
@app.route("/api/clear_error", methods=["POST"])
def api_clear_error():
    result = studio_send("xarm_clear_error_warn")
    return jsonify(result)
 
 
@app.route("/api/config", methods=["POST"])
def api_config():
    global MIN_CONTACT_FORCE_N, MAX_CONTACT_FORCE_N, TOOL_WEIGHT_N
    data = request.json or {}
    if "min_force"   in data: MIN_CONTACT_FORCE_N = float(data["min_force"])
    if "max_force"   in data: MAX_CONTACT_FORCE_N = float(data["max_force"])
    if "tool_weight" in data: TOOL_WEIGHT_N        = float(data["tool_weight"])
    return jsonify({"ok": True,
                    "min_force": MIN_CONTACT_FORCE_N,
                    "max_force": MAX_CONTACT_FORCE_N,
                    "tool_weight": TOOL_WEIGHT_N})
 
 
@app.route("/api/log/start", methods=["POST"])
def api_log_start():
    return jsonify({"ok": True, "path": start_log()})
 
 
@app.route("/api/log/stop", methods=["POST"])
def api_log_stop():
    stop_log()
    return jsonify({"ok": True})
 
 
# ── Entry point ───────────────────────────────────────────────────────────────
 
def parse_args():
    parser = argparse.ArgumentParser(description="xArm5 Dashboard Server")
    parser.add_argument("--ip",   default=None,
                        help="xArm5 / Studio IP. Omit for demo mode.")
    parser.add_argument("--port", type=int, default=5000)
    return parser.parse_args()
 
 
if __name__ == "__main__":
    args   = parse_args()
    arm_ip = args.ip
 
    if arm_ip:
        if XARM_AVAILABLE:
            connect_arm_readonly(arm_ip)
        if WS_AVAILABLE:
            connect_studio(arm_ip)
        else:
            print("[WARN] Install websocket-client: pip install websocket-client")
    else:
        print("[INFO] No IP provided — running in DEMO mode.")
 
    threading.Thread(target=poll_loop, daemon=True).start()
 
    print(f"\n  Dashboard : http://localhost:{args.port}")
    print(f"  Studio WS : ws://{arm_ip or 'N/A'}:{STUDIO_PORT}")
    print(f"  Scripts   : {SCRIPTS_CONFIG_FILE}\n")
    socketio.run(app, host="0.0.0.0", port=args.port, debug=False)
