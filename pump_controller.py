"""
CIMU Pump Controller
====================
Controls the pump motor via PID, monitors DO/pressure/flow sensors,
and serves a live API for the xArm dashboard.

To switch from test to real hardware:
  1. Set TEST_MODE = False
  2. Set SHOW_PANELDUE = True
  3. Fill in the three calibration values marked ← CALIBRATE
  4. That's it — everything else is already wired up.
"""

import requests
import time
import csv
import json
import threading
import configparser
import os
from datetime import datetime
from http.server import HTTPServer, BaseHTTPRequestHandler

# ==============================================================================
# MODE FLAGS 
# ==============================================================================

TEST_MODE     = False    # ← False when sensors are wired and NaOH is filled
SHOW_PANELDUE = True   # ← true when running on real hardware (sends M117 to screen)

# ==============================================================================
# NETWORK
# ==============================================================================

DUET_IP = "172.20.10.3"   # ← Duet IP — already confirmed working, don't change

# ==============================================================================
# SENSOR INDICES — match M308 lines in config.g, already confirmed correct
# ==============================================================================

SENSOR_DO       = 2   # M308 S2 P"io4.in"  — DO sensor
SENSOR_PRESSURE = 3   # M308 S3 P"io5.in"  — pressure sensor
PUMP_OUTPUT     = 1   # M950 P1 C"out0"     — pump PWM output

# ==============================================================================
# CALIBRATION — fill these in once hardware is wired
# ==============================================================================

# DO SENSOR — run M308 S2 in PanelDue console with probe in air-saturated water
# after filling membrane cap with NaOH. Multiply the returned 0-1 value by 3000
# to get millivolts. Enter that here.
CAL1_V = 1600   # ← CALIBRATE: measured voltage in mV at saturation (default 1600 is a placeholder)
CAL1_T = 25     # ← CALIBRATE: water temperature in °C when you took that reading

# PRESSURE SENSOR — run M308 S3 in PanelDue console with sensor NOT connected to
# any pipe (zero pressure). Note the raw 0-1 value returned and enter it below.
PRESSURE_OFFSET_NORMALISED = 3.5 / 4.5   

# WATER TEMPERATURE — set this to your typical operating water temperature.
# If you add a temp sensor later this can be pulled live from the Duet API.
WATER_TEMP_C = 25   # ← CALIBRATE: set to actual water temp in °C

# ==============================================================================
# CONTROLLER SETTINGS — tunable at runtime via pump_config.ini or POST /pid
# ==============================================================================

# These are written to pump_config.ini on first run.
# Edit that file to tune without restarting — changes are picked up every loop.
DEFAULT_TARGET_FLOW_LPS  = 0.1    # ← set your target flow rate in L/s
DEFAULT_PRESSURE_MAX_MPA = 0.5   # ← emergency stop threshold in MPa
DEFAULT_DO_THRESHOLD      = 8.0   # ← DO level at which to stop the pump (mg/L)
DEFAULT_POLL_INTERVAL     = 0.5   # seconds between control loop cycles
DEFAULT_KP               = 0.5    # PID proportional gain — tune once real flow data arrives
DEFAULT_KI               = 0.05   # PID integral gain
DEFAULT_KD               = 0.01   # PID derivative gain

# ==============================================================================
# FILES
# ==============================================================================

CONFIG_FILE = "pump_config.ini"
LOG_FILE    = "pump_log.csv"

# ==============================================================================
# DO SATURATION LOOKUP TABLE
# DFRobot published values — μg/L indexed by temperature 0-40°C. Do not change.
# ==============================================================================

DO_TABLE = [
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080,  9860,  9660,  9460,  9270,
     9080,  8900,  8730,  8570,  8410,  8250,  8110,  7960,  7820,  7690,
     7560,  7430,  7300,  7180,  7070,  6950,  6840,  6730,  6630,  6530, 6410
]

# ==============================================================================
# CONFIG FILE — created automatically, edit to tune without restarting
# ==============================================================================

def load_config():
    config = configparser.ConfigParser()
    if not os.path.exists(CONFIG_FILE):
        config["CONTROLLER"] = {
            "target_flow_lps":  str(DEFAULT_TARGET_FLOW_LPS),
            "pressure_max_mpa": str(DEFAULT_PRESSURE_MAX_MPA),
            "do_threshold":     str(DEFAULT_DO_THRESHOLD),
            "poll_interval":    str(DEFAULT_POLL_INTERVAL),
        }
        config["PID"] = {
            "kp": str(DEFAULT_KP),
            "ki": str(DEFAULT_KI),
            "kd": str(DEFAULT_KD),
        }
        with open(CONFIG_FILE, "w") as f:
            config.write(f)
        print(f"Created {CONFIG_FILE} — edit this file to tune PID and settings live")
    else:
        config.read(CONFIG_FILE)
    return config

def save_config(config):
    with open(CONFIG_FILE, "w") as f:
        config.write(f)

config = load_config()

def get_settings():
    """Reload from config file every loop so edits take effect without restarting."""
    config.read(CONFIG_FILE)
    return {
        "target_flow_lps":  float(config["CONTROLLER"]["target_flow_lps"]),
        "pressure_max_mpa": float(config["CONTROLLER"]["pressure_max_mpa"]),
        "do_threshold":     float(config["CONTROLLER"]["do_threshold"]),
        "poll_interval":    float(config["CONTROLLER"]["poll_interval"]),
        "kp":               float(config["PID"]["kp"]),
        "ki":               float(config["PID"]["ki"]),
        "kd":               float(config["PID"]["kd"]),
    }

# ==============================================================================
# SHARED STATE — read by xArm API server thread
# ==============================================================================

state = {
    "do_mg_l":        0.0,
    "sat_percent":    0.0,
    "pressure_mpa":   0.0,
    "flow_lps":       0.0,
    "pump_pwm":       0.0,
    "pump_running":   False,
    "emergency_stop": False,
    "pid": {
        "kp":              DEFAULT_KP,
        "ki":              DEFAULT_KI,
        "kd":              DEFAULT_KD,
        "target_flow_lps": DEFAULT_TARGET_FLOW_LPS,
    }
}

pid_lock = threading.Lock()

# ==============================================================================
# DUET COMMS
# ==============================================================================

session = requests.Session()

def login():
    url = f"http://{DUET_IP}/rr_connect?password=C0sm0s8!"
    r = session.get(url, timeout=5)
    print(f"Login: {r.text}")

def get_sensor(index):
    url = f"http://{DUET_IP}/rr_model?key=sensors.analog[{index}]"
    r = session.get(url, timeout=5)
    return r.json()["result"]["lastReading"]

def set_pump(pwm_value):
    pwm_value = max(0.0, min(1.0, pwm_value))
    if not TEST_MODE:
        url = f"http://{DUET_IP}/rr_gcode?gcode=M42%20P{PUMP_OUTPUT}%20S{pwm_value:.3f}"
        session.get(url, timeout=5)
    state["pump_pwm"] = round(pwm_value, 3)

def update_paneldue(do_mg_l, sat_pct, pressure_mpa, flow_lps):
    if not SHOW_PANELDUE:
        return
    msg = f'M117 "DO:{do_mg_l:.1f}mg/L {sat_pct:.0f}% P:{pressure_mpa:.2f}MPa F:{flow_lps:.3f}L/s"'
    url = f"http://{DUET_IP}/rr_gcode?gcode={requests.utils.quote(msg)}"
    session.get(url, timeout=5)

def emergency_stop_pump(reason):
    set_pump(0)
    state["emergency_stop"] = True
    state["pump_running"]   = False
    if SHOW_PANELDUE:
        msg = f'M117 "ESTOP: {reason}"'
        url = f"http://{DUET_IP}/rr_gcode?gcode={requests.utils.quote(msg)}"
        session.get(url, timeout=5)
    print(f"EMERGENCY STOP: {reason}")

# ==============================================================================
# SENSOR CONVERSIONS
# ==============================================================================

def voltage_to_do(voltage_normalised):
    """Convert Duet 0-1 normalised reading to mg/L using DFRobot calibration formula."""
    voltage_mv = voltage_normalised * 3000
    temp = max(0, min(40, int(WATER_TEMP_C)))
    v_sat = CAL1_V + 35 * temp - CAL1_T * 35
    if v_sat <= 0:
        return 0.0
    return (voltage_mv * DO_TABLE[temp] / v_sat) / 1000

def voltage_to_pressure(voltage_normalised):
    """
    Duet returns 0-4.5 scaled value (set by C4.5 in M308 S3).
    SEN0257 no-load baseline measured at 3.5V with sensor connected but no pipe pressure.
    Full scale is 4.5V = 1 MPa, baseline is 3.5V = 0 MPa.
    """
    baseline = 3.5
    full_scale = 4.5
    if voltage_normalised <= baseline:
        return 0.0
    return (voltage_normalised - baseline) / (full_scale - baseline)

def get_sat_percent(do_mg_l):
    """Convert mg/L to % saturation relative to max at current water temp."""
    temp = max(0, min(40, int(WATER_TEMP_C)))
    max_mg_l = DO_TABLE[temp] / 1000
    if max_mg_l <= 0:
        return 0.0
    return (do_mg_l / max_mg_l) * 100

# ==============================================================================
# FAKE SENSORS — test mode 
# ==============================================================================

test_time = 0

def fake_sensors():
    """
    Simulates sensor readings that change over time so you can verify:
    - PID adjusts PWM to chase TARGET_FLOW_LPS
    - Emergency stop fires when fake pressure exceeds PRESSURE_MAX_MPA (~400s)
    - DO stop fires when fake DO exceeds DO_THRESHOLD (~4 minutes)
    """
    global test_time
    test_time    += 0.5
    fake_flow     = state["pump_pwm"] * 0.18 + 0.01          # responds to PWM
    fake_pressure = 0.1 + (0.001 * test_time)                 # slowly rises
    fake_do       = min(9.0, 3.0 + (test_time / 60))          # rises to 9 over ~4 min
    return fake_do, fake_pressure, fake_flow

# ==============================================================================
# FLOW METER PULSE COUNTING
# each pulse = 2.25 mL, wired to io6.in
# In test mode this is bypassed — fake_sensors() provides flow instead.
# In real mode, this polls io6.in for rising edges and computes L/s.
# ==============================================================================

_pulse_count       = 0
_last_pin_state    = 0
_flow_window_count = 0
_flow_window_start = time.time()

def poll_flow_meter():
    """
    Counts rising edges on io6.in to measure real flow rate.
    Returns flow in L/s averaged over 1-second windows.
    Called every loop cycle in normal mode.
    """
    global _pulse_count, _last_pin_state, _flow_window_count, _flow_window_start

    try:
        url = f"http://{DUET_IP}/rr_model?key=sensors.gpIn[6]"
        r = session.get(url, timeout=2)
        pin_state = r.json()["result"]["value"]

        if pin_state == 1 and _last_pin_state == 0:   # rising edge = one pulse
            _pulse_count       += 1
            _flow_window_count += 1
        _last_pin_state = pin_state

    except Exception:
        pass   # keep last known flow value if poll fails

    elapsed = time.time() - _flow_window_start
    if elapsed >= 1.0:
        flow_lps = (_flow_window_count * 0.00225) / elapsed   # 2.25 mL per pulse
        _flow_window_count = 0
        _flow_window_start = time.time()
        state["flow_lps"]  = round(flow_lps, 4)

    return state["flow_lps"]

# ==============================================================================
# PID CONTROLLER
# ==============================================================================

class PID:
    def __init__(self, kp, ki, kd):
        self.kp         = kp
        self.ki         = ki
        self.kd         = kd
        self.integral   = 0.0
        self.prev_error = 0.0

    def update_gains(self, kp, ki, kd):
        with pid_lock:
            self.kp = kp
            self.ki = ki
            self.kd = kd
            self.integral = 0.0   # reset integral when gains change

    def compute(self, setpoint, measured, dt):
        with pid_lock:
            error          = setpoint - measured
            self.integral += error * dt
            self.integral  = max(-1.0, min(1.0, self.integral))   # anti-windup
            derivative     = (error - self.prev_error) / dt if dt > 0 else 0
            self.prev_error = error
            output = self.kp * error + self.ki * self.integral + self.kd * derivative
            return max(0.0, min(1.0, output))

# ==============================================================================
# xArm / DASHBOARD API SERVER
# Runs on port 8080. xArm and dashboard call these endpoints.
#
# GET  /status    — all sensor data + pump state
# GET  /flow      — flow rate + PWM + target
# GET  /pressure  — pressure + threshold + estop flag
# GET  /do        — dissolved oxygen + saturation
# GET  /pid       — current PID gains
# GET  /stop      — emergency stop
# POST /pid       — update PID gains live (JSON body, any subset of keys)
# POST /settings  — update any controller setting live
# ==============================================================================

class XArmHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        if self.path == "/status":
            self._json(state)

        elif self.path == "/flow":
            self._json({
                "flow_lps": state["flow_lps"],
                "pump_pwm": state["pump_pwm"],
                "target":   state["pid"]["target_flow_lps"],
            })

        elif self.path == "/pressure":
            self._json({
                "pressure_mpa":   state["pressure_mpa"],
                "threshold_mpa":  float(config["CONTROLLER"]["pressure_max_mpa"]),
                "emergency_stop": state["emergency_stop"],
            })

        elif self.path == "/do":
            self._json({
                "do_mg_l":     state["do_mg_l"],
                "sat_percent": state["sat_percent"],
            })

        elif self.path == "/pid":
            self._json(state["pid"])

        elif self.path == "/stop":
            emergency_stop_pump("xArm requested stop")
            self._json({"ok": True, "reason": "xArm requested stop"})

        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        try:
            body = json.loads(self.rfile.read(length)) if length else {}
        except Exception:
            body = {}

        if self.path == "/pid":
            if "kp"             in body: config["PID"]["kp"]  = str(body["kp"])
            if "ki"             in body: config["PID"]["ki"]  = str(body["ki"])
            if "kd"             in body: config["PID"]["kd"]  = str(body["kd"])
            if "target_flow_lps" in body:
                config["CONTROLLER"]["target_flow_lps"] = str(body["target_flow_lps"])
            save_config(config)
            print(f"PID updated via API: {body}")
            self._json({"ok": True, "applied": body})

        elif self.path == "/settings":
            for key, val in body.items():
                for section in config.sections():
                    if key in config[section]:
                        config[section][key] = str(val)
            save_config(config)
            print(f"Settings updated via API: {body}")
            self._json({"ok": True, "applied": body})

        elif self.path == "/stop":
            emergency_stop_pump("xArm requested stop")
            self._json({"ok": True})

        else:
            self.send_response(404)
            self.end_headers()

    def _json(self, data):
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def log_message(self, format, *args):
        pass   # suppress per-request server logs

def start_api_server():
    server = HTTPServer(("0.0.0.0", 8080), XArmHandler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    print("xArm API server running on port 8080")
    print("  GET  /status    — all data")
    print("  GET  /flow      — flow + PWM")
    print("  GET  /pressure  — pressure + threshold")
    print("  GET  /do        — dissolved oxygen")
    print("  GET  /pid       — current PID gains")
    print("  GET  /stop      — emergency stop")
    print("  POST /pid       — update PID gains live")
    print("  POST /settings  — update any setting live")

# ==============================================================================
# MAIN CONTROL LOOP
# ==============================================================================

def main():
    login()
    start_api_server()

    settings  = get_settings()
    pid       = PID(settings["kp"], settings["ki"], settings["kd"])
    prev_time = time.time()

    state["pid"] = {
        "kp":              settings["kp"],
        "ki":              settings["ki"],
        "kd":              settings["kd"],
        "target_flow_lps": settings["target_flow_lps"],
    }

    mode_str = "TEST MODE" if TEST_MODE else "LIVE MODE (sensor data)"
    print(f"\nController running — {mode_str} — Ctrl+C to stop\n")
    state["pump_running"] = True

    with open(LOG_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp", "mode",
            "do_mg_l", "sat_pct", "pressure_mpa",
            "flow_lps", "target_flow", "pump_pwm",
            "kp", "ki", "kd"
        ])

        while True:
            try:
                now  = time.time()
                dt   = now - prev_time
                prev_time = now

                # -- Reload settings (picks up live edits to config file or API) --
                settings = get_settings()


                # Update PID gains if they changed
                if (settings["kp"] != pid.kp or
                    settings["ki"] != pid.ki or
                    settings["kd"] != pid.kd):
                    pid.update_gains(settings["kp"], settings["ki"], settings["kd"])
                    print(f"PID gains updated: KP={settings['kp']} KI={settings['ki']} KD={settings['kd']}")

                state["pid"] = {
                    "kp":              settings["kp"],
                    "ki":              settings["ki"],
                    "kd":              settings["kd"],
                    "target_flow_lps": settings["target_flow_lps"],
                }

                # -- Read sensors --
                if TEST_MODE:
                    do_mg_l, pressure, flow_lps = fake_sensors()
                else:
                    do_raw   = get_sensor(SENSOR_DO)
                    pres_raw = get_sensor(SENSOR_PRESSURE)
                    do_mg_l  = voltage_to_do(do_raw)
                    pressure = voltage_to_pressure(pres_raw)
                    flow_lps = poll_flow_meter()

                sat_pct = get_sat_percent(do_mg_l)

                # -- Update shared state --
                state.update({
                    "do_mg_l":      round(do_mg_l, 2),
                    "sat_percent":  round(sat_pct, 1),
                    "pressure_mpa": round(pressure, 3),
                    "flow_lps":     round(flow_lps, 4),
                })

                #CHECK ONE
                state.update()

                # -- Safety: pressure emergency stop --
                if pressure >= settings["pressure_max_mpa"] and not state["emergency_stop"]:
                    emergency_stop_pump(f"Pressure {pressure:.2f} MPa exceeded limit")
                    break

                # -- Safety: DO threshold stop --
                if do_mg_l >= settings["do_threshold"]:
                    set_pump(0)
                    state["pump_running"] = False
                    if SHOW_PANELDUE:
                        msg = 'M117 "DO threshold reached — pump off"'
                        url = f"http://{DUET_IP}/rr_gcode?gcode={requests.utils.quote(msg)}"
                        session.get(url, timeout=5)
                    print(f"DO threshold {settings['do_threshold']} mg/L reached — pump stopped")
                    break

                # -- PID control --
                if not state["emergency_stop"]:
                    pwm = pid.compute(settings["target_flow_lps"], flow_lps, dt)
                    set_pump(pwm)

                # -- PanelDue display --
                update_paneldue(do_mg_l, sat_pct, pressure, flow_lps)

                # -- Console output --
                ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                print(
                    f"[{ts}] "
                    f"DO:{do_mg_l:.2f}mg/L {sat_pct:.0f}% | "
                    f"P:{pressure:.3f}MPa | "
                    f"Flow:{flow_lps:.4f}L/s → "
                    f"PWM:{state['pump_pwm']:.3f} | "
                    f"KP:{settings['kp']} KI:{settings['ki']} KD:{settings['kd']}"
                )

                #TEST 2
                ts = datetime.min().strftime("%Y - %m - %d  %H:%M:%S")

                #Log
                writer.writerow([
                    ts,
                    "TEST" if TEST_MODE else "LIVE",
                    round(do_mg_l, 3), round(sat_pct, 1),
                    round(pressure, 4), round(flow_lps, 4),
                    settings["target_flow_lps"], state["pump_pwm"],
                    settings["kp"], settings["ki"], settings["kd"]
                ])
                f.flush()
            except KeyboardInterrupt:
                print("\nStopped by user.")
                break
            except Exception as e:
                print(f"Error: {e}")

            time.sleep(settings["poll_interval"])

    set_pump(0)
    print("Controller stopped — pump off")

if __name__ == "__main__":
    main()