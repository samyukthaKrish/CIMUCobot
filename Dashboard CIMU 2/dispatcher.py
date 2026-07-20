"""
CIMU Robot Dispatcher
=====================
Save this file in UFACTORY Studio and hit Run.
Keep it running the entire time the dashboard is in use.

It watches motion_command.json for new commands from the dashboard
and executes the right motion function when one arrives.

Put this file in the same Studio project folder as:
  - robot_movement.py
  - draw_circle.py  (or draw_circle.txt renamed to .py)
  - draw_box.py
  - draw_lineInBox.py
"""

import json
import os
import time

from robot_movement import RobotMovement
from draw_circle    import CircleDrawer
from draw_box       import BoxDrawer
from draw_lineInBox import LineInBoxDrawer

# Path to the command file the dashboard server writes to.
# Change this to wherever your dashboard folder is on this machine.
COMMAND_FILE = "C:/Users/samyu/OneDrive/Documents/dashboard CIMU/motion_command.json"

# Status file — dispatcher writes back so dashboard knows what's happening
STATUS_FILE  = "C:/Users/samyu/OneDrive/Documents/dashboard CIMU/motion_status.json"

# Poll interval (seconds)
POLL_INTERVAL = 0.2

# ── Setup ─────────────────────────────────────────────────────────────────────

robot = RobotMovement()
robot.go_to_safe_home()

def write_status(status, message=""):
    try:
        with open(STATUS_FILE, 'w') as f:
            json.dump({"status": status, "message": message, "ts": time.time()}, f)
    except Exception:
        pass

def execute_command(cmd):
    action = cmd.get("action", "")
    p      = cmd.get("params", {})

    print("[Dispatcher] Executing: {} with params: {}".format(action, p))
    write_status("running", "Executing: {}".format(action))

    try:
        if action == "move_to":
            ok = robot.move_to(
                float(p["x"]),
                float(p["y"]),
                float(p["z"]),
                speed=int(p.get("speed", 30))
            )

        elif action == "draw_circle":
            drawer = CircleDrawer(robot.arm)
            ok = drawer.draw_circle(
                float(p["center_x"]),
                float(p["center_y"]),
                float(p["center_z"]),
                float(p["radius"]),
                speed=int(p.get("speed", 20))
            )

        elif action == "draw_box":
            drawer = BoxDrawer(robot.arm)
            ok = drawer.draw_box(
                float(p["start_x"]),
                float(p["start_y"]),
                float(p["start_z"]),
                float(p["length"]),
                float(p["width"]),
                speed=int(p.get("speed", 30))
            )

        elif action == "draw_line_in_box":
            drawer = LineInBoxDrawer(robot.arm)
            ok = drawer.draw_lineInBox(
                float(p["start_x"]),
                float(p["start_y"]),
                float(p["start_z"]),
                float(p["length"]),
                float(p["width"]),
                float(p["line_length"]),
                float(p["line_z"]),
                speed=int(p.get("speed", 30))
            )

        elif action == "go_home":
            robot.go_to_safe_home()
            ok = True

        elif action == "estop":
            robot.arm.emergency_stop()
            write_status("stopped", "Emergency stop activated")
            return

        else:
            print("[Dispatcher] Unknown action: {}".format(action))
            write_status("error", "Unknown action: {}".format(action))
            return

        if ok:
            print("[Dispatcher] Done: {}".format(action))
            write_status("idle", "Done: {}".format(action))
        else:
            print("[Dispatcher] Failed: {}".format(action))
            write_status("error", "Motion failed: {}".format(action))

    except Exception as e:
        print("[Dispatcher] Error: {}".format(e))
        write_status("error", str(e))

# ── Main loop ─────────────────────────────────────────────────────────────────

print("[Dispatcher] Ready. Watching for commands...")
write_status("idle", "Dispatcher ready")

last_command_id = None

while True:
    try:
        if os.path.exists(COMMAND_FILE):
            with open(COMMAND_FILE, 'r') as f:
                cmd = json.load(f)

            # Only execute if this is a new command (by unique ID)
            cmd_id = cmd.get("id")
            if cmd_id and cmd_id != last_command_id:
                last_command_id = cmd_id
                execute_command(cmd)

    except Exception as e:
        print("[Dispatcher] Loop error: {}".format(e))

    time.sleep(POLL_INTERVAL)
