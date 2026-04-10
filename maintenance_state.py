from xarm.wrapper import XArmAPI
import time

arm = XArmAPI('127.0.0.1')

# 1. THE EMERGENCY UNLOCK
# You MUST clear the C22/Code 3 error from the previous crash
arm.clean_error()
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
time.sleep(1)

# 2. THE HIGH-CLEARANCE POSE
# J1: 0
# J2: -30.0 (Tilt the shoulder BACK to lift the whole assembly up)
# J3: -20.0 (Angle the elbow up to gain more height)
# J4: -90.0 (Bends the wrist UP toward the sky)
# J5: 0.0   (Keeps the lens oriented correctly)
high_ceiling_pose = [0, -90.0, 0, -90.0, 0]

print("Lifting arm for 130mm clearance and flipping lens UP...")
# Moving at speed 20 for safety
code = arm.set_servo_angle(angle=high_ceiling_pose, speed=20, wait=True)

if code == 0:
    print("Success: Lens is up and attachment cleared the table.")
else:
    print(f"Still failing. Error: {code}. The attachment is likely still hitting the table.")

arm.disconnect()