from xarm.wrapper import XArmAPI
import time

arm = XArmAPI('127.0.0.1')

# force reset and starting 
arm.clean_error()
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
time.sleep(1)

# defining floor state, where the lens faces the floor
# J5=180 (Flipping the lens to face the ceiling)
floor_state = [0, 0, 0, 0, 180.0]

print("pointing lens to floor...")

# Using a slightly slower speed to avoid collisions 
code = arm.set_servo_angle(angle=floor_state, speed=30, mvacc=100, wait=True)

if code == 0:
    print("lens is facing the floor")
else:
    print(f"Move failed with code: {code}.")

arm.disconnect()
