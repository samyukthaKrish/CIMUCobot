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
# J5=180 (Flipping the lens to face the floor)
floor_state = [0, 0, 0, 0, 180.0]
#rotates joint 4 180 degrees so that the lens faces the ceiling
ceiling_state = [0, 0, 0, 180.0, 180.0]
#rotates joint 2 so that the lens faces the right side
right_state = [0, 215, 0, 0, 0]
#rotates joint 2 the opposite way (negative direction) so
# that the lens faces the left side 
left_state = [0, -250, 0, 0, 180]

print("pointing lens to floor...")

# Using a slightly slower speed to avoid collisions 
code = arm.set_servo_angle(angle=floor_state, speed=30, mvacc=100, wait=True)

if code == 0:
    print("lens is facing the floor")
else:
    print(f"Move failed with code: {code}.")

arm.disconnect()

