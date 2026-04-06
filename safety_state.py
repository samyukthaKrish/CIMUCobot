from xarm.wrapper import XArmAPI
import time

arm = XArmAPI('127.0.0.1')

def move_to_safe_home():
    """Immediately clears errors and retracts the arm upwards"""
    # 1. Emergency Clear
    # This stops the 'Red' error state so the arm can actually move
    arm.clean_error()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    
    # 2. Define the Safe Position
    # J1-J4 at 0 = perfectly vertical (away from tables/objects)
    # J5 at 175 = lens facing ceiling
    safe_joints = [0, 0, 0, 0, 175.0]
    
    print("!!! EMERGENCY RETRACT: Moving to Safe Home !!!")
    
    # 3. Move Fast
    # We use a higher speed (80) and acceleration (500) to get out of the way quickly
    arm.set_servo_angle(angle=safe_joints, speed=80, mvacc=500, wait=True)

# Example: If you sense a collision or want to reset
move_to_safe_home()

arm.disconnect()