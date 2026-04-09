from xarm.wrapper import XArmAPI
import time

arm = XArmAPI('127.0.0.1')

def move_to_safe_home():
    arm.clean_error()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    time.sleep(0.5)
    
    # THE "WIRING-FRIENDLY" POSE
    # J2: -30 (Lifts the arm enough to clear the 130mm tool)
    # J3: 0   (Keep elbow straight to avoid pulling cables)
    # J4: 0   (Straight wrist)
    # J5: 175 (Lens faces up)
    wire_safe_pose = [0, -30.0, 0, 0, 175.0]
    
    print("!!! WIRE SAFETY: Moving to Wide-Arc Home !!!")
    
    # Move at a moderate speed to prevent wire "whiplash"
    code = arm.set_servo_angle(angle=wire_safe_pose, speed=30, wait=True)
    
    if code == 0:
        print("Safe Home reached without stretching wires.")
    else:
        print(f"Failed. Sim Error: {code}. Try clearing the red error in Studio UI.")

move_to_safe_home()
arm.disconnect()