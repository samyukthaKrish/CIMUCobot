from xarm.wrapper import XArmAPI

arm = XArmAPI('127.0.0.1') 

arm.motion_enable(enable=True)
# position control mode 
arm.set_mode(0) 
# Put arm is in ready state
arm.set_state(state=0) 

#Get current positions so that other joints aren't moved 
code, current_joints = arm.get_servo_angle()

if code == 0:
    # upward target for Joint 5, where the tandard downward is 0
    # moves 180/-180 degrees to change orientation to face upwards
    target_joints = list(current_joints)
    target_joints[4] = 180.0 
    
    # Move only Joint 5, where the speed is deg/s and mvacc is deg/s^2
    arm.set_servo_angle(angle=target_joints, speed=30, mvacc=100, wait=True)
    
    print("Lfacing the ceiling.")
else:
    print(f"Fails,joint data: {code}")

arm.disconnect()