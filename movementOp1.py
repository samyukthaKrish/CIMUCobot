from xarm.wrapper import XArmAPI
import time

arm = XArmAPI('127.0.0.1')

SAFE_POSE = [0, -30.0, 0, -10.0, 175.0]

def prepare_arm():
    arm.clean_error()
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(0.5)

def move_front_then_touch(hold_time=3.0):
    prepare_arm()

    print("Moving to safe home...")
    code = arm.set_servo_angle(angle=SAFE_POSE, speed=20, wait=True)
    if code != 0:
        print(f"Safe home failed, code={code}")
        return

    # Move whole arm more in front
    front_pose = [0, -20.0, -10.0, -10.0, 175.0]
    print("Moving arm more in front...")
    code = arm.set_servo_angle(angle=front_pose, speed=15, wait=True)
    if code != 0:
        print(f"Front move failed, code={code}")
        return

    # Small touch using J4
    touch_pose = front_pose[:]
    touch_pose[3] = front_pose[3] + 8.0

    print("Doing touch...")
    code = arm.set_servo_angle(angle=touch_pose, speed=12, wait=True)
    if code != 0:
        print(f"Touch failed, code={code}")
        return

    print(f"Holding for {hold_time} seconds...")
    time.sleep(hold_time)

    print("Returning from touch...")
    arm.set_servo_angle(angle=front_pose, speed=12, wait=True)

    print("Returning home...")
    arm.set_servo_angle(angle=SAFE_POSE, speed=15, wait=True)

    print("Done.")

move_front_then_touch(hold_time=3.0)
arm.disconnect()