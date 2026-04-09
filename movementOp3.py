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

def move_front_then_touch(front_amount, touch_amount, hold_time=3.0):
    prepare_arm()

    print("Moving to safe home...")
    code = arm.set_servo_angle(angle=SAFE_POSE, speed=20, wait=True)
    if code != 0:
        print(f"Safe home failed, code={code}")
        return

    # front_amount controls how far in front the whole arm goes
    front_pose = SAFE_POSE[:]
    front_pose[2] = SAFE_POSE[2] - 1.5 *front_amount      # J3
    front_pose[1] = SAFE_POSE[1] +  2* front_amount      # J2

    print(f"Moving arm in front with front_amount={front_amount}")
    print(f"J2: {SAFE_POSE[1]:.1f} -> {front_pose[1]:.1f}")
    print(f"J3: {SAFE_POSE[2]:.1f} -> {front_pose[2]:.1f}")

    code = arm.set_servo_angle(angle=front_pose, speed=15, wait=True)
    if code != 0:
        print(f"Front move failed, code={code}")
        return

    # touch_amount controls the little touch motion
    touch_pose = front_pose[:]
    touch_pose[3] = front_pose[3] + touch_amount

    print(f"Doing touch with touch_amount={touch_amount}")
    code = arm.set_servo_angle(angle=touch_pose, speed=12, wait=True)
    if code != 0:
        print(f"Touch failed, code={code}")
        return

    print(f"Holding for {hold_time} seconds...")
    time.sleep(hold_time)

    print("Returning from touch...")
    code = arm.set_servo_angle(angle=front_pose, speed=12, wait=True)
    if code != 0:
        print(f"Touch return failed, code={code}")
        return

    print("Returning home...")
    code = arm.set_servo_angle(angle=SAFE_POSE, speed=15, wait=True)
    if code != 0:
        print(f"Return home failed, code={code}")
        return

    print("Done.")