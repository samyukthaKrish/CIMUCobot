from xarm.wrapper import XArmAPI
import time

arm = XArmAPI('127.0.0.1')

def move_to_safe_home():
    arm.clean_error()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    time.sleep(0.5)

    # Wiring-friendly pose
    wire_safe_pose = [0, -30.0, 0, 0, 175.0]

    print("Moving to safe home...")
    code = arm.set_servo_angle(angle=wire_safe_pose, speed=20, wait=True)

    if code == 0:
        print("Safe home reached.")
        return wire_safe_pose
    else:
        print(f"Failed with code {code}")
        return None


def debubble_motion(cycles=6, j5_amp=4.0, speed=12, settle_time=0.4):
    """
    Gentle anti-bubble motion.
    Small slow wrist tilts with pauses so bubbles can escape.
    """
    base_pose = move_to_safe_home()
    if base_pose is None:
        return

    print("Starting de-bubble motion...")

    for _ in range(cycles):
        pose_left = base_pose[:]
        pose_right = base_pose[:]

        # only small wrist motion
        pose_left[4] = base_pose[4] - j5_amp
        pose_right[4] = base_pose[4] + j5_amp

        arm.set_servo_angle(angle=pose_left, speed=speed, wait=True)
        time.sleep(settle_time)

        arm.set_servo_angle(angle=pose_right, speed=speed, wait=True)
        time.sleep(settle_time)

    arm.set_servo_angle(angle=base_pose, speed=speed, wait=True)
    print("De-bubble motion complete.")


debubble_motion(cycles=6, j5_amp=4.0, speed=12, settle_time=0.4)

arm.disconnect()