from xarm.wrapper import XArmAPI
import time

arm = XArmAPI('127.0.0.1')

def prepare_arm():
    arm.clean_error()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(state=0)
    time.sleep(0.5)

def debubble_motion(cycles=6, j4_amp=5.0, j5_amp=5.0, speed=1000):
    """
    Gentle anti-bubble vibration from whatever position the arm is currently at.
    Only moves J4 and J5 (wrist joints) so it won't cause self-collision
    regardless of where the arm is positioned.

    Args:
        cycles  : number of up/down left/right cycles
        j4_amp  : degrees of J4 wrist tilt (keep small, e.g. 3-10)
        j5_amp  : degrees of J5 wrist rotation (keep small, e.g. 3-10)
        speed   : joint speed for vibration moves
    """
    prepare_arm()

    # Read current joint angles as the base — works from any position
    code, current_joints = arm.get_servo_angle()
    if code != 0:
        print(f"Failed to read current joint angles, code={code}")
        return
    
    base_pose = list(current_joints[:5])
    print(f"Debubbling from current pose: {[f'{a:.1f}' for a in base_pose]}")

    # Safety check --> keep J4 and J5 within limits after amplitude applied
    # xArm5 limits --> J4 = -97 to 180, J5 = -360 to 360
    j4_base = base_pose[3]
    j5_base = base_pose[4]

    j4_amp = min(j4_amp, 
                 (180 - j4_base) / 3,   # don't exceed upper limit
                 (j4_base + 97)  / 3)   # don't exceed lower limit
    
    j5_amp = min(j5_amp,
                 360 - abs(j5_base))     # don't exceed either limit

    if j4_amp <= 0 or j5_amp <= 0:
        print("Not enough joint range to vibrate safely from this position.")
        return

    print(f"Using j4_amp={j4_amp:.1f}° j5_amp={j5_amp:.1f}°")
    print("Starting de-bubble motion...")

    for i in range(cycles):
        pose_up    = base_pose[:]
        pose_down  = base_pose[:]
        pose_left  = base_pose[:]
        pose_right = base_pose[:]

        pose_up[3]    = j4_base + 3 * j4_amp
        pose_down[3]  = j4_base - 3 * j4_amp
        pose_left[4]  = j5_base - j5_amp
        pose_right[4] = j5_base + j5_amp

        arm.set_servo_angle(angle=pose_left,  speed=speed, wait=True)
        arm.set_servo_angle(angle=pose_up,    speed=speed, wait=True)
        arm.set_servo_angle(angle=pose_right, speed=speed, wait=True)
        arm.set_servo_angle(angle=pose_down,  speed=speed, wait=True)

    # Return to exact pose we started from
    arm.set_servo_angle(angle=base_pose, speed=speed, wait=True)
    print("De-bubble motion complete.")

# Call from wherever the arm already is, without going home first
debubble_motion(cycles=6, j4_amp=5.0, j5_amp=5.0, speed=1000)

arm.disconnect()