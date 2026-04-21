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

def go_home(speed=20):
    prepare_arm()
    code = arm.set_servo_angle(angle=SAFE_POSE, speed=speed, wait=True)
    if code != 0:
        print(f"Failed to reach home, code={code}")
        return False
    time.sleep(0.3)
    return True

def get_pos():
    _, pos = arm.get_position()
    return pos  # [x, y, z, roll, pitch, yaw]

def get_joints():
    _, angles = arm.get_servo_angle()
    return list(angles[:5])

def move_to(x, y, z, speed=15):
    """
    Move to target XYZ (mm) from home pose.
    Strategy: 
      - J1 controls Y (left/right rotation of base)
      - J2+J3 together control X (reach) and Z (height)
      We compute deltas from home, then scale to joint adjustments
      based on what the arm actually does from SAFE_POSE.
    """
    go_home()
    time.sleep(0.5)

    home_pos = get_pos()
    home_joints = get_joints()
    print(f"Home TCP:    X:{home_pos[0]:.1f} Y:{home_pos[1]:.1f} Z:{home_pos[2]:.1f}")
    print(f"Home joints: {[f'{a:.1f}' for a in home_joints]}")

    delta_x = x - home_pos[0]
    delta_y = y - home_pos[1]
    delta_z = z - home_pos[2]
    print(f"Target deltas: dX:{delta_x:.1f} dY:{delta_y:.1f} dZ:{delta_z:.1f}")

    # These scale factors come from xArm5 geometry:
    # J1 (base rotation): 1 degree ≈ ~4.6mm Y change at 263mm reach
    # J2 (shoulder):      1 degree ≈ affects X and Z
    # J3 (elbow):         1 degree ≈ affects X and Z (opposite to J2)
    # Tuned conservatively to avoid self-collision
    J1_DEG_PER_MM_Y = 1.0 / 4.6
    J2_DEG_PER_MM_Z = 1.0 / 5.0   # J2 up = Z up
    J3_DEG_PER_MM_X = 1.0 / 4.0   # J3 negative = reach further

    j1_delta = delta_y * J1_DEG_PER_MM_Y
    j2_delta = delta_z * J2_DEG_PER_MM_Z
    j3_delta = delta_x * J3_DEG_PER_MM_X * -1  # negative because J3 negative = more reach

    target_joints = home_joints[:]
    target_joints[0] += j1_delta
    target_joints[1] += j2_delta
    target_joints[2] += j3_delta

    # Clamp to xArm5 joint limits
    target_joints[0] = max(-360, min(360,  target_joints[0]))
    target_joints[1] = max(-118, min(120,  target_joints[1]))
    target_joints[2] = max(-225, min(11,   target_joints[2]))
    target_joints[3] = max(-97,  min(180,  target_joints[3]))
    target_joints[4] = max(-360, min(360,  target_joints[4]))

    print(f"Target joints: {[f'{a:.1f}' for a in target_joints]}")

    code = arm.set_servo_angle(angle=target_joints, speed=speed, wait=True)
    if code != 0:
        print(f"Move failed code={code}")
        arm.clean_error()
        arm.set_state(0)
        return False

    actual = get_pos()
    print(f"Actual TCP: X:{actual[0]:.1f} Y:{actual[1]:.1f} Z:{actual[2]:.1f}")
    err = ((actual[0]-x)**2 + (actual[1]-y)**2 + (actual[2]-z)**2)**0.5
    print(f"Error from target: {err:.1f}mm")
    return True


# --- First just map what the arm actually does ---
# Move each joint independently to learn the real scale factors
go_home()
print("\n=== CALIBRATION: moving J3 by -10 degrees ===")
j = get_joints()
j[2] += -10
arm.set_servo_angle(angle=j, speed=10, wait=True)
pos = get_pos()
print(f"After J3-10: X:{pos[0]:.1f} Y:{pos[1]:.1f} Z:{pos[2]:.1f}")

go_home()
print("\n=== CALIBRATION: moving J2 by +10 degrees ===")
j = get_joints()
j[1] += 10
arm.set_servo_angle(angle=j, speed=10, wait=True)
pos = get_pos()
print(f"After J2+10: X:{pos[0]:.1f} Y:{pos[1]:.1f} Z:{pos[2]:.1f}")

go_home()
print("\n=== CALIBRATION: moving J1 by +10 degrees ===")
j = get_joints()
j[0] += 10
arm.set_servo_angle(angle=j, speed=10, wait=True)
pos = get_pos()
print(f"After J1+10: X:{pos[0]:.1f} Y:{pos[1]:.1f} Z:{pos[2]:.1f}")

go_home()

time.sleep(1)
arm.disconnect()