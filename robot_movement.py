from xarm.wrapper import XArmAPI
import math
import time

class RobotMovement:
    def __init__(self, ip_address='192.168.1.205'):
        # Connect to your robot
        self.arm = XArmAPI(ip_address)

    def prepare_arm(self):
        """Wakes up the arm and puts it in the correct mode if it isn't already."""
        state = self.arm.get_state()[1]
        if state != 0:
            print("\n[System] Initializing arm...")
            self.arm.clean_error()
            self.arm.motion_enable(enable=True)
            self.arm.set_mode(0)  # Position control mode
            self.arm.set_state(state=0)
            time.sleep(0.5)

    def go_to_safe_home(self):
        """Call this at the very beginning of your script to start from the safety state."""
        self.prepare_arm()
        wire_safe_pose = [0, -30.0, 0, 0, 0.0]
        print("Moving to Wire-Safe Home position...")
        self.arm.set_servo_angle(angle=wire_safe_pose, speed=30, wait=True)
        time.sleep(0.5)

    # =====================================================================
    # THE SINGLE POINT CALL FUNCTION (YOUR ORIGINAL LOGIC)
    # =====================================================================
    def move_to(self, x, y, z, speed=30, steps=50):
        """Moves the arm from its current location to (x, y, z) in a smooth straight line."""
        self.prepare_arm()
        
        # 1. Get the current actual position of the robot to use as the starting point
        code, current_pose = self.arm.get_position()
        if code != 0 or not current_pose:
            print("Failed to read current position. Cannot calculate straight line.")
            return False
            
        start_x, start_y, start_z = current_pose[0], current_pose[1], current_pose[2]
        
        # Target orientation settings for horizontal movement
        target_roll = 180.0
        target_pitch = 0.0

        # 2. Divide the path from current position to target position into mini-steps
        for i in range(1, steps + 1):
            t = i / steps
            # Linear interpolation math
            target_x = start_x + (x - start_x) * t
            target_y = start_y + (y - start_y) * t
            target_z = start_z + (z - start_z) * t

            # calculate the matching 5-axis base tracking angle
            target_yaw = math.degrees(math.atan2(target_y, target_x))

            # Query the factory calibrated configuration matrix
            ret, joints = self.arm.get_inverse_kinematics(
                pose=[target_x, target_y, target_z, target_roll, target_pitch, target_yaw], 
                input_is_radian=False
            )
            
            if ret == 0:
                # Keep J5 smoothly matched to the base rotation to prevent wild twisting
                joints[4] = joints[0] 
                
                # Arrive precisely at the final points
                is_destination = (i == steps)
                self.arm.set_servo_angle(angle=joints, speed=speed, mvacc=600, wait=is_destination)
            else:
                print(f"Path generation blocked mid-move at step {i}")
                return False
                
        print(f"Successfully arrived at point: X:{x}, Y:{y}, Z:{z}")
        return True
