#!/usr/bin/env python3
from xarm.wrapper import XArmAPI
import time

arm = XArmAPI('192.168.1.205')

#def maintenance_state():
    #arm.set_position(x=0, y=150, z=0, relative=True, speed=speed, mvacc=acc, wait=True)
    #arm.motion_enable(False)


def horizontal_sweep():
    # 1. Clear the previous Code 31 Collision
    arm.clean_error()
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(2)

    # 2. Settings for a smooth move
    # We use a lower acceleration to prevent torque/collision errors
    speed = 100
    acc = 500 

    print("Starting horizontal sweep (Left to Right)...")
    
    try:
        # Move RIGHT 150mm (approx 6 inches)
        print("Moving Right...")
        arm.set_position(x=0, y=150, z=0, relative=True, speed=speed, mvacc=acc, wait=True)
        
        time.sleep(0.5)
        
        # Move LEFT 300mm (back past center)
        print("Moving Left...")
        arm.set_position(x=0, y=-300, z=0, relative=True, speed=speed, mvacc=acc, wait=True)
        
        # Back to Center
        print("Returning to center...")
        arm.set_position(x=0, y=150, z=0, relative=True, speed=speed, mvacc=acc, wait=True)

    except Exception as e:
        print(f"Sweep interrupted: {e}")

try:
    horizontal_sweep()
finally:
    # 3. Secure the arm
    arm.set_state(3)
    arm.disconnect()
    print("Robot Disconnected.")
