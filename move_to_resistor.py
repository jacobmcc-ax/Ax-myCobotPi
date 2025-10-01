#!/usr/bin/env python3

from pymycobot.mycobot import MyCobot
import time

def main():
    try:
        # Connect to MyCobot
        mc = MyCobot('/dev/ttyAMA0', 1000000)
        
        print("Connecting to robot...")
        time.sleep(2)
        
        # Step 1: Move all joints to 0 degrees
        print("Step 1: Moving all joints to 0 degrees...")
        zero_angles = [0, 0, 0, 0, 0, 0]
        mc.send_angles(zero_angles, 50)
        time.sleep(3)  # Wait for movement to complete
        print("Current angles:", mc.get_angles())
        
        # Step 2: J1=10°, J2=-60°
        print("Step 2: Moving J1 to 10°, J2 to -60°...")
        step2_angles = [10, -60, 0, 0, 0, 0]
        mc.send_angles(step2_angles, 50)
        time.sleep(3)
        print("Current angles:", mc.get_angles())
        
        # Step 3: J1=10°, J2=-60°, J3=-45°
        print("Step 3: Adding J3 to -45°...")
        step3_angles = [10, -60, -45, 0, 0, 0]
        mc.send_angles(step3_angles, 50)
        time.sleep(3)
        print("Current angles:", mc.get_angles())
        
        # Step 4: J1=10°, J2=-60°, J3=-45°, J4=-15°
        print("Step 4: Adding J4 to -15°...")
        step4_angles = [10, -60, -45, -15, 0, 0]
        mc.send_angles(step4_angles, 50)
        time.sleep(3)
        print("Current angles:", mc.get_angles())
        
        print("Resistor position sequence complete!")
        
    except Exception as e:
        print(f"Error: {e}")
        print("Make sure the robot is connected and pymycobot library is installed.")

if __name__ == "__main__":
    main()