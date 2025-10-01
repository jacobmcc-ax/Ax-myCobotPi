#!/usr/bin/env python3

from pymycobot.mycobot import MyCobot
import time

def main():
    try:
        # Connect to MyCobot (using 1000000 baud rate)
        mc = MyCobot('/dev/ttyAMA0', 1000000)
        
        print("Connecting to robot...")
        time.sleep(2)
        
        # Move all joints to 0 degrees
        print("Moving all joints to 0 degrees...")
        zero_angles = [0, 0, 0, 0, 0, 0]  # 6 joints, all to 0 degrees
        
        # Send angles command with speed 50
        mc.send_angles(zero_angles, 50)
        
        print("Command sent. Robot should move to zero position.")
        print("Current angles:", mc.get_angles())
        
    except Exception as e:
        print(f"Error: {e}")
        print("Make sure the robot is connected and pymycobot library is installed.")

if __name__ == "__main__":
    main()