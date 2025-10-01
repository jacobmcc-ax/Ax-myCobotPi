#!/usr/bin/env python3

from pymycobot.mycobot import MyCobot
import sys
import time

def main():
    if len(sys.argv) != 7:
        print("Usage: python robot_controller_pi.py J1 J2 J3 J4 J5 J6")
        print("Example: python robot_controller_pi.py 0 -30 -45 0 0 0")
        sys.exit(1)

    try:
        # Parse joint angles from command line
        angles = [float(arg) for arg in sys.argv[1:7]]

        # Connect to robot
        mc = MyCobot('/dev/ttyAMA0', 1000000)
        time.sleep(1)

        # Get current angles for comparison
        current_angles = mc.get_angles()
        print(f"Current angles: {current_angles}")
        print(f"Target angles: {angles}")

        # Send new angles
        mc.send_angles(angles, 30)  # Speed 30
        print("Movement command sent")

        # Wait for movement to complete
        time.sleep(3)

        # Report final position
        final_angles = mc.get_angles()
        final_coords = mc.get_coords()
        print(f"Final angles: {final_angles}")
        print(f"Final coordinates: {final_coords}")

    except ValueError:
        print("Error: All arguments must be numbers")
        sys.exit(1)
    except Exception as e:
        print(f"Robot error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()