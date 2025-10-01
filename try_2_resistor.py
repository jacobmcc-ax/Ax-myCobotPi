#!/usr/bin/env python3

from pymycobot.mycobot import MyCobot
import time

def main():
    try:
        # Connect to MyCobot
        mc = MyCobot('/dev/ttyAMA0', 1000000)

        print("Connecting to robot...")
        time.sleep(2)

        # Get current position for reference
        current_coords = mc.get_coords()
        print("Current coordinates:", current_coords)
        current_angles = mc.get_angles()
        print("Current angles:", current_angles)

        # Step 1: Adjust orientation to make end effector parallel to work surface
        # Keep similar X,Y,Z position but adjust orientation angles
        # Target: Make RX closer to 180° for horizontal alignment, adjust RZ for parallel to resistor
        print("Step 1: Adjusting orientation to be parallel to resistor...")

        # Use coordinate control for precise positioning
        # Keep similar position but adjust orientation
        target_coords = [235.0, -25.0, 80.0, 180.0, 0.0, 0.0]  # RX=180° for horizontal, RY=0°, RZ=0° for parallel
        mc.send_coords(target_coords, 30)
        time.sleep(4)

        new_coords = mc.get_coords()
        new_angles = mc.get_angles()
        print("After orientation adjustment:")
        print("Coordinates:", new_coords)
        print("Joint angles:", new_angles)

        # Step 2: Fine-tune if needed - small adjustment to ensure parallel alignment
        print("Step 2: Fine-tuning for optimal parallel alignment...")

        # Slight adjustment to RZ if needed for better parallel alignment to resistor direction
        fine_tune_coords = [235.0, -25.0, 80.0, 180.0, 0.0, -90.0]  # RZ=-90° to align with resistor orientation
        mc.send_coords(fine_tune_coords, 20)
        time.sleep(3)

        final_coords = mc.get_coords()
        final_angles = mc.get_angles()
        print("Final position:")
        print("Coordinates:", final_coords)
        print("Joint angles:", final_angles)

        print("Parallel resistor alignment complete!")

    except Exception as e:
        print(f"Error: {e}")
        print("Make sure the robot is connected and pymycobot library is installed.")

if __name__ == "__main__":
    main()