#!/usr/bin/env python3

import cv2
import numpy as np
import time
from pymycobot.mycobot import MyCobot
import os

class VisualServoController:
    def __init__(self):
        # Connect to robot
        self.mc = MyCobot('/dev/ttyAMA0', 1000000)
        time.sleep(2)

        # Movement constraints
        self.max_joint_move = 10.0  # Maximum degrees per movement
        self.target_height = 100.0  # Target height above resistor (mm)
        self.position_tolerance = 20.0  # Position tolerance (mm)

        # Image capture settings
        self.image_counter = 0

    def capture_image(self):
        """Capture image from camera"""
        # For this implementation, we'll use the existing camera capture
        # In a real system, you'd capture directly here
        os.system("python /home/er/capture_camera.py")
        self.image_counter += 1
        return f"camera_capture_{self.image_counter}.jpg"

    def analyze_image(self, image_path="/home/er/camera_capture.jpg"):
        """Analyze image to find robot arm end effector and resistor positions"""
        try:
            # Load image
            img = cv2.imread(image_path)
            if img is None:
                print("Could not load image")
                return None, None

            # Convert to different color spaces for analysis
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find robot arm end effector (look for the blue/white robot parts)
            # Create mask for blue/light blue colors (robot arm)
            lower_blue = np.array([80, 50, 50])
            upper_blue = np.array([130, 255, 255])
            robot_mask = cv2.inRange(hsv, lower_blue, upper_blue)

            # Find contours for robot parts
            robot_contours, _ = cv2.findContours(robot_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            robot_end_pos = None
            if robot_contours:
                # Find the rightmost/lowest contour (likely the end effector)
                largest_contour = max(robot_contours, key=cv2.contourArea)
                moments = cv2.moments(largest_contour)
                if moments['m00'] != 0:
                    robot_cx = int(moments['m10'] / moments['m00'])
                    robot_cy = int(moments['m01'] / moments['m00'])
                    robot_end_pos = (robot_cx, robot_cy)

            # Find resistor (look for small rectangular components)
            # Use edge detection and contour finding
            edges = cv2.Canny(gray, 50, 150)
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            resistor_pos = None
            # Look for small rectangular contours that could be resistors
            for contour in contours:
                area = cv2.contourArea(contour)
                if 100 < area < 2000:  # Resistor size range
                    rect = cv2.boundingRect(contour)
                    aspect_ratio = rect[2] / rect[3]
                    if 1.5 < aspect_ratio < 6:  # Resistor aspect ratio
                        resistor_pos = (rect[0] + rect[2]//2, rect[1] + rect[3]//2)
                        break

            # If no specific resistor found, look for circuit board area
            if resistor_pos is None:
                # Look for the circuit board (darker rectangular area)
                # This is a fallback - aim for center of visible circuit area
                height, width = img.shape[:2]
                # Assume circuit board is in the center-lower portion
                resistor_pos = (width//2, int(height * 0.7))

            return robot_end_pos, resistor_pos

        except Exception as e:
            print(f"Error analyzing image: {e}")
            return None, None

    def calculate_movement(self, robot_pos, resistor_pos, current_coords):
        """Calculate required movement based on image analysis"""
        if robot_pos is None or resistor_pos is None:
            return None

        # Calculate pixel error
        pixel_error_x = resistor_pos[0] - robot_pos[0]
        pixel_error_y = resistor_pos[1] - robot_pos[1]

        print(f"Pixel error: X={pixel_error_x}, Y={pixel_error_y}")

        # Convert pixel error to real-world movement (rough approximation)
        # This would need calibration in a real system
        pixel_to_mm = 0.5  # Approximate conversion factor

        move_x = pixel_error_x * pixel_to_mm
        move_y = pixel_error_y * pixel_to_mm

        # Current position
        current_x, current_y, current_z = current_coords[:3]

        # Calculate target position (above resistor)
        target_x = current_x + move_x
        target_y = current_y + move_y
        target_z = self.target_height

        # Keep orientation horizontal
        target_coords = [target_x, target_y, target_z, 180.0, 0.0, 0.0]

        return target_coords

    def limit_joint_movement(self, current_angles, target_coords):
        """Limit movement to max degrees per joint"""
        try:
            # Get current coordinates
            current_coords = self.mc.get_coords()
            if not current_coords:
                return None

            # Calculate difference
            coord_diff = []
            for i in range(6):
                if i < len(target_coords) and i < len(current_coords):
                    diff = target_coords[i] - current_coords[i]
                    # Limit the change
                    max_change = 50 if i < 3 else 30  # Position vs orientation limits
                    diff = max(-max_change, min(max_change, diff))
                    coord_diff.append(current_coords[i] + diff)
                else:
                    coord_diff.append(current_coords[i] if i < len(current_coords) else 0)

            return coord_diff

        except Exception as e:
            print(f"Error limiting movement: {e}")
            return current_angles

    def is_close_enough(self, robot_pos, resistor_pos):
        """Check if robot is close enough to target"""
        if robot_pos is None or resistor_pos is None:
            return False

        distance = np.sqrt((robot_pos[0] - resistor_pos[0])**2 + (robot_pos[1] - resistor_pos[1])**2)
        return distance < 50  # Close enough in pixels

    def run_control_loop(self, max_iterations=20):
        """Main control loop"""
        print("Starting visual servo control loop...")

        for iteration in range(max_iterations):
            print(f"\n--- Iteration {iteration + 1} ---")

            # Step 1: Capture current image
            print("Capturing image...")
            time.sleep(1)  # Brief pause for stability

            # Step 2: Get current robot state
            current_coords = self.mc.get_coords()
            current_angles = self.mc.get_angles()
            print(f"Current position: {current_coords}")
            print(f"Current angles: {current_angles}")

            # Step 3: Analyze image
            print("Analyzing image...")
            robot_pos, resistor_pos = self.analyze_image()

            if robot_pos is None or resistor_pos is None:
                print("Could not detect robot or resistor positions")
                # Make a small exploratory movement
                current_coords[2] += 10  # Move up slightly
                limited_coords = self.limit_joint_movement(current_angles, current_coords)
                if limited_coords:
                    self.mc.send_coords(limited_coords, 30)
                    time.sleep(3)
                continue

            print(f"Robot end position (pixels): {robot_pos}")
            print(f"Resistor position (pixels): {resistor_pos}")

            # Step 4: Check if close enough
            if self.is_close_enough(robot_pos, resistor_pos):
                print("SUCCESS: Robot arm is close to resistor!")
                break

            # Step 5: Calculate required movement
            target_coords = self.calculate_movement(robot_pos, resistor_pos, current_coords)
            if target_coords is None:
                print("Could not calculate movement")
                continue

            # Step 6: Limit movement and execute
            limited_coords = self.limit_joint_movement(current_angles, target_coords)
            if limited_coords:
                print(f"Moving to: {limited_coords}")
                self.mc.send_coords(limited_coords, 30)
                time.sleep(4)  # Wait for movement to complete

        print("\nControl loop finished")

def main():
    try:
        controller = VisualServoController()
        controller.run_control_loop()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()