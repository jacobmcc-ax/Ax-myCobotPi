#!/usr/bin/env python3

import cv2
import numpy as np
import subprocess
import time
import os

class VisualServoMac:
    def __init__(self):
        self.pi_ip = "192.168.2.2"
        self.pi_user = "er"
        self.max_joint_change = 15.0  # Max degrees per movement
        self.image_path = "/Users/jacobmccarran_ax/Downloads/robot/camera_capture.jpg"

        # Current robot state (start from zero position)
        self.current_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def capture_image(self):
        """Capture image using local camera"""
        print("Capturing image...")
        result = subprocess.run(["python", "capture_camera.py"],
                              cwd="/Users/jacobmccarran_ax/Downloads/robot",
                              capture_output=True, text=True)
        if result.returncode == 0:
            print("Image captured successfully")
            return True
        else:
            print(f"Image capture failed: {result.stderr}")
            return False

    def analyze_image(self):
        """Analyze image to find robot arm and resistor positions"""
        try:
            img = cv2.imread(self.image_path)
            if img is None:
                print("Could not load image")
                return None, None, img

            # Convert to HSV for better color detection
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # Find robot arm (blue/light blue parts)
            lower_blue = np.array([80, 50, 50])
            upper_blue = np.array([130, 255, 255])
            robot_mask = cv2.inRange(hsv, lower_blue, upper_blue)

            # Find robot arm contours
            robot_contours, _ = cv2.findContours(robot_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            robot_end_pos = None
            if robot_contours:
                # Find the contour that's most likely the end effector
                # Look for the rightmost and lowest contour
                best_contour = None
                best_score = -1

                for contour in robot_contours:
                    if cv2.contourArea(contour) > 100:  # Filter small noise
                        moments = cv2.moments(contour)
                        if moments['m00'] != 0:
                            cx = int(moments['m10'] / moments['m00'])
                            cy = int(moments['m01'] / moments['m00'])
                            # Score based on being rightmost and lowest (higher score = more likely end effector)
                            score = cx + cy  # Simple scoring
                            if score > best_score:
                                best_score = score
                                best_contour = contour
                                robot_end_pos = (cx, cy)

            # Find resistor/circuit components
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Look for dark rectangular components (resistors, chips, etc.)
            # Use adaptive threshold to find components
            thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                         cv2.THRESH_BINARY, 11, 2)

            # Find contours in the thresholded image
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            resistor_pos = None
            best_resistor_area = 0

            # Look for components in the circuit board area
            height, width = img.shape[:2]
            circuit_area = (width//4, height//2, 3*width//4, height)  # Rough circuit board area

            for contour in contours:
                area = cv2.contourArea(contour)
                if 200 < area < 5000:  # Component size range
                    rect = cv2.boundingRect(contour)
                    cx, cy = rect[0] + rect[2]//2, rect[1] + rect[3]//2

                    # Check if it's in the circuit board area
                    if (circuit_area[0] < cx < circuit_area[2] and
                        circuit_area[1] < cy < circuit_area[3]):

                        aspect_ratio = rect[2] / max(rect[3], 1)
                        if 1.5 < aspect_ratio < 8:  # Resistor-like aspect ratio
                            if area > best_resistor_area:
                                best_resistor_area = area
                                resistor_pos = (cx, cy)

            # Fallback: if no resistor found, target center of circuit area
            if resistor_pos is None:
                resistor_pos = (width//2, int(height * 0.65))
                print("No specific resistor detected, targeting circuit center")

            # Create visualization
            vis_img = img.copy()
            if robot_end_pos:
                cv2.circle(vis_img, robot_end_pos, 10, (0, 255, 0), -1)  # Green circle for robot
                cv2.putText(vis_img, "Robot", (robot_end_pos[0]+15, robot_end_pos[1]),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            if resistor_pos:
                cv2.circle(vis_img, resistor_pos, 10, (0, 0, 255), -1)  # Red circle for resistor
                cv2.putText(vis_img, "Target", (resistor_pos[0]+15, resistor_pos[1]),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            # Save visualization
            cv2.imwrite("/Users/jacobmccarran_ax/Downloads/robot/analysis_visualization.jpg", vis_img)

            return robot_end_pos, resistor_pos, vis_img

        except Exception as e:
            print(f"Error analyzing image: {e}")
            return None, None, None

    def calculate_movement(self, robot_pos, resistor_pos):
        """Calculate required joint angle changes"""
        if robot_pos is None or resistor_pos is None:
            return None

        # Calculate pixel errors
        error_x = resistor_pos[0] - robot_pos[0]  # Horizontal error
        error_y = resistor_pos[1] - robot_pos[1]  # Vertical error

        print(f"Pixel errors: X={error_x}, Y={error_y}")

        # Convert to joint angle changes (this is a simplified mapping)
        # In a real system, this would use proper inverse kinematics

        angle_changes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Simple heuristic mappings:
        # J1 (base rotation) - affects X position
        if abs(error_x) > 20:  # Deadband to prevent jittering
            angle_changes[0] = -error_x * 0.05  # Scale factor

        # J2 (shoulder) - affects Y position and reach
        if abs(error_y) > 20:
            angle_changes[1] = error_y * 0.03

        # J3 (elbow) - fine adjustment for reach
        if abs(error_y) > 30:
            angle_changes[2] = -error_y * 0.02

        # If robot is close, move down towards the resistor
        distance = np.sqrt(error_x**2 + error_y**2)
        if distance < 100:  # Close to target
            angle_changes[1] -= 2  # Move shoulder down
            angle_changes[2] -= 1  # Adjust elbow

        # Limit changes to max allowed
        for i in range(len(angle_changes)):
            angle_changes[i] = max(-self.max_joint_change,
                                 min(self.max_joint_change, angle_changes[i]))

        return angle_changes

    def send_angles_to_pi(self, angles):
        """Send joint angles to Pi via SSH"""
        try:
            # Format command
            cmd = ["ssh", f"{self.pi_user}@{self.pi_ip}",
                   f"cd ~ && python robot_controller_pi.py {' '.join(map(str, angles))}"]

            print(f"Sending angles to Pi: {angles}")
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)

            if result.returncode == 0:
                print("Robot movement successful")
                print(result.stdout)
                return True
            else:
                print(f"Robot movement failed: {result.stderr}")
                return False

        except subprocess.TimeoutExpired:
            print("Robot command timed out")
            return False
        except Exception as e:
            print(f"Error sending to Pi: {e}")
            return False

    def is_close_enough(self, robot_pos, resistor_pos):
        """Check if robot is close enough to target (touching/very close)"""
        if robot_pos is None or resistor_pos is None:
            return False

        distance = np.sqrt((robot_pos[0] - resistor_pos[0])**2 +
                          (robot_pos[1] - resistor_pos[1])**2)
        return distance < 15  # Very close threshold - touching the resistor

    def run_control_loop(self, max_iterations=15):
        """Main visual servo control loop"""
        print("Starting visual servo control (Mac + Pi)...")
        print(f"Max iterations: {max_iterations}")
        print(f"Max joint change per iteration: {self.max_joint_change}°")

        for iteration in range(max_iterations):
            print(f"\n{'='*50}")
            print(f"ITERATION {iteration + 1}/{max_iterations}")
            print(f"{'='*50}")

            # Step 1: Capture image
            if not self.capture_image():
                print("Failed to capture image, skipping iteration")
                continue

            # Step 2: Analyze image
            robot_pos, resistor_pos, vis_img = self.analyze_image()

            if robot_pos is None:
                print("Could not detect robot arm")
                # Make exploratory movement
                self.current_angles[1] -= 5  # Move shoulder
                if not self.send_angles_to_pi(self.current_angles):
                    print("Failed to send exploratory movement")
                time.sleep(2)
                continue

            if resistor_pos is None:
                print("Could not detect resistor target")
                continue

            print(f"Robot detected at: {robot_pos}")
            print(f"Resistor target at: {resistor_pos}")

            # Step 3: Check if close enough
            if self.is_close_enough(robot_pos, resistor_pos):
                print("\n🎯 SUCCESS! Robot arm is positioned above the resistor!")
                break

            # Step 4: Calculate movement
            angle_changes = self.calculate_movement(robot_pos, resistor_pos)
            if angle_changes is None:
                print("Could not calculate movement")
                continue

            # Step 5: Apply changes to current angles
            for i in range(len(self.current_angles)):
                self.current_angles[i] += angle_changes[i]

            print(f"Angle changes: {[f'{x:.1f}' for x in angle_changes]}")
            print(f"New target angles: {[f'{x:.1f}' for x in self.current_angles]}")

            # Step 6: Send to Pi
            if not self.send_angles_to_pi(self.current_angles):
                print("Failed to send movement command")
                continue

            # Step 7: Wait for movement
            print("Waiting for robot movement...")
            time.sleep(4)

        print(f"\nControl loop completed after {iteration + 1} iterations")

def main():
    try:
        controller = VisualServoMac()
        controller.run_control_loop()
    except KeyboardInterrupt:
        print("\nControl loop interrupted by user")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()