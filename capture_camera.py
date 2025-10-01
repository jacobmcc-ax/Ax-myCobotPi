#!/usr/bin/env python3

import cv2

def capture_from_c920():
    # Try to find the C920 webcam
    cap = cv2.VideoCapture(0)  # Try index 0 first
    
    if not cap.isOpened():
        print("Could not open webcam at index 0")
        return False
    
    print("Webcam opened successfully")
    
    # Give camera time to initialize
    import time
    time.sleep(2)
    
    # Capture frame
    ret, frame = cap.read()
    
    if ret:
        # Save the image
        cv2.imwrite('/Users/jacobmccarran_ax/Downloads/robot/camera_capture.jpg', frame)
        print("Image captured from C920!")
        cap.release()
        return True
    else:
        print("Failed to capture frame")
        cap.release()
        return False

if __name__ == "__main__":
    capture_from_c920()