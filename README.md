# Ax-myCobotPi

Visual servoing and control software for the MyCobot 280 PI robotic arm.

## Overview

This repository contains Python scripts for controlling the MyCobot 280 PI robotic arm with visual servoing capabilities. The system can autonomously position the robot arm above electronic components (resistors) using computer vision feedback.

## Features

- **Direct Robot Control**: Command-line interface for precise joint angle control
- **Visual Servoing**: Autonomous positioning using camera feedback and computer vision
- **Dual Operation Modes**:
  - Pi-only mode: All processing on Raspberry Pi
  - Mac+Pi mode: Vision processing on Mac, robot control on Pi via SSH
- **Safety Features**: Movement limiting and incremental positioning

## Hardware Requirements

- MyCobot 280 PI robotic arm
- Raspberry Pi (with robot connected via serial `/dev/ttyAMA0`)
- Logitech C920 webcam (or compatible)
- Network connection (for Mac+Pi mode)

## Software Requirements

- Python 3
- pymycobot library (installed at `/home/er/pymycobot/` on Pi)
- OpenCV (cv2)
- NumPy

## Quick Start

### Basic Robot Control

```bash
# Move robot to zero position
python3 zero_robot.py

# Move to predefined resistor position
python3 move_to_resistor.py

# Manual control with specific joint angles
python3 robot_controller_pi.py 0 -30 -45 0 0 0
```

### Visual Servoing

```bash
# Run visual servo on Pi (all-in-one)
python3 visual_servo_resistor.py

# Run visual servo from Mac (requires SSH access to Pi)
python3 visual_servo_mac.py
```

## How It Works

### Visual Servoing Control Loop

1. **Capture** image from webcam
2. **Detect** robot arm end effector (blue color detection in HSV space)
3. **Detect** target resistor (edge detection + contour analysis)
4. **Calculate** position error between robot and target
5. **Command** incremental movement to reduce error
6. **Repeat** until robot is positioned above target

### Architecture

- **robot_controller_pi.py**: Low-level robot control interface accepting joint angles
- **visual_servo_resistor.py**: Complete visual servoing running on Pi
- **visual_servo_mac.py**: Distributed system with vision on Mac, control on Pi
- **capture_camera.py**: Camera capture utility

## Configuration

### Robot Connection

All scripts use the following connection parameters:
```python
port = '/dev/ttyAMA0'
baud = 1000000
```

### Network (Mac+Pi mode)

Update the Pi IP address in `visual_servo_mac.py`:
```python
self.pi_ip = "192.168.2.2"
self.pi_user = "er"
```

### Safety Limits

Movement limits can be adjusted in the visual servo scripts:
- **visual_servo_resistor.py**: 50mm position change, 30° orientation change
- **visual_servo_mac.py**: 15° joint angle change per iteration

## Joint Configuration

The MyCobot 280 has 6 degrees of freedom:

| Joint | Description | Range |
|-------|-------------|-------|
| J1 | Base rotation | -170° to +170° |
| J2 | Shoulder | -170° to +170° |
| J3 | Elbow | -170° to +170° |
| J4 | Wrist roll | -170° to +170° |
| J5 | Wrist pitch | -180° to +180° |
| J6 | Wrist yaw | -170° to +170° |

## Safety

⚠️ **Important Safety Guidelines:**
- Ensure clear workspace before running any scripts
- Start with slow speeds (20-40 range)
- Keep emergency stop accessible
- Monitor first runs of visual servoing closely
- Never leave robot unattended during operation

## Troubleshooting

### Connection Issues
```bash
# Check if serial port exists
ls -la /dev/ttyAMA0

# Check for processes using the port
sudo fuser -k /dev/ttyAMA0
```

### Camera Issues
```bash
# Test camera capture
python3 capture_camera.py

# Check for saved image
ls -la camera_capture.jpg
```

### SSH Issues (Mac+Pi mode)
```bash
# Test SSH connection
ssh er@192.168.2.2

# Test robot control via SSH
ssh er@192.168.2.2 "cd ~ && python robot_controller_pi.py 0 0 0 0 0 0"
```

## Documentation

- **README.md** (this file): Quick start and overview
- **CLAUDE.md**: Detailed architecture and development guide
- **README_MyCobot_Control.md**: Comprehensive API reference and examples

## License

MIT

## Author

Developed at Ax
