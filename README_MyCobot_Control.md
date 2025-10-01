# MyCobot 280 PI Control Guide

This guide explains how to connect to and control your MyCobot 280 PI robotic arm using Python on Raspberry Pi.

## Prerequisites

- MyCobot 280 PI robotic arm
- Raspberry Pi with Python 3 installed
- pymycobot library (should be in `/home/er/pymycobot/`)

## Hardware Setup

1. **Power on the robot**: Make sure your MyCobot 280 PI is powered on and all joints are unlocked
2. **Physical connection**: The robot should be connected to the Raspberry Pi via the built-in serial connection
3. **Verify connection**: The robot uses `/dev/ttyAMA0` serial port at 1000000 baud rate

## Software Setup

### 1. Python Environment
Make sure you have the pymycobot library available:
```bash
# The library should be located at:
/home/er/pymycobot/
```

### 2. Import the Library
```python
import sys
sys.path.append('/home/er/pymycobot')
from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD
```

## Basic Connection

### Connect to the Robot
```python
# Use the predefined PI settings
mc = MyCobot(PI_PORT, PI_BAUD)  # PI_PORT = "/dev/ttyAMA0", PI_BAUD = 1000000

# Or specify directly:
mc = MyCobot("/dev/ttyAMA0", 1000000)
```

### Check Robot Status
```python
# Check if robot is powered on
is_powered = mc.is_power_on()
print(f"Robot power status: {is_powered}")  # Should return 1 if powered on
```

## Basic Movement Commands

### Move to Specific Joint Angles
```python
# Define joint angles (in degrees) for all 6 joints
angles = [joint1, joint2, joint3, joint4, joint5, joint6]
speed = 30  # Speed from 1-100

# Send angles to robot
mc.send_angles(angles, speed)
```

### Common Positions

#### Zero Position (All joints at 0°)
```python
zero_angles = [0, 0, 0, 0, 0, 0]
mc.send_angles(zero_angles, 30)
```

#### Circuit Working Position
```python
circuit_angles = [90, -45, -30, 0, 45, 0]
mc.send_angles(circuit_angles, 25)
```

## Available Scripts

This directory contains several ready-to-use scripts:

### 1. `move_to_zero.py`
Moves all joints to 0° position
```bash
python3 move_to_zero.py
```
**Note:** The original script uses incorrect connection parameters. Use `move_to_zero_correct.py` instead for reliable operation.

### 1a. `move_to_zero_correct.py`
Fixed version that uses correct connection parameters (PI_PORT and PI_BAUD)
```bash
python3 move_to_zero_correct.py
```

### 2. `move_to_circuit_pi.py`
Complete script that:
- Connects to the robot
- Moves to zero position first (safety)
- Then moves arm closer to circuit/electronics area
```bash
python3 move_to_circuit_pi.py
```

## Understanding Joint Angles

The MyCobot 280 has 6 degrees of freedom:

- **Joint 1 (Base)**: Rotates the entire arm left/right (0° = forward)
- **Joint 2 (Shoulder)**: Moves arm up/down from base
- **Joint 3 (Elbow)**: Bends the arm forward/backward
- **Joint 4 (Wrist Roll)**: Rotates the wrist
- **Joint 5 (Wrist Pitch)**: Tilts end effector up/down
- **Joint 6 (Wrist Yaw)**: Rotates end effector left/right

### Angle Ranges
- Most joints: -170° to +170°
- Joint 5: -180° to +180°

### Safety Tips
- Always start with low speeds (20-40)
- Move to zero position before large movements
- Ensure clear workspace before moving

## Example Complete Script

```python
#!/usr/bin/env python3
import sys
sys.path.append('/home/er/pymycobot')
from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD
import time

def main():
    try:
        # Connect
        mc = MyCobot(PI_PORT, PI_BAUD)
        print("Connected!")

        # Check status
        if mc.is_power_on():
            print("Robot is powered on")
        else:
            print("Robot not powered - check connection")
            return

        # Move to zero
        print("Moving to zero position...")
        mc.send_angles([0, 0, 0, 0, 0, 0], 30)
        time.sleep(5)

        # Move to custom position
        print("Moving to working position...")
        mc.send_angles([45, -30, -45, 0, 30, 0], 25)
        time.sleep(5)

        print("Movement complete!")

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
```

## Troubleshooting

### Connection Issues
- **Error: Permission denied**: Make sure user has access to serial ports
- **Error: Device busy**: Another program might be using the robot
- **Error: No such device**: Check if robot is properly connected

### Movement Issues
- **Robot not moving**: Check if robot is powered and unlocked
- **Jerky movement**: Try lower speeds (10-30)
- **Out of range errors**: Check joint angle limits

### Common Solutions
```bash
# Check available serial ports
ls /dev/tty*

# Check if port exists
ls -la /dev/ttyAMA0

# Kill any processes using the port
sudo fuser -k /dev/ttyAMA0
```

## API Reference

### Key Methods
- `send_angles(angles, speed)`: Move to joint angles
- `get_angles()`: Get current joint angles
- `is_power_on()`: Check if robot is powered
- `is_moving()`: Check if robot is currently moving
- `stop()`: Emergency stop
- `release_all_servos()`: Unlock all joints
- `power_on()`: Power on servos
- `power_off()`: Power off servos

### Speed Settings
- `1-100`: Valid speed range
- `20-40`: Recommended for general use
- `10-20`: Slow/precise movements
- `50-80`: Faster movements (be careful)

## Safety Notes

⚠️ **Important Safety Guidelines:**
- Always ensure clear workspace before moving robot
- Start with slow speeds when testing new positions
- Keep emergency stop accessible
- Never leave robot unattended during movement
- Check joint limits before sending commands
- Power off robot when not in use

---

*This guide covers the basic control of MyCobot 280 PI. For advanced features like coordinate control, inverse kinematics, or custom trajectories, refer to the pymycobot documentation.*