# Robotiq Gripper Control Scripts

This directory contains Python scripts for controlling the Robotiq gripper in ROS2.

## Available Scripts

### 1. Command-Line Control (`control_gripper`)

A command-line interface for gripper control.

#### Usage

```bash
# Make sure the gripper controller is running first
ros2 launch my_robotiq_controller robotiq_controller.launch.py 

# In another terminal:



# Open the gripper
ros2 run robotiq_app control_gripper --open

# Close the gripper
ros2 run robotiq_app control_gripper --close

# Move to specific position (0.0 = closed, 0.085 = open for 2F-85)
ros2 run robotiq_app control_gripper --position 0.04

# Set position with custom effort (in Newtons)
ros2 run robotiq_app control_gripper --position 0.02 --effort 100.0

# Activate and then open (multiple commands)
ros2 run robotiq_app control_gripper --activate --open
```

#### Command-Line Options

- `--activate`: Activate or reactivate the gripper
- `--open`: Fully open the gripper
- `--close`: Fully close the gripper
- `--position METERS`: Move to specific position (0.0 to 0.085 for 2F-85, 0.0 to 0.140 for 2F-140)
- `--effort NEWTONS`: Set maximum effort/force (default: 50.0, max: 235.0)

### 2. GUI Control (`gripper_gui.py`)

A graphical user interface for gripper control and monitoring.

#### Usage

```bash
# Make sure the gripper controller is running
ros2 launch robotiq_description robotiq_control.launch.py

# In another terminal, launch the GUI
ros2 run robotiq_app gripper_gui.py
```

#### GUI Features

**Status Display:**
- Real-time current position display
- Position progress bar
- Velocity indicator
- Moving/Idle status with color indicator
- Last command result

**Controls:**
- Activate button to initialize the gripper
- Position slider (0.0 to 0.085 meters)
- Effort/Force slider (10 to 235 Newtons)
- Send Command button

**Quick Actions:**
- Fully Open: Opens gripper completely
- Half Open: Opens gripper to 50% position
- Fully Close: Closes gripper completely
- Gentle Close: Closes with low force (20N)
- Firm Grip: Closes with high force (100N)

## Prerequisites

### ROS2 Dependencies

Make sure you have the following packages installed:

```bash
sudo apt install ros-${ROS_DISTRO}-control-msgs
sudo apt install ros-${ROS_DISTRO}-sensor-msgs
sudo apt install ros-${ROS_DISTRO}-std-srvs
```

### Python Dependencies

The scripts use standard Python libraries included with ROS2:
- `rclpy` - ROS2 Python client library
- `tkinter` - GUI library (for gripper_gui.py)

Install tkinter if not already available:
```bash
sudo apt install python3-tk
```

## Starting the Gripper System

### For Simulation/Testing

```bash
# Terminal 1: Launch the gripper controllers
ros2 launch robotiq_description robotiq_control.launch.py

# Terminal 2: Use the control scripts
ros2 run robotiq_app control_gripper --activate --open
# OR
ros2 run robotiq_app gripper_gui.py
```

### For Real Hardware

```bash
# Terminal 1: Launch with hardware interface
ros2 launch robotiq_description robotiq_control.launch.py \
  use_fake_hardware:=false \
  robot_ip:=<GRIPPER_IP>

# Terminal 2: Use the control scripts
ros2 run robotiq_app gripper_gui.py
```

## Gripper Position Reference

### 2F-85 Gripper
- Fully Closed: `0.000 m`
- Half Open: `0.0425 m`
- Fully Open: `0.085 m`

### 2F-140 Gripper
- Fully Closed: `0.000 m`
- Half Open: `0.070 m`
- Fully Open: `0.140 m`

## Force/Effort Guidelines

- **Light Grip (10-30 N)**: For delicate objects
- **Medium Grip (30-80 N)**: For most objects
- **Strong Grip (80-150 N)**: For heavy or secure holding
- **Maximum (150-235 N)**: For maximum holding force

## Troubleshooting

### "Action server not available"

Make sure the controller is running:
```bash
ros2 control list_controllers
```

You should see `robotiq_gripper_controller` in the list.

### "Activation service not available"

Check if the activation controller is loaded:
```bash
ros2 service list | grep reactivate
```

You should see `/robotiq_activation_controller/reactivate_gripper`.

### GUI not showing position updates

Verify joint states are being published:
```bash
ros2 topic echo /joint_states
```

### Permission denied when running scripts

The scripts are installed as ROS2 executables, so no chmod is needed when using `ros2 run`.

## Examples

### Example 1: Pick and Place Sequence

```bash
# Open gripper
ros2 run robotiq_app control_gripper --open

# Close with medium force to grasp object
ros2 run robotiq_app control_gripper --close --effort 60.0

# (Move arm to new position - not shown)

# Open to release
ros2 run robotiq_app control_gripper --open
```

### Example 2: Precision Grip

```python
#!/usr/bin/env python3
import subprocess
import time

# Activate
subprocess.run(["ros2", "run", "robotiq_app", "control_gripper", "--activate"])
time.sleep(1)

# Open
subprocess.run(["ros2", "run", "robotiq_app", "control_gripper", "--open"])
time.sleep(2)

# Precision grip (20mm opening, gentle force)
subprocess.run(["ros2", "run", "robotiq_app", "control_gripper", "--position", "0.020", "--effort", "25.0"])
```

## ROS2 Topics and Services

### Subscribed Topics
- `/joint_states` (sensor_msgs/JointState): Gripper joint states

### Action Servers
- `/robotiq_gripper_controller/gripper_cmd` (control_msgs/GripperCommand): Position control

### Services
- `/robotiq_activation_controller/reactivate_gripper` (std_srvs/Trigger): Gripper activation

## Additional Information

For more details about the Robotiq gripper driver, see the main [README.md](README.md) file.

For hardware interface documentation, see the [robotiq_driver](robotiq_driver/) package.

For URDF and visualization, see the [robotiq_description](robotiq_description/) package.

