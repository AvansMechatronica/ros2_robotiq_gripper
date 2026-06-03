# my_robotiq_controller

ROS 2 package that provides a simple controller node for a Robotiq 2F gripper through `robotiq_driver`.

It exposes:
- an action server for open/close commands,
- services to change force and speed,
- a `joint_states` publisher for visualization and integration with robot descriptions.

## What this node does

`robotiq_controller` connects to the gripper over serial (Modbus RTU), activates the gripper, and then:

- accepts goals on `/robotiq_gripper_controller/gripper_cmd` (`control_msgs/action/ParallelGripperCommand`),
- publishes `sensor_msgs/msg/JointState` on `joint_states`,
- provides:
  - `set_gripper_force` (`my_robotiq_controller/srv/SetForce`)
  - `set_gripper_speed` (`my_robotiq_controller/srv/SetSpeed`)

## Package structure

- `src/robotiq_controller.cpp`: main node implementation
- `include/my_robotiq_controller/robotiq_controller.hpp`: node class definition
- `launch/robotiq_controller.launch.py`: launch file with runtime arguments
- `srv/SetForce.srv`: force service (`uint8 force`)
- `srv/SetSpeed.srv`: speed service (`uint8 speed`)

## Dependencies

Declared in `package.xml` / `CMakeLists.txt`:

- `rclcpp`
- `rclcpp_action`
- `control_msgs`
- `sensor_msgs`
- `robotiq_driver`
- `rosidl_default_generators` / `rosidl_default_runtime`

## Build

From your workspace root:

```bash
colcon build --packages-select my_robotiq_controller
source install/setup.bash
```

## Run

### Launch (recommended)

```bash
ros2 launch my_robotiq_controller robotiq_controller.launch.py
```

### Launch with custom serial settings

```bash
ros2 launch my_robotiq_controller robotiq_controller.launch.py \
  port:=/dev/ttyUSB0 baudrate:=115200 timeout:=0.5 slave_address:=9 \
  update_rate:=50.0 max_joint_position:=0.085
```

## Parameters

| Parameter | Type | Default | Description |
|---|---:|---:|---|
| `port` | string | `/dev/ttyUSB0` | Serial device path |
| `baudrate` | int | `115200` | Serial baud rate |
| `timeout` | double | `0.5` | Serial timeout (seconds) |
| `slave_address` | int | `9` | Modbus slave ID |
| `update_rate` | double | `50.0` | Joint state publish rate (Hz) |
| `max_joint_position` | double | `0.085` | Modeled max open position |

`max_joint_position` examples:
- Robotiq 2F-85: `0.085`
- Robotiq 2F-140: `0.140`

## ROS interfaces

### Action server

- Name: `/robotiq_gripper_controller/gripper_cmd`
- Type: `control_msgs/action/ParallelGripperCommand`

Implementation notes:
- the node uses the first entry of `goal.command.position` and `goal.command.effort`,
- position is clamped to `[0, max_joint_position]`,
- effort is interpreted as percentage and clamped to `[0, 100]`.

### Services

#### `set_gripper_force`
Type: `my_robotiq_controller/srv/SetForce`

Request:
- `uint8 force`

Response:
- `bool success`
- `string message`

Example:

```bash
ros2 service call /set_gripper_force my_robotiq_controller/srv/SetForce "{force: 128}"
```

#### `set_gripper_speed`
Type: `my_robotiq_controller/srv/SetSpeed`

Request:
- `uint8 speed`

Response:
- `bool success`
- `string message`

Example:

```bash
ros2 service call /set_gripper_speed my_robotiq_controller/srv/SetSpeed "{speed: 128}"
```

### Published topic

- `joint_states` (`sensor_msgs/msg/JointState`)

The node publishes six coupled Robotiq finger joints for URDF-compatible visualization/control.

## Quick action example

```bash
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/ParallelGripperCommand \
  "{command: {position: [0.04], effort: [60.0]}}"
```

## Troubleshooting

- **Cannot open serial port**: verify `port` and Linux permissions (`dialout` group).
- **No response from gripper**: check wiring, baud rate, and `slave_address`.
- **Build include errors in editor**: source your ROS and workspace setup files so language tooling sees dependencies.

## Notes

- On startup, the node connects and activates the gripper.
- On shutdown, it deactivates and disconnects the driver.
- Internal conversion maps raw device position (`0..255`) to model position using `max_joint_position`.
