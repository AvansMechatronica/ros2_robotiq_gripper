# ROS2 Grippercontroller

De Grippercontroller is een ROS2-node die de communicatie tussen ROS2 en een Robotiq 2F-85 gripper verzorgt. De node vertaalt ROS2-acties en -services naar commando's die de gripper kan begrijpen, en publiceert de status van de gripper via ROS2-topics. Hieronder worden de belangrijkste ROS2-interfaces van de Grippercontroller beschreven, inclusief de actionsserver, services en gepubliceerde topics.

## ROS-interfaces
### Actionsserver

- Naam: `/robotiq_gripper_controller/gripper_cmd`
- Type: `control_msgs/action/ParallelGripperCommand`

Implementatienotities:
- de node gebruikt de eerste waarde van `goal.command.position` en `goal.command.effort`,
- de positie wordt begrensd op `[0, max_joint_position]`,
- de kracht wordt geïnterpreteerd als percentage en begrensd op `[0, 100]`.

### Services

#### `set_gripper_force`
Type: `my_robotiq_controller/srv/SetForce`

Aanvraag:
- `uint8 force`

Antwoord:
- `bool success`
- `string message`

Voorbeeld:

```bash
ros2 service call /set_gripper_force my_robotiq_controller/srv/SetForce "{force: 128}"
```

#### `set_gripper_speed`
Type: `my_robotiq_controller/srv/SetSpeed`

Aanvraag:
- `uint8 speed`

Antwoord:
- `bool success`
- `string message`

Voorbeeld:

```bash
ros2 service call /set_gripper_speed my_robotiq_controller/srv/SetSpeed "{speed: 128}"
```

### Gepubliceerd topic

- `joint_states` (`sensor_msgs/msg/JointState`)

De node publiceert zes gekoppelde Robotiq-vingergewrichten voor URDF-compatibele visualisatie/besturing.

## Snel actievoorbeeld

```bash
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/ParallelGripperCommand \
  "{command: {position: [0.04], effort: [60.0]}}"
```
