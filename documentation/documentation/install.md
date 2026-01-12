# Installatie van de Robotiq package

```bash
cd ~/my_ur_ws/src
git clone https://github.com/AvansMechatronica/ros2_robotiq_gripper.git
git clone https://github.com/RoverRobotics-forks/serial-ros2.git serial

```

```
# Build the workspace
cd ~/my_ur_ws
colcon build --symlink-install
source install/setup.bash
```

Voeg een virtuele gripper toe aan je URDF bestand van je project:

```xml
<?xml version="1.0"?>
<robot name="example_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!--
    Inhoud bestaande URDF file
    -->

    <!-- parameters -->
    <xacro:arg name="use_fake_hardware" default="true" />
    <xacro:arg name="com_port" default="/dev/ttyUSB0" />

    <!-- Import macros -->
    <xacro:include filename="$(find robotiq_description)/urdf/robotiq_2f_85_macro.urdf.xacro" />

    <xacro:robotiq_gripper name="RobotiqGripperHardwareInterface" prefix="" parent="<link naar de end-effector>" use_fake_hardware="$(arg use_fake_hardware)" com_port="$(arg com_port)">
        <origin xyz="0 0 0" rpy="0 0 0" />
    </xacro:robotiq_gripper>

</robot>
```

De grippercontroller kan als volgt gestart worden:

```bash
ros2 launch robotiq_description robotiq_control.launch.py
```




