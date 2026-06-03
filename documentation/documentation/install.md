# Installatie van de Robotiq package

## Clone repository

```bash
cd ~/my_ur_ws/src
git clone https://github.com/AvansMechatronica/ros2_robotiq_gripper.git
git clone https://github.com/tylerjw/serial.git -b ros2

```
## Build workspace
```
# Build the workspace
cd ~/my_ur_ws
colcon build --symlink-install
source install/setup.bash
```
# URDF(virtuele gripper)

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

:::{note}
Als je de gripper aan de robot hebt toegevoegd en je maak gebruik van de `Movegroup` interface, zorg er dan voor de je een dummy  `Planning Group` toevoegt aan je `moveit_config` package. Deze groep heeft alle joints van de gripper nodig. Hierdoor blijft de `Movegroup` interface werken.
Je kunt in geen geval de gripper vanuit de `Movegroup` interface aansturen, maar zonder deze dummy groep werkt de `Movegroup` interface helemaal niet meer.
:::
