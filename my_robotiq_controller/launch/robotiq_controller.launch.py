from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyUSB0",
        description="Serial port used to communicate with the Robotiq gripper",
    )
    baudrate_arg = DeclareLaunchArgument(
        "baudrate",
        default_value="115200",
        description="Serial baudrate",
    )
    timeout_arg = DeclareLaunchArgument(
        "timeout",
        default_value="0.5",
        description="Serial timeout in seconds",
    )
    slave_address_arg = DeclareLaunchArgument(
        "slave_address",
        default_value="9",
        description="Modbus slave address",
    )

    controller_node = Node(
        package="my_robotiq_controller",
        executable="robotiq_controller",
        name="my_robotiq_controller",
        output="screen",
        parameters=[
            {
                "port": LaunchConfiguration("port"),
                "baudrate": LaunchConfiguration("baudrate"),
                "timeout": LaunchConfiguration("timeout"),
                "slave_address": LaunchConfiguration("slave_address"),
            }
        ],
    )

    return LaunchDescription(
        [
            port_arg,
            baudrate_arg,
            timeout_arg,
            slave_address_arg,
            controller_node,
        ]
    )