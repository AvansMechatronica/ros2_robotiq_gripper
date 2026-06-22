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
    update_rate_arg = DeclareLaunchArgument(
        "update_rate",
        default_value="50.0",
        description="Joint state publish/update rate in Hz",
    )
    # For 2F-85: 0.085m, for 2F-140: 0.140m
    max_joint_position_arg = DeclareLaunchArgument(
        "max_joint_position",
        default_value="0.085",
        description="Maximum joint position mapped from raw gripper feedback",
    )
    startup_homing_arg = DeclareLaunchArgument(
        "startup_homing",
        default_value="true",
        description="Run open-close-open startup homing and fail launch if the gripper does not respond",
    )
    startup_timeout_arg = DeclareLaunchArgument(
        "startup_timeout",
        default_value="8.0",
        description="Timeout in seconds for each startup homing movement",
    )
    startup_speed_arg = DeclareLaunchArgument(
        "startup_speed",
        default_value="128",
        description="Raw Robotiq speed value used during startup homing",
    )
    startup_force_arg = DeclareLaunchArgument(
        "startup_force",
        default_value="80",
        description="Raw Robotiq force value used during startup homing",
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
                "update_rate": LaunchConfiguration("update_rate"),
                "max_joint_position": LaunchConfiguration("max_joint_position"),
                "startup_homing": LaunchConfiguration("startup_homing"),
                "startup_timeout": LaunchConfiguration("startup_timeout"),
                "startup_speed": LaunchConfiguration("startup_speed"),
                "startup_force": LaunchConfiguration("startup_force"),
            }
        ],
    )

    return LaunchDescription(
        [
            port_arg,
            baudrate_arg,
            timeout_arg,
            slave_address_arg,
            update_rate_arg,
            max_joint_position_arg,
            startup_homing_arg,
            startup_timeout_arg,
            startup_speed_arg,
            startup_force_arg,
            controller_node,
        ]
    )
