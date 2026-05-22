from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "command_service",
                default_value="/mavros/cmd/command",
                description="MAVROS CommandLong service.",
            ),
            DeclareLaunchArgument(
                "status_text_topic",
                default_value="/mavros/statustext/recv",
                description="MAVROS STATUSTEXT receive topic.",
            ),
            Node(
                package="px4_calibration",
                executable="px4_calibration_node",
                name="px4_calibration",
                output="screen",
                parameters=[
                    {
                        "command_service": LaunchConfiguration("command_service"),
                        "status_text_topic": LaunchConfiguration("status_text_topic"),
                    }
                ],
            ),
        ]
    )
