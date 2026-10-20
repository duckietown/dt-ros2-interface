#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for the image relayer node."""

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='',
        description='Name of the robot (leave empty for auto-detection)'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )

    image_relayer_node = Node(
        package='image_relayer',
        executable='image_relayer_node',
        name='image_relayer_node',
        namespace=LaunchConfiguration('robot_name'),
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([
        robot_name_arg,
        log_level_arg,
        image_relayer_node,
    ])


if __name__ == '__main__':
    generate_launch_description()
