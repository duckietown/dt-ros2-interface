#!/usr/bin/env python3
"""Launch the Duckiematrix DTPS-to-ROS 2 bridge node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the Duckiematrix bridge node."""

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='',
        description='Name of the robot namespace'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )

    duckiematrix_interface_node = Node(
        package='duckiematrix_interface',
        executable='duckiematrix_interface_node',
        name='duckiematrix_interface_node',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([
        robot_name_arg,
        use_sim_time_arg,
        log_level_arg,
        duckiematrix_interface_node,
    ])


if __name__ == '__main__':
    generate_launch_description()