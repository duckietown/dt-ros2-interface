#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for camera driver node."""
    
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='',
        description='Name of the robot (leave empty for auto-detection)'
    )
    
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='front_center',
        description='Name of the camera configuration'
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
    
    # Create the camera driver node
    camera_driver_node = Node(
        package='camera_driver',
        executable='camera_driver_node',
        name='camera_driver_node',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[{
            'camera_name': LaunchConfiguration('camera_name'),
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
        camera_name_arg,
        use_sim_time_arg,
        log_level_arg,
        camera_driver_node,
    ])


if __name__ == '__main__':
    generate_launch_description()
