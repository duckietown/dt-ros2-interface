#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    """Generate launch description for ToF driver node."""
    
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='',
        description='Name of the robot (leave empty for auto-detection)'
    )
    
    sensor_name_arg = DeclareLaunchArgument(
        'sensor_name',
        default_value='front_center',
        description='Name of the ToF sensor'
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
    
    # Create the ToF driver node
    tof_driver_node = Node(
        package='tof_driver',
        executable='tof_driver_node',
        name=[LaunchConfiguration('sensor_name'), TextSubstitution(text='_tof_driver_node')],
        namespace=LaunchConfiguration('robot_name'),
        parameters=[{
            'sensor_name': LaunchConfiguration('sensor_name'),
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
        sensor_name_arg,
        use_sim_time_arg,
        log_level_arg,
        tof_driver_node,
    ])


if __name__ == '__main__':
    generate_launch_description()
