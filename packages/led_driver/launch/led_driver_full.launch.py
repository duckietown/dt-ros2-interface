#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    """Generate launch description for LED driver with various configurations."""
    
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='',
        description='Name of the robot (leave empty for auto-detection)'
    )
    
    lights_name_arg = DeclareLaunchArgument(
        'lights_name',
        default_value='base',
        description='Name of the lights configuration (default: base)'
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
    
    # Create the LED driver node
    led_driver_node = Node(
        package='led_driver',
        executable='led_driver_node',
        name='led_driver_node',
        parameters=[{
            'robot_name': LaunchConfiguration('robot_name'),
            'lights_name': LaunchConfiguration('lights_name'),
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
        lights_name_arg,
        use_sim_time_arg,
        log_level_arg,
        led_driver_node,
    ])


if __name__ == '__main__':
    generate_launch_description()