#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for LED driver node."""
    
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='',
        description='Name of the robot'
    )
    
    lights_name_arg = DeclareLaunchArgument(
        'lights_name',
        default_value='base',
        description='Name of the lights configuration'
    )
    
    # Create the LED driver node
    led_driver_node = Node(
        package='led_driver',
        executable='led_driver_node',
        name='led_driver_node',
        parameters=[{
            'robot_name': LaunchConfiguration('robot_name'),
            'lights_name': LaunchConfiguration('lights_name'),
        }],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        robot_name_arg,
        lights_name_arg,
        led_driver_node,
    ])


if __name__ == '__main__':
    generate_launch_description()