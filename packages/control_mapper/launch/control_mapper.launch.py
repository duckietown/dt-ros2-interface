#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable


def generate_launch_description():
    """Generate launch description for control mapper node."""

    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value=EnvironmentVariable(name='VEHICLE_NAME', default_value=''),
        description='Name of the robot; defaults to VEHICLE_NAME environment variable or auto-detection.'
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

    def _make_node(context, *args, **kwargs):
        # Resolve provided robot name; if empty, auto-detect via dt_robot_utils
        provided = LaunchConfiguration('robot_name').perform(context)
        if not provided:
            try:
                from dt_robot_utils import get_robot_name
                robot_ns = get_robot_name()
            except Exception:
                robot_ns = ''
        else:
            robot_ns = provided

        node = Node(
            package='control_mapper',
            executable='control_mapper_node',
            name='control_mapper_node',
            namespace=robot_ns,
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            output='screen',
            emulate_tty=True,
            respawn=True,
            respawn_delay=2.0,
        )
        return [node]

    return LaunchDescription([
        robot_name_arg,
        use_sim_time_arg,
        log_level_arg,
        OpaqueFunction(function=_make_node),
    ])


if __name__ == '__main__':
    generate_launch_description()
