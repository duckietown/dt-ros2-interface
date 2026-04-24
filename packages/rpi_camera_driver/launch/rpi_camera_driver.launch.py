#!/usr/bin/env python3
"""Launch file for the Raspberry Pi camera driver node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("robot_name", default_value="", description="Robot namespace"),
        DeclareLaunchArgument("camera_name", default_value="front_center"),
        DeclareLaunchArgument("frame_id", default_value="camera_color_optical_frame"),
        DeclareLaunchArgument("width", default_value="640"),
        DeclareLaunchArgument("height", default_value="480"),
        DeclareLaunchArgument("framerate", default_value="30.0"),
        DeclareLaunchArgument("rotation", default_value="0"),
        DeclareLaunchArgument("jpeg_quality", default_value="90"),
        DeclareLaunchArgument("exposure_mode", default_value=""),
        DeclareLaunchArgument("calibration_file", default_value=""),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]

    node = Node(
        package="rpi_camera_driver",
        executable="rpi_camera_driver_node",
        name="rpi_camera_driver_node",
        namespace=LaunchConfiguration("robot_name"),
        parameters=[{
            "camera_name": LaunchConfiguration("camera_name"),
            "frame_id": LaunchConfiguration("frame_id"),
            "width": LaunchConfiguration("width"),
            "height": LaunchConfiguration("height"),
            "framerate": LaunchConfiguration("framerate"),
            "rotation": LaunchConfiguration("rotation"),
            "jpeg_quality": LaunchConfiguration("jpeg_quality"),
            "exposure_mode": LaunchConfiguration("exposure_mode"),
            "calibration_file": LaunchConfiguration("calibration_file"),
        }],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        output="screen",
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription(args + [node])


if __name__ == "__main__":
    generate_launch_description()
