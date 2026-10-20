#!/bin/bash
source /environment.sh
dt-launchfile-init --quiet

# Launch both wheel encoder nodes using launch files.
# NOTE: `dt-exec` already runs each command in the background and `dt-launchfile-join` waits for both.
# NOTE: the node names (<wheel>_wheel_encoder_driver_node) are set by the launch file; do not pass
#       `--ros-args` here, `ros2 launch` does not accept them.
dt-exec ros2 launch wheel_encoder_driver wheel_encoder_driver.launch.py robot_name:=$VEHICLE_NAME wheel:=left
dt-exec ros2 launch wheel_encoder_driver wheel_encoder_driver.launch.py robot_name:=$VEHICLE_NAME wheel:=right

dt-launchfile-join --quiet
