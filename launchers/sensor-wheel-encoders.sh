#!/bin/bash
source /environment.sh
dt-launchfile-init --quiet

# Launch both wheel encoder nodes using launch files
dt-exec ros2 launch wheel_encoder_driver wheel_encoder_driver.launch.py robot_name:=$VEHICLE_NAME wheel:=left --ros-args -r __node:=wheel_encoder_left &
dt-exec ros2 launch wheel_encoder_driver wheel_encoder_driver.launch.py robot_name:=$VEHICLE_NAME wheel:=right --ros-args -r __node:=wheel_encoder_right

dt-launchfile-join --quiet
