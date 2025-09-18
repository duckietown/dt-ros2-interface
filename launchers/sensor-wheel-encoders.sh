#!/bin/bash
source /environment.sh
dt-launchfile-init --quiet

dt-exec ros2 run wheel_encoder_driver wheel_encoder_driver_node -p wheel:=left  --ros-args -r __node:=wheel_encoder_left  &
dt-exec ros2 run wheel_encoder_driver wheel_encoder_driver_node -p wheel:=right --ros-args -r __node:=wheel_encoder_right

dt-launchfile-join --quiet
