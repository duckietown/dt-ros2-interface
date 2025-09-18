#!/bin/bash
source /environment.sh
dt-launchfile-init --quiet

dt-exec ros2 run wheel_encoder_driver wheel_encoder_driver_node -p wheel:=left &
dt-exec ros2 run wheel_encoder_driver wheel_encoder_driver_node -p wheel:=right

dt-launchfile-join --quiet
