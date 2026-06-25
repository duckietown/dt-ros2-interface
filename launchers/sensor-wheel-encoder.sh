#!/bin/bash

source /environment.sh

# Initialize launch file.
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

exec ros2 run wheel_encoder_driver wheel_encoder_driver_node --ros-args \
	-r "__node:=wheel_encoder_${WHEEL}" \
	-r "__ns:=/${VEHICLE_NAME}" \
	-p "wheel:=${WHEEL}"

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# Wait for app to end.
dt-launchfile-join
