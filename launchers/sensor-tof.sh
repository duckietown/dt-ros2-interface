#!/bin/bash

source /environment.sh

# Initialize launch file.
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

exec ros2 run tof_driver tof_driver_node --ros-args \
	-r "__node:=tof_${SENSOR_NAME}" \
	-r "__ns:=/${VEHICLE_NAME}" \
	-p "sensor_name:=${SENSOR_NAME}"

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# Wait for app to end.
dt-launchfile-join
