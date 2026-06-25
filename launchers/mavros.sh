#!/bin/bash

source /environment.sh

# Initialize launch file.
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# The params file enables quaternion attitude setpoints for the MAVROS
# setpoint_attitude plugin and sets setpoint_raw.thrust_scaling=1.0.
# Without it, MAVROS uses its built-in defaults and the current
# PoseStamped+thrust OFFBOARD control path is not consumed.
FCU_URL=${FCU_URL:-"udp://:14540@"}
MAVROS_CONFIG=${MAVROS_CONFIG:-"${DT_PROJECT_PATH}/assets/mavros/px4_config.yaml"}
exec ros2 run mavros mavros_node --ros-args \
    -p "fcu_url:=${FCU_URL}" \
    --params-file "${MAVROS_CONFIG}"

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# Wait for app to end.
dt-launchfile-join
