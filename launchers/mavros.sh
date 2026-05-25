#!/bin/bash

source /environment.sh

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# Launch MAVROS
# The params file enables quaternion attitude setpoints for the MAVROS
# setpoint_attitude plugin and sets setpoint_raw.thrust_scaling=1.0.
# Without it, MAVROS uses its built-in defaults and the current
# PoseStamped+thrust OFFBOARD control path is not consumed.
#
# Select the MAVLink endpoint based on the robot hardware:
#   - virtual robots: PX4 SITL exposes MAVLink over UDP on port 14540
#   - real robots:    the PX4 flight controller is connected over USB (CDC-ACM),
#                     enumerating as /dev/ttyACM0 (baud is nominal for CDC-ACM)
# An explicit FCU_URL environment variable always takes precedence.
if [ "${ROBOT_HARDWARE}" == "virtual" ]; then
  FCU_URL=${FCU_URL:-"udp://:14540@"}
else
  FCU_URL=${FCU_URL:-"serial:///dev/ttyACM0:57600"}
fi
MAVROS_CONFIG=${MAVROS_CONFIG:-"${DT_PROJECT_PATH}/assets/mavros/px4_config.yaml"}
dt-exec ros2 run mavros mavros_node --ros-args \
    -p "fcu_url:=${FCU_URL}" \
    --params-file "${MAVROS_CONFIG}"

# wait for app to end
dt-launchfile-join

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE
