#!/bin/bash

source /environment.sh

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# Launch MAVROS
#
# Select the MAVLink endpoint based on robot hardware:
#   - virtual robots: connect over UDP port 14540
#   - real robots:    connect over serial /dev/ttyACM0 with 921600 baud (recommended by PX4)
# An explicit FCU_URL environment variable always takes precedence.
if [ "${ROBOT_HARDWARE}" == "virtual" ]; then
  FCU_URL=${FCU_URL:-"udp://:14540@"}
else
  FCU_URL=${FCU_URL:-"serial:///dev/ttyACM0:921600"}
fi
MAVROS_CONFIG=${MAVROS_CONFIG:-"${DT_PROJECT_PATH}/assets/mavros/px4_config.yaml"}
dt-exec ros2 run mavros mavros_node --ros-args \
    -p "fcu_url:=${FCU_URL}" \
    --params-file "${MAVROS_CONFIG}"

# wait for app to end
dt-launchfile-join

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE
