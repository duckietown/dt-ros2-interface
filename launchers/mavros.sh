#!/bin/bash

source /environment.sh

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# Launch MAVROS
# The params file enables setpoint_attitude.use_quaternion (so the
# /mavros/setpoint_attitude/attitude PoseStamped topic exists) and sets
# setpoint_raw.thrust_scaling=1.0. Without it, attitude+thrust OFFBOARD
# control (the altitude PID) cannot work.
FCU_URL=${FCU_URL:-"udp://:14540@"}
MAVROS_CONFIG=${MAVROS_CONFIG:-"${DT_PROJECT_PATH}/assets/mavros/px4_config.yaml"}
dt-exec ros2 run mavros mavros_node --ros-args \
    -p fcu_url:=${FCU_URL} \
    --params-file ${MAVROS_CONFIG}

# wait for app to end
dt-launchfile-join

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE
