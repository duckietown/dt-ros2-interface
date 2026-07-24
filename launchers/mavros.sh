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
# Bridge the real ToF topic onto what the distance_sensor plugin expects (see px4_config.yaml).
TOF_RANGE_TOPIC=${TOF_RANGE_TOPIC:-"/${HOSTNAME}/bottom_tof_driver_node/range"}

# PX4 echoes our ToF data back out, which spams a harmless error in the mavros log.
# Tell it to stop (MAV_CMD 511). This doesn't persist on the FC, so redo it every launch,
# once the FC link is actually up (no fixed delay — poll instead of guessing).
(
  for _ in $(seq 1 30); do
    grep -q "connected: true" <(ros2 topic echo /mavros/state --once 2>/dev/null) && break
    sleep 2
  done
  for _ in 1 2 3 4 5 6; do
    ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong \
      "{broadcast: false, command: 511, confirmation: 0, param1: 132.0, param2: -1.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}" >/dev/null 2>&1 && break
    sleep 5
  done
) &

dt-exec ros2 run mavros mavros_node --ros-args \
    -p "fcu_url:=${FCU_URL}" \
    -r "/mavros/bottom_tof:=${TOF_RANGE_TOPIC}" \
    --params-file "${MAVROS_CONFIG}"

# wait for app to end
dt-launchfile-join

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE
