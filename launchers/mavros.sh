#!/bin/bash

source /environment.sh

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# Launch MAVROS
FCU_URL=${FCU_URL:-"udp://:14540@"}
dt-exec ros2 run mavros mavros_node --ros-args -p fcu_url:=${FCU_URL}

# wait for app to end
dt-launchfile-join

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE
