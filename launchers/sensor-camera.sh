#!/bin/bash

source /environment.sh

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# TODO: update to ROS2 launch
#exec ros2 launch --wait camera_driver camera_driver_node.launch veh:=$VEHICLE_NAME

# initialize launch file
dt-launchfile-init

# launch subscriber
ros2 run camera_driver camera_driver_node

# wait for app to end
dt-launchfile-join

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE
