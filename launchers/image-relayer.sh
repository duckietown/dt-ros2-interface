#!/bin/bash

source /environment.sh

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# Initialize launch file
dt-launchfile-init

# Launch the image relayer node (ROS 2 compressed image topics -> DTPS, for the Duckietown Viewer)
dt-exec ros2 launch image_relayer image_relayer.launch.py robot_name:=$VEHICLE_NAME

# Wait for app to end
dt-launchfile-join

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE
