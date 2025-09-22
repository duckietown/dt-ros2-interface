#!/bin/bash

source /environment.sh

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# Initialize launch file
dt-launchfile-init

# Launch right wheel encoder node using launch file
dt-exec ros2 launch wheel_encoder_driver wheel_encoder_driver.launch.py robot_name:=$VEHICLE_NAME wheel:=right

# Wait for app to end
dt-launchfile-join

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE
