#!/bin/bash

source /environment.sh

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# Initialize launch file
dt-launchfile-init

# Launch the LED driver node using launch file
dt-exec ros2 launch led_driver led_driver.launch.py robot_name:=$VEHICLE_NAME

# Wait for app to end
dt-launchfile-join


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

