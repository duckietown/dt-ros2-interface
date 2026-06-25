#!/bin/bash

source /environment.sh

# Initialize launch file.
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

exec ros2 launch control_mapper control_mapper.launch.py robot_name:=$VEHICLE_NAME

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# Wait for app to end.
dt-launchfile-join
