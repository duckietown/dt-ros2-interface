#!/bin/bash

source /environment.sh

# Initialize launch file.
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

exec ros2 launch px4_calibration px4_calibration.launch.py

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# Wait for app to end.
dt-launchfile-join
