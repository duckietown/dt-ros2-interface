#!/bin/bash

source /environment.sh

# Initialize launch file.
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

exec ros2 launch duckiematrix_interface duckiematrix_interface.launch.py

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# Wait for app to end.
dt-launchfile-join
