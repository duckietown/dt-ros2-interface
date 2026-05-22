#!/bin/bash

source /environment.sh

dt-launchfile-init

dt-exec ros2 launch px4_calibration px4_calibration.launch.py

dt-launchfile-join
