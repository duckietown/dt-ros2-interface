#!/bin/bash

source /environment.sh

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# NOTE: Use the variable DT_PROJECT_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

if [ "$ROBOT_HARDWARE" == "raspberry_pi_64" ]; then
    # Launch the Raspberry Pi camera driver (picamera2 / libcamera).
    # Publishes <VEHICLE_NAME>/camera_node/image/compressed and
    # <VEHICLE_NAME>/camera_node/camera_info.
    ARGS=(
        "robot_name:=$VEHICLE_NAME"
        "camera_name:=${CAMERA_NAME:-front_center}"
        "width:=${CAMERA_WIDTH:-640}"
        "height:=${CAMERA_HEIGHT:-480}"
        "framerate:=${CAMERA_FPS:-30.0}"
        "rotation:=${CAMERA_ROTATION:-0}"
        "jpeg_quality:=${CAMERA_JPEG_QUALITY:-90}"
    )
    [ -n "${CAMERA_EXPOSURE_MODE:-}" ]     && ARGS+=("exposure_mode:=${CAMERA_EXPOSURE_MODE}")
    [ -n "${CAMERA_CALIBRATION_FILE:-}" ]  && ARGS+=("calibration_file:=${CAMERA_CALIBRATION_FILE}")

    dt-exec ros2 launch rpi_camera_driver rpi_camera_driver.launch.py "${ARGS[@]}"
else
    # Launch the camera driver node using launch file
    dt-exec ros2 launch camera_driver camera_driver.launch.py robot_name:=$VEHICLE_NAME
fi

# wait for app to end
dt-launchfile-join

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE
