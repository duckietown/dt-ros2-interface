# Tests

This directory currently contains the camera transport benchmark tooling used to compare HTTP and SHM behavior in the ROS2 camera bridge.

## Contents

- `benchmark_camera_transport.py`: benchmark utility for capture and comparison.
- `PLACE_YOUR_TESTS_HERE`: placeholder file for future test additions.

## Benchmark modes

`benchmark_camera_transport.py` provides three subcommands:

- `capture`: measures transport-ingress behavior.
  For HTTP it subscribes to the camera JPEG topic through the DTPS API.
  For SHM it opens the SHM reader directly.
  This mode reports latency from the embedded camera timestamp to benchmark ingress, plus container CPU and memory.
- `capture-output`: measures the existing ROS2 output topic.
  This mode does not open the SHM FIFO and is the preferred way to compare HTTP and SHM throughput end to end.
  It reports output fps, camera-source-to-external-ROS2-subscriber delay, and container CPU and memory.
- `compare`: compares one HTTP and one SHM ROS2 output-capture result only.

The ROS2 and ROS1 benchmark tools are intentionally separate. Do not combine
their result files: this script accepts only schema-v2 ROS2 output captures
with the same measurement surface and source-timestamp origin.

## Recommended HTTP vs SHM Workflow

Use `capture-output` for the HTTP vs SHM comparison. It avoids the direct SHM reader side effect that can split FIFO notifications with the live consumer.

Example HTTP run:

```bash
cd /home/ubuntu/duckietown/dt-ros2-interface

python3 tests/benchmark_camera_transport.py capture-output \
  --transport http \
  --robot-name morpheus \
  --ros-domain-id 42 \
  --mode-container ros2-camera \
  --containers driver-camera ros2-camera \
  --output-json /tmp/ros2-camera-http-output.json
```

Example SHM run:

```bash
cd /home/ubuntu/duckietown/dt-ros2-interface

python3 tests/benchmark_camera_transport.py capture-output \
  --transport shm \
  --robot-name morpheus \
  --ros-domain-id 42 \
  --mode-container ros2-camera \
  --containers driver-camera ros2-camera \
  --output-json /tmp/ros2-camera-shm-output.json
```

Compare the two outputs:

```bash
cd /home/ubuntu/duckietown/dt-ros2-interface

python3 tests/benchmark_camera_transport.py compare \
  /tmp/ros2-camera-http-output.json \
  /tmp/ros2-camera-shm-output.json \
  --output-json /tmp/ros2-camera-output-compare.json
```

`capture-output` measures from the timestamp embedded by the camera producer
to receipt by the external ROS2 subscriber. It therefore includes DTPS,
bridge conversion, and ROS2 delivery. All participants must share a wall
clock; a nonzero `negative_timestamp_delta_count` means the result should not
be compared.

## Runtime requirements

- `capture` requires DTPS Python dependencies and access to the switchboard socket.
- `capture-output` requires ROS2 Python packages such as `rclpy` and `sensor_msgs`.
- Both capture modes require access to container stats through either the Docker CLI or the Python Docker SDK.

If your local shell does not have the required DTPS or ROS2 runtime available, run the script inside a helper container based on `duckietown/dt-ros2-interface:ente-arm64v8` and mount the Docker socket. For direct HTTP capture, also mount the DTPS socket path.

For direct SHM capture, mount the named camera SHM volume at `/dtps-shm`, for example `-v "${DT_CAMERA_SHM_VOLUME:-duckietown-camera-shm}:/dtps-shm"`. The default channel is `/dtps-shm/front_center`; use `--shm-path` for a different channel.

## JSON output format

Use `--output-json` to write results wherever you want.

Each JSON result includes:

- `benchmark_schema_version`: current results use version `2`; schema-v1
  results use a different timing boundary and cannot be compared.
- `dtps_transport`, `ros_transport`, and `transport_path`: the DTPS mode and
  the fixed ROS2 output path for the run.
- `timestamp_origin` and `timestamp_clock`: the provenance and clock expected
  by the timing summary.
- `measurement_surface`: `transport_ingress` or `ros2_output_topic`
- `frame_metrics`: fps, timing summaries, dropped frames, and payload-size summaries
- `resource_metrics`: per-container CPU and memory samples

## Notes

- Prefer `capture-output` when comparing HTTP and SHM throughput.
- Use `capture` when you specifically want transport-ingress latency at the HTTP or SHM boundary.
- Direct SHM `capture` is deliberately blocked unless
  `--allow-direct-shm-reader` is supplied. Use it only with the live SHM
  consumer stopped, because the FIFO wake-ups are not broadcast to readers.
- The live stack must be in the transport mode you are measuring. `--mode-container` is only a safety check; it does not switch the stack for you.
- For an HTTP run, the producer must leave `DT_CAMERA_SHM_ONLY_JPEG` unset or set it to `0`. The benchmark checks `driver-camera` by default; use `--producer-container` for a differently named producer.
