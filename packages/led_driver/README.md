# LED Driver Launch Files

This directory contains launch files for the LED driver system.

## Launch Files

### 1. `led_driver.launch.py`
Basic launch file for the LED driver node.

**Usage:**
```bash
ros2 launch led_driver led_driver.launch.py
```

**Parameters:**
- `robot_name`: Name of the robot (default: auto-detected)
- `lights_name`: Name of the lights configuration (default: "base")

### 2. `led_driver_full.launch.py`
Comprehensive launch file with additional configuration options.

**Usage:**
```bash
ros2 launch led_driver led_driver_full.launch.py
```

**Parameters:**
- `robot_name`: Name of the robot (default: auto-detected)
- `lights_name`: Name of the lights configuration (default: "base")
- `use_sim_time`: Use simulation time (default: false)
- `log_level`: Logging level - debug, info, warn, error, fatal (default: info)

**Example with parameters:**
```bash
ros2 launch led_driver led_driver_full.launch.py robot_name:=duckiebot01 lights_name:=base log_level:=debug
```

### 3. `../launch/led_system.launch.py`
System-level launch file that includes the LED driver launch file.

**Usage:**
```bash
# From the workspace root
ros2 launch launch/led_system.launch.py
```

## Node Information

The LED driver node (`led_driver_node`) controls the LEDs on Duckiebots with the following LED layout:

| Index | Position (relative to direction of movement) |
|-------|---------------------------------------------|
| 0     | Front left                                  |
| 1     | Rear left                                   |
| 2     | Top / Front middle (DB1X models only)      |
| 3     | Rear right                                  |
| 4     | Front right                                 |

## Topics

The node subscribes to LED pattern messages and publishes LED status information as defined in the `duckietown_msgs` package.

## Building and Running

1. Build the package:
   ```bash
   colcon build --packages-select led_driver
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Launch the LED driver:
   ```bash
   ros2 launch led_driver led_driver.launch.py
   ```