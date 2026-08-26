#!/usr/bin/env python3

import asyncio
from threading import Thread

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node as ROS2Node
from sensor_msgs.msg import Range as ROSRange
from std_msgs.msg import Header
from dt_robot_utils import get_robot_name
from dtps import context
from dtps_http import RawData
from duckietown_messages.sensors.range import Range
from duckietown_messages.sensors.range_finder import RangeFinder
from duckietown_messages.utils.exceptions import DataDecodingError

# well outside [min_range, max_range], which is how sensor_msgs/Range says to mark a
# reading the consumer should discard
OUT_OF_RANGE = 999.0

# used until the driver's info message arrives. VL53L1X in long distance mode
DEFAULT_MIN_RANGE = 0.04  # meters
DEFAULT_MAX_RANGE = 3.6  # meters
DEFAULT_FOV = 0.471  # radians, 27 degrees


class ToFNode(ROS2Node):
    def __init__(self):
        self._robot_name = get_robot_name()
        super().__init__('tof_driver_node', namespace=f"/{self._robot_name}")
        # arguments
        self.declare_parameter("sensor_name", "front_center")
        self._sensor_name = self.get_parameter("sensor_name").get_parameter_value().string_value
        # replaced by whatever the driver advertises on its info queue
        self._min_range: float = DEFAULT_MIN_RANGE
        self._max_range: float = DEFAULT_MAX_RANGE
        self._fov: float = DEFAULT_FOV
        # create publisher
        self._pub = self.create_publisher(
            ROSRange,
            "~/range",
            1
        )
        self.get_logger().info(f"Initialized for {self._sensor_name} sensor.")

    async def on_info(self, data: RawData):
        """Pick up the sensor's real geometry from the driver. Without it the messages go
        out with an unset fov and minimum, making the standard validity check meaningless.
        """
        try:
            info: RangeFinder = RangeFinder.from_rawdata(data)
        except DataDecodingError as e:
            self.get_logger().error(f"Failed to decode the sensor info message: {e.message}")
            return
        before = (self._min_range, self._max_range, self._fov)
        if info.minimum is not None:
            self._min_range = float(info.minimum)
        if info.maximum is not None:
            self._max_range = float(info.maximum)
        if info.fov is not None:
            self._fov = float(info.fov)
        if (self._min_range, self._max_range, self._fov) != before:
            self.get_logger().info(
                f"Sensor geometry: fov={self._fov:.3f}rad, "
                f"range=[{self._min_range}, {self._max_range}]m."
            )

    async def publish(self, data: RawData):
        # print(data)
        # decode data
        try:
            tof: Range = Range.from_rawdata(data)
        except DataDecodingError as e:
            self.get_logger().error(f"Failed to decode an incoming message: {e.message}")
            return

        # Log the incoming data
        # self.get_logger().info(f"Received data: {tof.data}, frame: {tof.header.frame}")

        # Ensure the data is of the correct type and within valid ranges
        if tof.data is not None and isinstance(tof.data, (float, int)):
            distance = float(tof.data)
        else:
            # the driver sends nothing when the chip saw no target, which is ordinary
            # for a sensor pointed at open space, so this is not a warning
            self.get_logger().debug("No target in range.")
            distance = OUT_OF_RANGE

        if distance > self._max_range:
            distance = OUT_OF_RANGE

        # create Range message
        tof_msg = ROSRange()
        tof_msg.header = Header()
        # stamp with the driver's read time, not ours: it is the closest we have to when the
        # chip actually measured. This node's own receive time only tells you when the bridge
        # got around to forwarding it, which drifts with scheduling and I2C bus contention
        driver_timestamp = getattr(tof.header, "timestamp", None)
        if driver_timestamp is not None:
            tof_msg.header.stamp = self._stamp_from_epoch(driver_timestamp)
        else:
            tof_msg.header.stamp = self.get_clock().now().to_msg()
        frame_id = getattr(tof.header, "frame", None)
        if isinstance(frame_id, bytes):
            try:
                frame_id = frame_id.decode("utf-8", errors="ignore")
            except Exception:
                frame_id = ""
        elif frame_id is None:
            frame_id = ""
        elif not isinstance(frame_id, str):
            frame_id = str(frame_id)
        tof_msg.header.frame_id = frame_id
        # left unset, radiation_type reads as ULTRASOUND
        tof_msg.radiation_type = ROSRange.INFRARED
        tof_msg.field_of_view = float(self._fov)
        tof_msg.min_range = float(self._min_range)
        tof_msg.max_range = float(self._max_range)
        tof_msg.range = float(distance)  # Ensure distance is float
        tof_msg.variance = 0.0
        # print(tof_msg)
        self._pub.publish(tof_msg)
        # self.get_logger().info(f"Published range message: {tof_msg}")

    @staticmethod
    def _stamp_from_epoch(epoch_seconds: float) -> TimeMsg:
        sec = int(epoch_seconds)
        nanosec = int(round((epoch_seconds - sec) * 1e9))
        # rounding a fraction just under a second carries into the next one, and a
        # nanosec of 1e9 is out of contract even though it fits the field
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        return TimeMsg(sec=sec, nanosec=nanosec)

    async def worker(self):
        try:
            switchboard = (await context("switchboard")).navigate(self._robot_name)
            self.get_logger().info("Connected to switchboard context.")
            info = await (switchboard / "sensor" / "time_of_flight" / self._sensor_name / "info").until_ready()
            await info.subscribe(self.on_info)
            tof = await (switchboard / "sensor" / "time_of_flight" / self._sensor_name / "range").until_ready()
            self.get_logger().info(f"Publishing to topic: sensor/time_of_flight/{self._sensor_name}/range")
            await tof.subscribe(self.publish)
        except Exception as e:
            self.get_logger().error(f"Failed to navigate ToF context: {str(e)}")
            return
        await self.join()

    async def join(self):
        while rclpy.ok():
            await asyncio.sleep(1)

    def spin(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        spin_thread = Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        try:
            asyncio.run(self.worker())
        except RuntimeError:
            if rclpy.ok():
                self.get_logger().error("An error occurred while running the event loop")
                raise
        finally:
            shutdown_ok = executor.shutdown(1)
            if not shutdown_ok:
                self.get_logger().warning("ROS2 executor did not shut down within 1 second.")
            spin_thread.join(timeout=1)
            if spin_thread.is_alive():
                self.get_logger().warning("ROS2 executor thread is still running; skipping node destruction.")
            else:
                self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    tof_node = ToFNode()
    try:
        tof_node.spin()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
