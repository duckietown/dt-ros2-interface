#!/usr/bin/env python3

import asyncio
import rclpy
from rclpy.node import Node as ROS2Node
from sensor_msgs.msg import Range as ROSRange
from std_msgs.msg import Header
from dt_robot_utils import get_robot_name
from dtps import context
from dtps_http import RawData
from duckietown_messages.sensors.range import Range
from duckietown_messages.utils.exceptions import DataDecodingError

MAX_RANGE = 99  # meters
OUT_OF_RANGE = 999

SPIN_TIMEOUT_SEC = 0.0  # Non-blocking ROS2 spin timeout to keep the asyncio loop free for dtps
SPIN_PERIOD_SEC = 0.02  # Poll rate (50 Hz) for ROS2 spin to maintain parameter service and shutdown responsiveness


class ToFNode(ROS2Node):
    def __init__(self):
        self._robot_name = get_robot_name()
        super().__init__('tof_node', namespace=f"/{self._robot_name}")
        # arguments
        self.declare_parameter("sensor_name", "front_center")
        self._sensor_name = self.get_parameter("sensor_name").get_parameter_value().string_value
        # create publisher
        self._pub = self.create_publisher(
            ROSRange,
            "range",
            1
        )
        self.get_logger().info(f"Initialized for {self._sensor_name} sensor.")

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
            self.get_logger().warn(f"Invalid or None data received: {tof.data}")
            distance = OUT_OF_RANGE

        if distance > MAX_RANGE:
            self.get_logger().warn(f"Distance {distance} is out of valid range, setting to max range {OUT_OF_RANGE}")
            distance = OUT_OF_RANGE

        # create Range message
        tof_msg = ROSRange()
        tof_msg.header = Header()
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
        tof_msg.max_range = float(MAX_RANGE)
        tof_msg.range = float(distance)  # Ensure distance is float
        tof_msg.variance = 0.0
        # print(tof_msg)
        self._pub.publish(tof_msg)
        # self.get_logger().info(f"Published range message: {tof_msg}")

    async def worker(self):
        try:
            switchboard = (await context("switchboard")).navigate(self._robot_name)
            self.get_logger().info("Connected to switchboard context.")
            tof = await (switchboard / "sensor" / "time_of_flight" / self._sensor_name / "range").until_ready()
            self.get_logger().info(f"Publishing to topic: sensor/time_of_flight/{self._sensor_name}/range")
            await tof.subscribe(self.publish)
        except Exception as e:
            self.get_logger().error(f"Failed to navigate ToF context: {str(e)}")
            return
        await self.join()

    async def join(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=SPIN_TIMEOUT_SEC)
            await asyncio.sleep(SPIN_PERIOD_SEC)

    def spin(self):
        try:
            asyncio.run(self.worker())
        except RuntimeError:
            if rclpy.ok():
                self.get_logger().error("An error occurred while running the event loop")
                raise


def main(args=None):
    rclpy.init(args=args)
    tof_node = ToFNode()
    tof_node.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
