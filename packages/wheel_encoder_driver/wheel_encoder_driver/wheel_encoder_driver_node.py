import asyncio
from math import pi

import rclpy
from rclpy.node import Node as ROS2Node
import tf_transformations
from duckietown_msgs.msg import WheelEncoderStamped
from geometry_msgs.msg import TransformStamped, Transform, Quaternion
from tf2_ros import TransformBroadcaster

from dt_robot_utils import get_robot_name
from dtps import context, ContextConfig
from dtps_http import RawData
from duckietown_messages.standard.integer import Integer
from duckietown_messages.utils.exceptions import DataDecodingError

RESOLUTION: int = 135

SPIN_TIMEOUT_SEC = 0.0  # Non-blocking ROS2 spin timeout to keep the asyncio loop free for dtps
SPIN_PERIOD_SEC = 0.005  # Poll rate for ROS2 spin to maintain parameter service and shutdown responsiveness


class WheelEncoderNode(ROS2Node):

    def __init__(self):
        self._robot_name = get_robot_name()
        super().__init__('wheel_encoder_node')
        # get parameters
        self.declare_parameter("wheel", "left")  # Default value
        self._wheel = self.get_parameter("wheel").get_parameter_value().string_value
        # publishers
        self.pub_ticks = self.create_publisher(WheelEncoderStamped, "tick", 1)
        # tf broadcaster for wheel frame
        self.tf_broadcaster = TransformBroadcaster(self)
        # ---
        self.get_logger().info(f"Initialized for {self._wheel} wheel.")

    async def publish(self, data: RawData):
        # decode data
        try:
            ticks: Integer = Integer.from_rawdata(data)
        except DataDecodingError as e:
            self.get_logger().error(f"Failed to decode an incoming message: {e.message}")
            return
        # create encoder ticks message
        ticks_msg: WheelEncoderStamped = WheelEncoderStamped()
        ticks_msg.header.stamp = self.get_clock().now().to_msg()
        ticks_msg.header.frame_id = f"{self._robot_name}/{self._wheel}_wheel_axis"
        ticks_msg.data = ticks.data
        ticks_msg.resolution = RESOLUTION
        ticks_msg.type = WheelEncoderStamped.ENCODER_TYPE_INCREMENTAL
        # publish messages
        self.pub_ticks.publish(ticks_msg)
        # publish TF
        angle = (float(ticks.data) / float(RESOLUTION)) * 2 * pi
        quat = tf_transformations.quaternion_from_euler(0, angle, 0)
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = ticks_msg.header.frame_id
        transform_stamped.child_frame_id = f"{self._robot_name}/{self._wheel}_wheel"
        transform_stamped.transform = Transform(
            rotation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        )
        self.tf_broadcaster.sendTransform(transform_stamped)

    async def worker(self):
        try:
            switchboard = (await context("switchboard")).navigate(self._robot_name)
            encoder = await (switchboard / "sensor" / "wheel_encoder" / self._wheel / "ticks").until_ready()
            # Enable resilient reconnection to the DTPS topic
            encoder = encoder.configure(ContextConfig(patient=True))
            await encoder.subscribe(self.publish)
        except Exception as e:
            self.get_logger().error(f"Failed to navigate wheel encoder context: {str(e)}")
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
    wheel_encoder_node = WheelEncoderNode()
    wheel_encoder_node.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
