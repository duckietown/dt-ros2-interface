#!/usr/bin/env python3

import asyncio
from asyncio import AbstractEventLoop
from typing import Optional

import rclpy
from rclpy.node import Node as ROS2Node
from std_msgs.msg import ColorRGBA
from duckietown_msgs.msg import LEDPattern

from dt_robot_utils import get_robot_name
from dtps import DTPSContext, context
from dtps_http import RawData
from duckietown_messages.actuators.car_lights import CarLights
from duckietown_messages.colors.rgba import RGBA
from duckietown_messages.standard.header import Header


class LEDDriverNode(ROS2Node):
    """Node for controlling LEDs."""

    def __init__(self):
        super().__init__('leds_driver')
        self._robot_name = get_robot_name()
        self.sub = self.create_subscription(LEDPattern, "led_pattern", self.led_cb, 1)
        self._pattern: Optional[DTPSContext] = None
        self._loop: Optional[AbstractEventLoop] = None
        self.is_initialized = True  # Track whether the node is initialized
        self.get_logger().info(f"Robot name: {self._robot_name}")
        self.get_logger().info("Subscription to 'led_pattern' topic created.")
        self.get_logger().info("LEDDriverNode initialized.")

    def led_cb(self, msg):
        """
        Callback that processes the LED pattern message.

        Args:
            msg (LEDPattern): Message containing the LED pattern
        """
        self.get_logger().info("Received LEDPattern message.")
        if not self.is_initialized:  # Ensure that the node is fully initialized
            self.get_logger().warn("Node not fully initialized. Skipping message.")
            return

        # Make sure enough data is available
        if len(msg.rgb_vals) != 5:
            self.get_logger().error(f"Invalid message. Expected 5 LED values, but got {len(msg.rgb_vals)}")
            return

        # Pack data
        raw: RawData = CarLights(
            header=Header(
                # TODO: reuse the timestamp from the incoming message
            ),
            front_left=self._rgba(msg.rgb_vals[0]),
            front_right=self._rgba(msg.rgb_vals[4]),
            # rear_left=self._rgba(msg.rgb_vals[1]),
            # rear_right=self._rgba(msg.rgb_vals[3]),
            back_left=self._rgba(msg.rgb_vals[1]),
            back_right=self._rgba(msg.rgb_vals[3])  
        ).to_rawdata()

        # Schedule the message for publishing
        try:
            self.get_logger().info("Publishing LED pattern to the queue.")
            asyncio.run_coroutine_threadsafe(self._pattern.publish(raw), self._loop)
        except Exception as e:
            self.get_logger().error(f"Failed to publish LED pattern: {e}")

    @staticmethod
    def _rgba(ros: ColorRGBA) -> RGBA:
        return RGBA(r=ros.r, g=ros.g, b=ros.b, a=ros.a)

    async def worker(self):
        try:
            self.get_logger().info("Starting worker coroutine.")
            # Create switchboard context
            switchboard = (await context("switchboard")).navigate(self._robot_name)
            self.get_logger().info("Switchboard context established.")
            # LEDs pattern queue
            self._pattern = await (switchboard / "actuator" / "lights" / "base" / "pattern").until_ready()
            self.get_logger().info("Pattern publisher initialized.")
            # Set event loop
            self._loop = asyncio.get_event_loop()
            self.is_initialized = True  # Set the flag when the node is initialized
            self.get_logger().info("Event loop initialized. Node is ready to process messages.")
            await self.join()
        except Exception as e:
            self.get_logger().error(f"Worker failed: {e}")

    async def join(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            await asyncio.sleep(1)

    def spin(self):
        try:
            asyncio.run(self.worker())
        except RuntimeError as e:
            if rclpy.ok():
                self.get_logger().error(f"An error occurred while running the event loop: {e}")
                raise

    def on_shutdown(self):
        if self._loop is not None:
            self.get_logger().info("Shutting down the event loop.")
            self._loop.stop()


def main(args=None):
    rclpy.init(args=args)
    node = LEDDriverNode()
    try:
        node.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node due to KeyboardInterrupt.")
    finally:
        rclpy.shutdown()
        node.get_logger().info("ROS2 shutdown complete.")


if __name__ == "__main__":
    main()