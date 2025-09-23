#!/usr/bin/env python3
"""The Control Mapper node (ROS2).

This is a ROS2 port of the original ROS1 Control Mapper node.
It listens for DTPS messages toggling autopilot state and publishes
the corresponding `duckietown_msgs/BoolStamped` on the Duckietown
`joy_mapper_node/joystick_override` topic.
"""

import asyncio
from typing import Optional

import rclpy
from rclpy.node import Node as ROS2Node
from rclpy.qos import QoSProfile

from dt_robot_utils import get_robot_name
from dtps_http import RawData
from duckietown_messages.standard.boolean import Boolean
from duckietown_messages.utils.exceptions import DataDecodingError

from duckietown_msgs.msg import BoolStamped


class ControlMapperNode(ROS2Node):
    """Control Mapper node.

    ROS2 port notes:
    - Replaces `rospy` with `rclpy` primitives.
    - Uses `create_publisher` and `get_clock().now().to_msg()` for stamped messages.
    - Preserves the original DTPS-based subscription flow using `asyncio`.
    """

    _robot_name: str
    _frequency: float

    def __init__(self) -> None:
        """Initialize Control Mapper node."""
        super().__init__("control_mapper_node")
        self._robot_name = get_robot_name()
        self._frequency = 10.0

        qos_profile = QoSProfile(depth=10)
        topic = f"/{self._robot_name}/joy_mapper_node/joystick_override"
        self._joystick_override_publisher = self.create_publisher(BoolStamped, topic, qos_profile)

        self.get_logger().info("Initialized.")

    async def _join(self) -> None:
        """Join.

        Keeps the coroutine alive while ROS is running.
        """
        while rclpy.ok():
            # let ROS callbacks run without blocking asyncio
            rclpy.spin_once(self, timeout_sec=0.0)
            await asyncio.sleep(1.0 / self._frequency)

    async def _on_autopilot(self, raw_data: RawData) -> None:
        """Handle incoming DTPS autopilot messages and publish override.

        In ROS2, we construct `BoolStamped` and set the header stamp via
        `self.get_clock().now().to_msg()`.
        """
        try:
            autopilot: Boolean = Boolean.from_rawdata(raw_data)
        except DataDecodingError as error:
            self.get_logger().error(
                f"Failed to decode an incoming message: {error.message}"
            )
            return
        msg = BoolStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # joystick override is the inverse of autopilot
        msg.data = not autopilot.data
        self._joystick_override_publisher.publish(msg)

    def spin(self) -> None:
        """Spin.

        Runs the asyncio worker until shutdown.
        """
        try:
            asyncio.run(self.worker())
        except RuntimeError:
            if rclpy.ok():
                self.get_logger().error("An error occurred while running the event loop.")
                raise

    async def worker(self) -> None:
        """Worker.

        Binds to the DTPS "switchboard" context and subscribes to the
        `/<robot>/node/control_mapper/autopilot` queue.
        """
        # Import dtps lazily to avoid import costs during package discovery
        import dtps

        self.get_logger().info("Retrieving switchboard context...")
        switchboard = await dtps.context("switchboard")
        self.get_logger().info("Switchboard context retrieved.")
        self.get_logger().info("Waiting for DTPS queues to come online...")
        autopilot_queue = await (
            switchboard
            / self._robot_name
            / "node"
            / "control_mapper"
            / "autopilot"
        ).until_ready()
        self.get_logger().info("DTPS queues have come online.")
        self.get_logger().info("Subscribing to DTPS queues...")
        await autopilot_queue.subscribe(self._on_autopilot)
        self.get_logger().info("Subscribed to DTPS queues.")
        self.get_logger().info("Running...")
        await self._join()
        self.get_logger().info("Shutting down...")


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ControlMapperNode()
    node.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

