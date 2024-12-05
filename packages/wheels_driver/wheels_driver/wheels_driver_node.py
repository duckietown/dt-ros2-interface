#!/usr/bin/env python3

import asyncio
import traceback
from asyncio import AbstractEventLoop
from typing import Optional

import rclpy
from rclpy.node import Node as ROS2Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool
from duckietown_msgs.msg import WheelsCmdStamped

from dt_robot_utils import get_robot_name
from dtps import context, DTPSContext
from duckietown_messages.actuators.differential_pwm import DifferentialPWM
from duckietown_messages.standard.boolean import Boolean
from duckietown_messages.utils.exceptions import DataDecodingError


class WheelsDriverNode(ROS2Node):
    def __init__(self, actuator_name: str = "base"):
        super().__init__('wheels_driver')
        self._robot_name = get_robot_name()
        self._actuator_name = actuator_name
        self._frequency = 30.0
        self._data: Optional[DifferentialPWM] = None

        # QoS profile for topics
        qos_profile = QoSProfile(depth=10)
        self.pub_wheels_cmd = self.create_publisher(WheelsCmdStamped, '~/wheels_cmd_executed', qos_profile)
        self.create_subscription(WheelsCmdStamped, '~/wheels_cmd', self.cmds_cb, qos_profile)
        self.create_subscription(Bool, '~/emergency_stop', self.estop_cb, qos_profile)

        # DTPS context variables
        self._pwm: Optional[DTPSContext] = None
        self._estop: Optional[DTPSContext] = None
        self._loop: Optional[AbstractEventLoop] = None

        self.get_logger().info("Initialized WheelsDriverNode.")

    def estop_cb(self, msg: Bool):
        if self._loop:
            estop = msg.data
            raw = Boolean(data=estop).to_rawdata()
            asyncio.run_coroutine_threadsafe(self._estop.publish(raw), self._loop)
            self.get_logger().info("Emergency Stop Activated" if estop else "Emergency Stop Released")

    def cmds_cb(self, msg: WheelsCmdStamped):
        self.get_logger().info(f"Received wheels command: left={msg.vel_left}, right={msg.vel_right}")
        self._data = DifferentialPWM(left=msg.vel_left, right=msg.vel_right)

    async def publisher(self):
        while rclpy.ok():
            if self._data:
                self.get_logger().info(f"Publishing data: left={self._data.left}, right={self._data.right}")
                try:
                    await self._pwm.publish(self._data.to_rawdata())
                    # Only clear _data if publishing succeeds
                    self._data = None
                except Exception:
                    self.get_logger().error("Failed to publish the last command.")
                    traceback.print_exc()
            rclpy.spin_once(self)
            await asyncio.sleep(1.0 / self._frequency)

    async def publish_executed(self, data):
        try:
            self.get_logger().info("Received data to publish on wheels_cmd_executed")
            executed = DifferentialPWM.from_rawdata(data)
            print(executed)
            executed_msg = WheelsCmdStamped()
            executed_msg.header.stamp = self.get_clock().now().to_msg()
            executed_msg.vel_left = executed.left
            executed_msg.vel_right = executed.right
            self.pub_wheels_cmd.publish(executed_msg)
            self.get_logger().info(f"Publishing executed command: left={executed_msg.vel_left}, right={executed_msg.vel_right}")

        except DataDecodingError as e:
            self.get_logger().error(f"Failed to decode an incoming message: {e.message}")

    async def worker(self):
        self.get_logger().info("Starting worker setup...")
        try:
            switchboard = (await context("switchboard")).navigate(self._robot_name)
            self.get_logger().info(f"Switchboard context navigated for robot: {self._robot_name}")

            # Setting up PWM context
            self._pwm = await (switchboard / "actuator" / "wheels" / self._actuator_name / "pwm").until_ready()
            self.get_logger().info("PWM context is ready.")

            # Setting up emergency stop context
            self._estop = await (switchboard / "actuator" / "wheels" / self._actuator_name / "estop").until_ready()
            self.get_logger().info("Emergency stop context is ready.")

            # Setting up filtered PWM context
            pwm_filtered = await (
                        switchboard / "actuator" / "wheels" / self._actuator_name / "pwm_filtered").until_ready()
            self.get_logger().info("Filtered PWM context is ready.")

            # Subscribe to the pwm_filtered topic
            await pwm_filtered.subscribe(self.publish_executed)
            self.get_logger().info("Subscribed to pwm_filtered topic for executed commands.")

            # Start the publisher task
            await asyncio.create_task(self.publisher())
            self.get_logger().info("Publisher task started successfully.")

            self._loop = asyncio.get_event_loop()
            self.get_logger().info("Event loop obtained successfully.")

        except Exception as e:
            self.get_logger().error(f"Failed to setup worker: {e}")
            traceback.print_exc()

    def spin(self):
        try:
            asyncio.run(self.worker())
        except RuntimeError:
            self.get_logger().error("An error occurred while running the event loop.")


def main(args=None):
    rclpy.init(args=args)
    node = WheelsDriverNode()
    node.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
