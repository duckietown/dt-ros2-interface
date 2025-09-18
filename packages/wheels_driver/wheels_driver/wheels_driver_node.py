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
        robot_name = get_robot_name()
        super().__init__('wheels_driver_node', namespace=f"/{robot_name}")
        self._robot_name = robot_name
        self._actuator_name = actuator_name
        self._frequency = 30.0
        self._data: Optional[DifferentialPWM] = None
        self._pending_estop: Optional[bool] = None

        # Binding/retry state
        self._pwm_bind_inflight: bool = False
        self._estop_bind_inflight: bool = False
        self._autopilot_bind_inflight: bool = False
        self._pwm_filtered_bind_inflight: bool = False

        # QoS profile for topics
        qos_profile = QoSProfile(depth=10)
        self.pub_wheels_cmd = self.create_publisher(WheelsCmdStamped, 'wheels_cmd_executed', qos_profile)
        self.pub_autopilot = self.create_publisher(Bool, 'autopilot', qos_profile)
        self.create_subscription(WheelsCmdStamped, 'wheels_cmd', self.cmds_cb, qos_profile)
        self.create_subscription(Bool, 'emergency_stop', self.estop_cb, qos_profile)

        # DTPS context variables
        self._pwm: Optional[DTPSContext] = None
        self._estop: Optional[DTPSContext] = None
        self._autopilot: Optional[DTPSContext] = None
        self._loop: Optional[AbstractEventLoop] = None

        self.get_logger().info("Initialized WheelsDriverNode.")

    def estop_cb(self, msg: Bool):
        estop = msg.data
        # Store desired state so we can publish once context is bound
        self._pending_estop = estop
        if self._loop and self._estop is not None:
            raw = Boolean(data=estop).to_rawdata()
            asyncio.run_coroutine_threadsafe(self._estop.publish(raw), self._loop)
            self.get_logger().info("Emergency Stop Activated" if estop else "Emergency Stop Released")
        else:
            self.get_logger().warn("Received emergency_stop but ESTOP context not ready; will publish once bound.")

    def cmds_cb(self, msg: WheelsCmdStamped):
        self.get_logger().info(f"Received wheels command: left={msg.vel_left}, right={msg.vel_right}")
        self._data = DifferentialPWM(left=msg.vel_left, right=msg.vel_right)

    async def publisher(self):
        while rclpy.ok():
            if self._data:
                if self._pwm is None:
                    self.get_logger().warn("PWM context not ready; scheduling bind and retaining last command.")
                    await self._schedule_bind_pwm()
                else:
                    self.get_logger().info(f"Publishing data: left={self._data.left}, right={self._data.right}")
                    try:
                        await self._pwm.publish(self._data.to_rawdata())
                        # Only clear _data if publishing succeeds
                        self._data = None
                    except Exception:
                        self.get_logger().error("Failed to publish the last command. Will rebind PWM and retry later.")
                        traceback.print_exc()
                        # Drop the binding to trigger rebind
                        self._pwm = None
                        await self._schedule_bind_pwm()
            # process ROS callbacks without blocking the loop
            rclpy.spin_once(self, timeout_sec=0.0)
            await asyncio.sleep(1.0 / self._frequency)

    async def publish_autopilot(self, data):
        try:
            self.get_logger().info("Received data to publish on autopilot")
            autopilot = Boolean.from_rawdata(data)
            print(autopilot)
            autopilot_msg = Bool()
            autopilot_msg.data = autopilot.data
            self.pub_autopilot.publish(autopilot_msg)
            self.get_logger().info(f"Publishing autopilot command: autopilot={autopilot_msg.data}")

        except DataDecodingError as e:
            self.get_logger().error(f"Failed to decode an incoming message: {e.message}")

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

            # Start publisher ASAP so commands can flow even if contexts are missing
            self._loop = asyncio.get_event_loop()
            asyncio.create_task(self.publisher())
            self.get_logger().info("Publisher task started successfully.")

            # Bind PWM in the background with retries
            await self._schedule_bind_pwm(switchboard)

            # Setting up emergency stop context (optional but useful)
            try:
                asyncio.create_task(self._bind_estop_until_ready(switchboard))
            except Exception:
                self.get_logger().warn("Emergency stop context not available.")

            # Try to bind autopilot context without blocking actuation
            async def bind_autopilot():
                try:
                    await self._bind_autopilot_until_ready(switchboard)
                except Exception:
                    self.get_logger().warn("Autopilot context not available.")

            # Try to bind pwm_filtered (execution feedback) without blocking
            async def bind_pwm_filtered():
                try:
                    await self._bind_pwm_filtered_until_ready(switchboard)
                except Exception:
                    self.get_logger().warn("Filtered PWM context not available.")

            asyncio.create_task(bind_autopilot())
            asyncio.create_task(bind_pwm_filtered())

            # keep the node alive
            await self.join()

        except Exception as e:
            self.get_logger().error(f"Failed to setup worker: {e}")
            traceback.print_exc()

    async def _schedule_bind_pwm(self, switchboard: Optional[DTPSContext] = None):
        if self._pwm_bind_inflight:
            return
        self._pwm_bind_inflight = True
        async def _bind():
            try:
                if switchboard is None:
                    sb = (await context("switchboard")).navigate(self._robot_name)
                else:
                    sb = switchboard
                retry_sec = 0.5
                while rclpy.ok() and self._pwm is None:
                    try:
                        self.get_logger().info("Binding PWM context...")
                        self._pwm = await (sb / "actuator" / "wheels" / self._actuator_name / "pwm").until_ready()
                        self.get_logger().info("PWM context is ready.")
                        break
                    except Exception:
                        self.get_logger().warn(f"PWM context not ready, retrying in {retry_sec:.1f}s")
                        await asyncio.sleep(retry_sec)
                        retry_sec = min(retry_sec * 2.0, 5.0)
            finally:
                self._pwm_bind_inflight = False
        asyncio.create_task(_bind())

    async def _bind_estop_until_ready(self, switchboard: DTPSContext):
        if self._estop_bind_inflight:
            return
        self._estop_bind_inflight = True
        try:
            retry_sec = 0.5
            while rclpy.ok() and self._estop is None:
                try:
                    self.get_logger().info("Binding ESTOP context...")
                    self._estop = await (switchboard / "actuator" / "wheels" / self._actuator_name / "estop").until_ready()
                    self.get_logger().info("Emergency stop context is ready.")
                    # If there was a pending estop value, publish it now
                    if self._pending_estop is not None and self._loop:
                        raw = Boolean(data=self._pending_estop).to_rawdata()
                        asyncio.run_coroutine_threadsafe(self._estop.publish(raw), self._loop)
                        self.get_logger().info("Applied pending emergency stop state after binding.")
                    break
                except Exception:
                    self.get_logger().warn(f"ESTOP context not ready, retrying in {retry_sec:.1f}s")
                    await asyncio.sleep(retry_sec)
                    retry_sec = min(retry_sec * 2.0, 5.0)
        finally:
            self._estop_bind_inflight = False

    async def _bind_autopilot_until_ready(self, switchboard: DTPSContext):
        if self._autopilot_bind_inflight:
            return
        self._autopilot_bind_inflight = True
        try:
            retry_sec = 0.5
            while rclpy.ok() and self._autopilot is None:
                try:
                    self.get_logger().info("Binding AUTOPILOT context...")
                    self._autopilot = await (switchboard / "actuator" / "wheels" / self._actuator_name / "autopilot").until_ready()
                    self.get_logger().info("Autopilot context is ready.")
                    await self._autopilot.subscribe(self.publish_autopilot)
                    self.get_logger().info("Subscribed to autopilot topic.")
                    break
                except Exception:
                    self.get_logger().warn(f"Autopilot context not ready, retrying in {retry_sec:.1f}s")
                    await asyncio.sleep(retry_sec)
                    retry_sec = min(retry_sec * 2.0, 5.0)
        finally:
            self._autopilot_bind_inflight = False

    async def _bind_pwm_filtered_until_ready(self, switchboard: DTPSContext):
        if self._pwm_filtered_bind_inflight:
            return
        self._pwm_filtered_bind_inflight = True
        try:
            retry_sec = 0.5
            while rclpy.ok():
                try:
                    self.get_logger().info("Binding PWM_FILTERED context...")
                    pwm_filtered = await (switchboard / "actuator" / "wheels" / self._actuator_name / "pwm_filtered").until_ready()
                    self.get_logger().info("Filtered PWM context is ready.")
                    await pwm_filtered.subscribe(self.publish_executed)
                    self.get_logger().info("Subscribed to pwm_filtered for executed commands.")
                    break
                except Exception:
                    self.get_logger().warn(f"Filtered PWM context not ready, retrying in {retry_sec:.1f}s")
                    await asyncio.sleep(retry_sec)
                    retry_sec = min(retry_sec * 2.0, 5.0)
        finally:
            self._pwm_filtered_bind_inflight = False

    async def join(self):
        while rclpy.ok():
            await asyncio.sleep(0.1)

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
