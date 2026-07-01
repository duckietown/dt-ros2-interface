#!/usr/bin/env python3
"""ROS 2 bridge node for the Duckiematrix state stream."""

import asyncio
from threading import Thread
from typing import Optional

import rclpy
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance, Vector3
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node as ROS2Node

from dt_robot_utils import get_robot_name
from dtps import context
from dtps_http import RawData
from duckietown_messages.geometry_3d.transformation import Transformation
from duckietown_messages.geometry_3d.twist import Twist as DTTwist
from duckietown_messages.utils.exceptions import DataDecodingError


class DuckiematrixInterfaceNode(ROS2Node):
    """Bridge Duckiematrix pose and twist streams into a ROS 2 state topic."""

    def __init__(self) -> None:
        self._robot_name = get_robot_name()
        self._previous_pose: Optional[Pose] = None
        self._state_frame_id = "map"
        self._child_frame_id = "base_link"
        self._missing_pose_warned = False
        super().__init__("duckiematrix_interface_node", namespace=f"/{self._robot_name}")

        self._state_pub = self.create_publisher(Odometry, "~/state", 1)

        self.get_logger().info("Initialized Duckiematrix DTPS bridge.")

    async def _publish_pose(self, data: RawData) -> None:
        try:
            pose = Transformation.from_rawdata(data)
        except DataDecodingError as error:
            self.get_logger().error(
                f"Failed to decode pose message: {error.message}"
            )
            return

        frame_id = pose.source or pose.header.frame or "map"
        self._state_frame_id = self._normalize_frame_id(frame_id) or "map"
        self._previous_pose = Pose(
            position=Point(
                x=float(pose.position.x),
                y=float(pose.position.y),
                z=float(pose.position.z),
            ),
            orientation=Quaternion(
                x=float(pose.rotation.x),
                y=float(pose.rotation.y),
                z=float(pose.rotation.z),
                w=float(pose.rotation.w),
            ),
        )

    async def _publish_twist(self, data: RawData) -> None:
        try:
            twist = DTTwist.from_rawdata(data)
        except DataDecodingError as error:
            self.get_logger().error(
                f"Failed to decode twist message: {error.message}"
            )
            return

        twist_frame = twist.header.frame or "base_link"
        self._child_frame_id = self._normalize_frame_id(twist_frame) or "base_link"

        if self._previous_pose is None:
            if not self._missing_pose_warned:
                self.get_logger().warning(
                    "Skipping state publish until a pose message is available."
                )
                self._missing_pose_warned = True
            return

        self._missing_pose_warned = False

        odometry = Odometry()
        odometry.header.stamp = self.get_clock().now().to_msg()
        odometry.header.frame_id = self._state_frame_id
        odometry.child_frame_id = self._child_frame_id
        odometry.pose = PoseWithCovariance(pose=self._previous_pose)
        odometry.twist = TwistWithCovariance(
            twist=Twist(
                linear=Vector3(
                    x=float(twist.linear_velocity.x),
                    y=float(twist.linear_velocity.y),
                    z=float(twist.linear_velocity.z),
                ),
                angular=Vector3(
                    x=float(twist.angular_velocity.x),
                    y=float(twist.angular_velocity.y),
                    z=float(twist.angular_velocity.z),
                ),
            )
        )
        self._state_pub.publish(odometry)

    async def worker(self) -> None:
        try:
            switchboard = (await context("switchboard")).navigate(self._robot_name)
            pose_queue = await (switchboard / "state" / "pose").until_ready()
            twist_queue = await (switchboard / "state" / "twist").until_ready()

            await pose_queue.subscribe(self._publish_pose)
            await twist_queue.subscribe(self._publish_twist)

            self.get_logger().info(
                "Subscribed to DTPS topics: state/pose, state/twist."
            )
        except Exception as error:
            self.get_logger().error(
                f"Failed to connect Duckiematrix DTPS topics: {str(error)}"
            )
            return

        await self.join()

    async def join(self) -> None:
        while rclpy.ok():
            await asyncio.sleep(1)

    def spin(self) -> None:
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        spin_thread = Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        try:
            asyncio.run(self.worker())
        except RuntimeError:
            if rclpy.ok():
                self.get_logger().error("An error occurred while running the event loop.")
                raise
        finally:
            shutdown_ok = executor.shutdown(1)
            if not shutdown_ok:
                self.get_logger().warning("ROS2 executor did not shut down within 1 second.")
            spin_thread.join(timeout=1)
            if spin_thread.is_alive():
                self.get_logger().warning(
                    "ROS2 executor thread is still running; skipping node destruction."
                )
            else:
                self.destroy_node()

    @staticmethod
    def _normalize_frame_id(frame_id: Optional[str]) -> str:
        if frame_id is None:
            return ""
        if isinstance(frame_id, bytes):
            try:
                return frame_id.decode("utf-8", errors="ignore")
            except Exception:
                return ""
        return str(frame_id)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = DuckiematrixInterfaceNode()
    try:
        node.spin()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
