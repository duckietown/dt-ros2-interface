#!/usr/bin/env python3

import asyncio
import os
from threading import Thread
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node as ROS2Node
from sensor_msgs.msg import CompressedImage as ROS2CompressedImage, CameraInfo as ROS2CameraInfo
from dt_node_utils import NodeType
from dt_node_utils.node import Node
from dt_robot_utils import get_robot_name
from dtps import context, DTPSContext, ContextConfig
from dtps_http import RawData
from duckietown_messages.calibrations.camera_intrinsic import CameraIntrinsicCalibration
from duckietown_messages.sensors.camera import Camera
from duckietown_messages.sensors.compressed_image import CompressedImage
from duckietown_messages.utils.exceptions import DataDecodingError


class CameraNode(Node):
    def __init__(self, camera_name: str = "front_center"):
        super(CameraNode, self).__init__(
            name="camera_node",
            kind=NodeType.DRIVER,
            description="Reads a stream of images from a camera and publishes the frames over ROS",
        )
        self._robot_name = get_robot_name()
        self._camera_name = camera_name
        self._ros2 = ROS2Node("camera_node")
        self.pub_img = self._ros2.create_publisher(ROS2CompressedImage, "~/image/compressed", 1)
        self.pub_camera_info = self._ros2.create_publisher(ROS2CameraInfo, "~/camera_info", 1)
        self._has_published = False
        self._camera_shm_path = os.environ.get("DT_CAMERA_SHM_IN_PATH", "").strip()
        self._shm_topic_paths = {
            "jpeg": self._camera_shm_path,
            "info": self._topic_shm_path(self._camera_shm_path, ".info"),
            "parameters": self._topic_shm_path(
                self._camera_shm_path,
                ".parameters",
            ),
        }
        topic_shm_only_variables = {
            "jpeg": "DT_CAMERA_SHM_ONLY_JPEG",
            "info": "DT_CAMERA_SHM_ONLY_INFO",
            "parameters": "DT_CAMERA_SHM_ONLY_PARAMETERS",
        }
        self._shm_topic_only = {}
        for topic_name, shm_path in self._shm_topic_paths.items():
            shm_only_variable = topic_shm_only_variables[topic_name]
            shm_only_requested = self._read_boolean_environment(
                shm_only_variable,
                False,
            )
            shm_is_enabled = shm_path != ""
            if shm_only_requested and not shm_is_enabled:
                self.logwarn(
                    f"Ignoring {shm_only_variable}=1 because "
                    "DT_CAMERA_SHM_IN_PATH is not configured."
                )
            self._shm_topic_only[topic_name] = shm_is_enabled and shm_only_requested
        self._topic_handlers = {
            "jpeg": self.publish,
            "info": self.save_camera_info,
            "parameters": self.save_camera_intrinsics,
        }
        
        # Store camera info and intrinsics
        self.camera_info: Optional[Camera] = None
        self.camera_intrinsics: Optional[CameraIntrinsicCalibration] = None
        
        self.loginfo("Initialized.")

    def _read_boolean_environment(self, variable_name: str, default: bool) -> bool:
        """Read a ``0`` or ``1`` transport option and warn for invalid values."""
        default_value = "1" if default else "0"
        variable_value = os.environ.get(variable_name, default_value)
        variable_value = variable_value.strip()
        if variable_value not in ("0", "1"):
            self.logwarn(
                f"{variable_name} must be '0' or '1'; using '{default_value}'."
            )
            return default
        return variable_value == "1"

    @staticmethod
    def _topic_shm_path(base_path: str, suffix: str) -> str:
        """Derive a topic channel path from the compatible JPEG base path."""
        if not base_path:
            return ""
        return base_path + suffix

    def _source_timestamp_to_ros_stamp(self, timestamp: Optional[float]):
        """Preserve the source camera timestamp in an outgoing ROS header."""
        if timestamp is None:
            return self._ros2.get_clock().now().to_msg()
        try:
            timestamp_seconds = float(timestamp)
        except (TypeError, ValueError):
            self.logwarn("Camera image has an invalid source timestamp; using ROS publish time.")
            return self._ros2.get_clock().now().to_msg()
        seconds = int(timestamp_seconds)
        nanoseconds = int(round((timestamp_seconds - seconds) * 1_000_000_000))
        if nanoseconds >= 1_000_000_000:
            seconds += 1
            nanoseconds -= 1_000_000_000
        stamp = self._ros2.get_clock().now().to_msg()
        stamp.sec = seconds
        stamp.nanosec = nanoseconds
        return stamp

    async def publish(self, data: RawData):
        try:
            jpeg: CompressedImage = CompressedImage.from_rawdata(data)
        except DataDecodingError as e:
            self.logerr(f"Failed to decode an incoming message: {e.message}")
            return
        # create CompressedImage message
        msg = ROS2CompressedImage()
        msg.header.stamp = self._source_timestamp_to_ros_stamp(
            jpeg.header.timestamp,
        )
        # msg.header.frame_id = jpeg.header.frame # TODO: restore this
        msg.header.frame_id = "camera_color_optical_frame"
        msg.format = jpeg.format
        msg.data = jpeg.data
        self.pub_img.publish(msg)
        
        # Publish camera info alongside the image
        self.publish_camera_info(msg)
        
        if not self._has_published:
            self.loginfo("Published the first image.")
            self._has_published = True

    def publish_camera_info(self, image_msg: ROS2CompressedImage):
        if self.camera_intrinsics is None:
            self.loginfo("No camera intrinsic parameters received yet")
            return

        if self.camera_info is None:
            self.loginfo("No camera information received yet")
            return

        msg = ROS2CameraInfo()
        msg.header.stamp = image_msg.header.stamp
        msg.header.frame_id = image_msg.header.frame_id
        msg.width = self.camera_info.width
        msg.height = self.camera_info.height
        msg.distortion_model = "plumb_bob"
        msg.d = self.camera_intrinsics.D
        msg.k = self.camera_intrinsics.K
        msg.r = self.camera_intrinsics.R
        msg.p = self.camera_intrinsics.P
        self.pub_camera_info.publish(msg)

    async def save_camera_info(self, rdata: RawData):
        """
        Get the camera specification and save it to a variable.
        """
        try:
            camera: Camera = Camera.from_rawdata(rdata)
        except DataDecodingError as e:
            self.logerr(f"Failed to decode an incoming message: {e.message}")
            self.logwarn("Camera information not available yet.")
            return
        
        if self.camera_info is None:
            self.loginfo("Received camera information.")

        self.camera_info = camera

    async def save_camera_intrinsics(self, rdata: RawData):
        try:
            self.camera_intrinsics = CameraIntrinsicCalibration.from_rawdata(rdata)
        except DataDecodingError as e:
            self.logerr(f"Failed to decode an incoming message: {e.message}")
            return

    async def worker(self):
        switchboard = (await context("switchboard")).navigate(self._robot_name)
        # TODO: the camera name should be passed in as a CLI argument
        camera: DTPSContext = switchboard / "sensor" / "camera" / self._camera_name
        subscriptions = []
        try:
            for topic_name, shm_path in self._shm_topic_paths.items():
                shm_only = self._shm_topic_only[topic_name]
                topic_context = camera / topic_name
                if shm_only:
                    self.loginfo(
                        f"Using camera {topic_name} SHM input at '{shm_path}'."
                    )
                else:
                    topic_context = await topic_context.until_ready()
                    topic_context = topic_context.configure(
                        ContextConfig(patient=True)
                    )
                subscription = await topic_context.subscribe(
                    self._topic_handlers[topic_name],
                    shm_path=shm_path or None,
                    shm_only=shm_only,
                )
                subscriptions.append(subscription)
            await self.join()
        finally:
            for subscription in subscriptions:
                await subscription.unsubscribe()
        
    def spin(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self._ros2)
        spin_thread = Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        try:
            asyncio.run(self.worker())
        except RuntimeError:
            if not self.is_shutdown:
                self.logerr("An error occurred while running the event loop")
                raise
        finally:
            shutdown_ok = executor.shutdown(1)
            if not shutdown_ok:
                self.logwarn("ROS2 executor did not shut down within 1 second.")
            spin_thread.join(timeout=1)
            if spin_thread.is_alive():
                self.logwarn("ROS2 executor thread is still running; skipping node destruction.")
            else:
                self._ros2.destroy_node()

    async def join(self):
        while rclpy.ok():
            await asyncio.sleep(1)


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    try:
        camera_node.spin()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
