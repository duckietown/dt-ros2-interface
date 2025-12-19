#!/usr/bin/env python3

import asyncio
from typing import Optional
import rclpy
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
            name="camera",
            kind=NodeType.DRIVER,
            description="Reads a stream of images from a camera and publishes the frames over ROS",
        )
        self._robot_name = get_robot_name()
        self._camera_name = camera_name
        self._ros2 = ROS2Node("camera_driver_node")
        self.pub_img = self._ros2.create_publisher(ROS2CompressedImage, "image/compressed", 1)
        self.pub_camera_info = self._ros2.create_publisher(ROS2CameraInfo, "camera_info", 1)
        self._has_published = False
        
        # Store camera info and intrinsics
        self.camera_info: Optional[Camera] = None
        self.camera_intrinsics: Optional[CameraIntrinsicCalibration] = None
        
        self.loginfo("Initialized.")

    async def publish(self, data: RawData):
        try:
            jpeg: CompressedImage = CompressedImage.from_rawdata(data)
        except DataDecodingError as e:
            self.logerr(f"Failed to decode an incoming message: {e.message}")
            return
        # create CompressedImage message
        msg = ROS2CompressedImage()
        msg.header.stamp = self._ros2.get_clock().now().to_msg()
        # msg.header.frame_id = jpeg.header.frame # TODO: restore this
        msg.header.frame_id = "camera_color_optical_frame"
        msg.format = jpeg.format
        msg.data = jpeg.data
        self.pub_img.publish(msg)
        
        # Publish camera info alongside the image
        self.publish_camera_info()
        
        if not self._has_published:
            self.loginfo("Published the first image.")
            self._has_published = True

    def publish_camera_info(self):
        if self.camera_intrinsics is None:
            self.loginfo("No camera intrinsic parameters received yet")
            return

        if self.camera_info is None:
            self.loginfo("No camera information received yet")
            return

        msg = ROS2CameraInfo()
        msg.header.stamp = self._ros2.get_clock().now().to_msg()
        msg.header.frame_id = "camera_color_optical_frame"  # TODO: use self.camera_intrinsics.header.frame
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
            self.camera_intrinsics: CameraIntrinsicCalibration = CameraIntrinsicCalibration.from_rawdata(rdata)
        except DataDecodingError as e:
            self.logerr(f"Failed to decode an incoming message: {e.message}")
            return

    async def worker(self):
        switchboard = (await context("switchboard")).navigate(self._robot_name)
        # TODO: the camera name should be passed in as a CLI argument
        camera: DTPSContext = switchboard / "sensor" / "camera" / self._camera_name
        jpeg: DTPSContext = await (camera / "jpeg").until_ready()
        info: DTPSContext = await (camera / "info").until_ready()
        parameters: DTPSContext = await (camera / "parameters").until_ready()
        
        # Enable dynamic reconnection to the topics
        jpeg = jpeg.configure(ContextConfig(patient=True))
        parameters = parameters.configure(ContextConfig(patient=True))
        info = info.configure(ContextConfig(patient=True))
        
        # Subscribe to camera info and parameters first to get initial data
        await info.subscribe(self.save_camera_info)
        await parameters.subscribe(self.save_camera_intrinsics)
        await jpeg.subscribe(self.publish)
        
        await self.join()

    def spin(self):
        try:
            asyncio.run(self.worker())
        except RuntimeError:
            if not self.is_shutdown:
                self.logerr("An error occurred while running the event loop")
                raise

    async def join(self):
        while not self.is_shutdown:
            # Spin the ROS2 node to handle callbacks and parameter services
            rclpy.spin_once(self._ros2, timeout_sec=0.1)
            await asyncio.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    camera_node.spin()
    # camera_node.worker()
    # rclpy.spin(camera_node)
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
