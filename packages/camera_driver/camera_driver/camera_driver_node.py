#!/usr/bin/env python3

import asyncio
import rclpy
from rclpy.node import Node as ROS2Node
from sensor_msgs.msg import CompressedImage as ROS2CompressedImage, CameraInfo as ROS2CameraInfo
from dt_node_utils import NodeType
from dt_node_utils.node import Node
from dt_robot_utils import get_robot_name
from dtps import context, DTPSContext
from dtps_http import RawData

from duckietown_messages.calibrations.camera_intrinsic import CameraIntrinsicCalibration
from duckietown_messages.sensors.camera import Camera
from duckietown_messages.sensors.compressed_image import CompressedImage
from duckietown_messages.utils.exceptions import DataDecodingError


class CameraNode(Node):
    def __init__(self):
        super(CameraNode, self).__init__(
            name="camera",
            kind=NodeType.DRIVER,
            description="Reads a stream of images from a camera and publishes the frames over ROS",
        )
        self._robot_name = get_robot_name()
        self._ros2 = ROS2Node("camera_driver_node")
        self.pub_img = self._ros2.create_publisher(ROS2CompressedImage, "image/compressed", 1)
        self.pub_camera_info = self._ros2.create_publisher(ROS2CameraInfo, "camera_info", 1)
        self._has_published = False
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
        if not self._has_published:
            self.loginfo("Published the first image.")
            self._has_published = True

    async def publish_camera_info(self, width: int, height: int, K, D, R, P):
        msg = ROS2CameraInfo()
        msg.header.stamp = self._ros2.get_clock().now().to_msg()
        # TODO: update this
        msg.header.frame_id = "camera_color_optical_frame"
        msg.width = width
        msg.height = height
        msg.distortion_model = "plumb_bob"
        msg.d = D
        msg.k = K
        msg.r = R
        msg.p = P
        self.pub_camera_info.publish(msg)

    async def worker(self):
        switchboard = (await context("switchboard")).navigate(self._robot_name)
        # TODO: the camera name should passed in as a CLI argument
        camera: DTPSContext = switchboard / "sensor" / "camera" / "front_center"
        jpeg: DTPSContext = await (camera / "jpeg").until_ready()
        info: DTPSContext = await (camera / "info").until_ready()
        parameters: DTPSContext = await (camera / "parameters").until_ready()
        # get camera sensor info
        try:
            info_data: Camera = Camera.from_rawdata(await info.data_get())
        except DataDecodingError as e:
            self.logerr(f"Failed to decode an incoming message: {e.message}")
            return
        # get camera intrinsics calibration
        try:
            parameters_data: CameraIntrinsicCalibration = CameraIntrinsicCalibration.from_rawdata(
                await parameters.data_get())
        except DataDecodingError as e:
            self.logerr(f"Failed to decode an incoming message: {e.message}")
            return
        await self.publish_camera_info(
            info_data.width, info_data.height,
            parameters_data.K, parameters_data.D, parameters_data.R, parameters_data.P
        )
        await jpeg.subscribe(self.publish)
        await self.join()

    async def join(self):
        while not self.is_shutdown:
            await asyncio.sleep(1)


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    camera_node.spin()
    # camera_node.worker()
    # rclpy.spin(camera_node)
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
