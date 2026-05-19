#!/usr/bin/env python3
"""ROS 2 driver for the Raspberry Pi CSI camera.

Uses picamera2 (libcamera) to capture frames, JPEG-encodes them with OpenCV,
and publishes `<ns>/image/compressed` (sensor_msgs/CompressedImage) plus
`<ns>/camera_info` (sensor_msgs/CameraInfo) at the requested framerate.

Falls back to /dev/video0 via OpenCV + V4L2 when picamera2 is unavailable or
fails to open the sensor.
"""
import atexit
import os
import threading
import time
from typing import Optional, Union

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage


class RpiCameraDriverNode(Node):
    # clockwise degrees -> numpy rot90 counter-clockwise k
    _ROTATION_K = {0: 0, 90: 3, 180: 2, 270: 1}
    VIDEO_DEVICE = "/dev/video0"

    def __init__(self):
        super().__init__("rpi_camera_driver_node")

        self.declare_parameter("camera_name", "front_center")
        self.declare_parameter("frame_id", "camera_color_optical_frame")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("framerate", 30.0)
        self.declare_parameter("rotation", 0)
        self.declare_parameter("jpeg_quality", 90)
        self.declare_parameter("exposure_mode", "")
        self.declare_parameter("calibration_file", "")

        self.camera_name = self.get_parameter("camera_name").value
        self.frame_id = self.get_parameter("frame_id").value
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.framerate = float(self.get_parameter("framerate").value)
        self.rotation = int(self.get_parameter("rotation").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.exposure_mode = str(self.get_parameter("exposure_mode").value)
        self.calibration_file = str(self.get_parameter("calibration_file").value)

        self.pub_image = self.create_publisher(CompressedImage, "image/compressed", 1)
        self.pub_camera_info = self.create_publisher(CameraInfo, "camera_info", 1)

        self._camera: Optional[Union["Picamera2", cv2.VideoCapture]] = None
        self._use_picamera2 = False
        self._camera_info_msg: Optional[CameraInfo] = None
        self._load_calibration()

        self._stop = threading.Event()
        self._worker: Optional[threading.Thread] = None

        self._open_camera()

        atexit.register(self._release)
        self._worker = threading.Thread(target=self._capture_loop, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f"rpi_camera_driver_node started: {self.width}x{self.height} @ "
            f"{self.framerate:.1f} fps, rotation={self.rotation}, "
            f"backend={'picamera2' if self._use_picamera2 else 'v4l2'}"
        )

    def _load_calibration(self):
        msg = CameraInfo()
        msg.width = self.width
        msg.height = self.height
        msg.distortion_model = "plumb_bob"
        cx, cy = self.width / 2.0, self.height / 2.0
        fx, fy = float(self.width), float(self.height)
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        if self.calibration_file and os.path.isfile(self.calibration_file):
            try:
                with open(self.calibration_file) as f:
                    cal = yaml.safe_load(f)
                msg.width = int(cal.get("image_width", msg.width))
                msg.height = int(cal.get("image_height", msg.height))
                msg.distortion_model = cal.get("distortion_model", msg.distortion_model)
                msg.k = list(cal["camera_matrix"]["data"])
                msg.d = list(cal["distortion_coefficients"]["data"])
                msg.r = list(cal["rectification_matrix"]["data"])
                msg.p = list(cal["projection_matrix"]["data"])
                self.get_logger().info(f"Loaded calibration from {self.calibration_file}")
            except Exception as e:
                self.get_logger().warn(f"Failed to load calibration: {e} — using identity")

        self._camera_info_msg = msg

    def _open_camera(self):
        if not self._try_picamera2():
            self._try_v4l2()

    def _try_picamera2(self) -> bool:
        try:
            from picamera2 import Picamera2
        except ImportError:
            self.get_logger().warn("picamera2 not installed; skipping libcamera path")
            return False
        try:
            cam = Picamera2()
            cfg = cam.create_video_configuration(
                main={"format": "RGB888", "size": (self.width, self.height)},
                controls={"FrameRate": float(self.framerate)},
            )
            cam.configure(cfg)
            cam.options["quality"] = self.jpeg_quality
            if self.exposure_mode == "sports":
                cam.set_controls({"AeExposureMode": 1})
            cam.start()
            frame = cam.capture_array()
            if frame is None:
                cam.stop()
                cam.close()
                self.get_logger().warn("picamera2 opened but first frame was empty")
                return False
            self._camera = cam
            self._use_picamera2 = True
            self.get_logger().info("Camera opened via picamera2 (libcamera)")
            return True
        except Exception as e:
            self.get_logger().warn(f"picamera2 failed: {e}")
            return False

    def _try_v4l2(self):
        cap = cv2.VideoCapture()
        try:
            cap.open(self.VIDEO_DEVICE, cv2.CAP_V4L2)
            if not cap.isOpened():
                raise RuntimeError(f"OpenCV cannot open {self.VIDEO_DEVICE}")
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.framerate)
            cap.set(cv2.CAP_PROP_CONVERT_RGB, 0.0)
            ok, _ = cap.read()
            if not ok:
                raise RuntimeError("Could not read first frame from V4L2 device")
            self._camera = cap
            self._use_picamera2 = False
            self.get_logger().info(f"Camera opened via V4L2 ({self.VIDEO_DEVICE})")
        except Exception as e:
            cap.release()
            raise RuntimeError(f"Could not open camera: {e}")

    def _capture_jpeg(self) -> Optional[bytes]:
        if self._use_picamera2:
            frame = self._camera.capture_array()
            if frame is None:
                return None
            k = self._ROTATION_K.get(self.rotation, 0)
            if k:
                frame = np.rot90(frame, k=k)
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            return buf.tobytes() if ok else None
        ok, image = self._camera.read()
        if not ok or image is None:
            return None
        return image.tobytes()

    def _capture_loop(self):
        period = 1.0 / self.framerate if self.framerate > 0 else 0.0
        next_t = time.monotonic()
        frames = 0
        t0 = time.monotonic()
        while not self._stop.is_set() and rclpy.ok():
            jpeg = self._capture_jpeg()
            if jpeg is None:
                self.get_logger().warn("Empty frame from camera")
                time.sleep(0.1)
                continue
            stamp = self.get_clock().now().to_msg()
            img_msg = CompressedImage()
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = self.frame_id
            img_msg.format = "jpeg"
            img_msg.data = jpeg
            self.pub_image.publish(img_msg)

            info = self._camera_info_msg
            info.header.stamp = stamp
            info.header.frame_id = self.frame_id
            self.pub_camera_info.publish(info)

            frames += 1
            if frames % 60 == 0:
                dt = time.monotonic() - t0
                self.get_logger().info(
                    f"published {frames} frames ({frames / dt:.1f} Hz avg)"
                )

            if period > 0:
                next_t += period
                sleep = next_t - time.monotonic()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    next_t = time.monotonic()

    def _release(self):
        self._stop.set()
        if self._worker is not None and self._worker.is_alive():
            self._worker.join(timeout=2.0)
        if self._camera is not None:
            try:
                if self._use_picamera2:
                    self._camera.stop()
                    self._camera.close()
                else:
                    self._camera.release()
            except Exception:
                pass
            self._camera = None

    def destroy_node(self):
        self._release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RpiCameraDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
