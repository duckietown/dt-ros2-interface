import threading
import time
from typing import Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from mavros_msgs.msg import StatusText
from mavros_msgs.srv import CommandLong


MAV_CMD_PREFLIGHT_CALIBRATION = 241


class PX4CalibrationNode(Node):
    def __init__(self):
        super().__init__("px4_calibration")

        self.declare_parameter("command_service", "/mavros/cmd/command")
        self.declare_parameter("status_text_topic", "/mavros/statustext/recv")
        self.declare_parameter("gyro_timeout_sec", 45.0)
        # PX4 >=1.15 reports calibration state over MAVLink events, which MAVROS does
        # not expose. Gyro retains the established accepted-command fallback.
        self.declare_parameter("gyro_statustext_grace_sec", 5.0)
        self.declare_parameter("gyro_assume_success_without_statustext", True)
        # Level-horizon calibration can retry for much longer after motion. Do not
        # report an unobservable completion unless an integrator explicitly opts in.
        self.declare_parameter("level_timeout_sec", 45.0)
        self.declare_parameter("level_statustext_grace_sec", 5.0)
        self.declare_parameter("level_assume_success_without_statustext", False)

        self._callback_group = ReentrantCallbackGroup()
        self._command_client = self.create_client(
            CommandLong,
            self.get_parameter("command_service").value,
            callback_group=self._callback_group,
        )
        self._status_pub = self.create_publisher(String, "/px4_calibration/status", 10)
        self._last_status_text: Optional[str] = None
        self._status_lock = threading.Lock()
        self._calibration_lock = threading.Lock()
        self._calibration_busy_until = 0.0

        self.create_subscription(
            StatusText,
            self.get_parameter("status_text_topic").value,
            self._status_text_cb,
            10,
            callback_group=self._callback_group,
        )
        self.create_service(
            Trigger,
            "/px4_calibration/calibrate_gyro",
            self._calibrate_gyro_cb,
            callback_group=self._callback_group,
        )
        self.create_service(
            Trigger,
            "/px4_calibration/calibrate_level",
            self._calibrate_level_cb,
            callback_group=self._callback_group,
        )

        self.get_logger().info("PX4 calibration services ready.")

    def _status_text_cb(self, msg: StatusText) -> None:
        text = msg.text.strip()
        if not text:
            return
        with self._status_lock:
            self._last_status_text = text
        if text.startswith("[cal]") or "calibration" in text.lower():
            self._publish_status(text)

    def _publish_status(self, text: str) -> None:
        self._status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def _calibrate_gyro_cb(self, _request, response):
        return self._run_calibration(
            response=response,
            name="gyro",
            timeout_sec=float(self.get_parameter("gyro_timeout_sec").value),
            param1=1.0,
            param5=0.0,
            statustext_grace_sec=float(
                self.get_parameter("gyro_statustext_grace_sec").value
            ),
            assume_success_without_statustext=bool(
                self.get_parameter("gyro_assume_success_without_statustext").value
            ),
        )

    def _calibrate_level_cb(self, _request, response):
        return self._run_calibration(
            response=response,
            name="level",
            timeout_sec=float(self.get_parameter("level_timeout_sec").value),
            param1=0.0,
            param5=2.0,
            statustext_grace_sec=float(
                self.get_parameter("level_statustext_grace_sec").value
            ),
            assume_success_without_statustext=bool(
                self.get_parameter("level_assume_success_without_statustext").value
            ),
        )

    def _run_calibration(
        self,
        response,
        name: str,
        timeout_sec: float,
        param1: float,
        param5: float,
        statustext_grace_sec: Optional[float] = None,
        assume_success_without_statustext: bool = False,
    ):
        if not self._calibration_lock.acquire(blocking=False):
            response.success = False
            response.message = "Another PX4 calibration is already running."
            return response

        try:
            now = time.monotonic()
            if now < self._calibration_busy_until:
                remaining_sec = self._calibration_busy_until - now
                response.success = False
                response.message = (
                    "Another PX4 calibration may still be running; retry in "
                    f"up to {remaining_sec:.0f} seconds."
                )
                return response
            self._calibration_busy_until = 0.0

            if not self._command_client.wait_for_service(timeout_sec=5.0):
                response.success = False
                response.message = "MAVROS command service is not available."
                return response

            with self._status_lock:
                self._last_status_text = None

            self._publish_status(f"Starting PX4 {name} calibration.")
            command_response = self._send_preflight_calibration(
                param1=param1, param5=param5
            )
            if command_response is None:
                response.success = False
                response.message = "No response from MAVROS command service."
                return response
            if not command_response.success:
                response.success = False
                response.message = (
                    f"PX4 rejected {name} calibration command: "
                    f"{command_response.result}"
                )
                self._publish_status(response.message)
                return response

            # Prefer a precise "[cal] done/fail" STATUSTEXT. An accepted-command
            # fallback is opt-in because PX4's completion events are not observable
            # through MAVROS. Reserve the local slot through the full timeout when
            # that fallback is used, since PX4 may still be retrying after the reply.
            start = time.monotonic()
            deadline = start + timeout_sec
            grace_deadline = (
                start + statustext_grace_sec if statustext_grace_sec is not None else None
            )
            done_text = f"calibration done: {name}"
            saw_cal_text = False
            while time.monotonic() < deadline and rclpy.ok():
                with self._status_lock:
                    status = self._last_status_text or ""
                lower_status = status.lower()
                if status.startswith("[cal]"):
                    saw_cal_text = True
                if done_text in lower_status:
                    response.success = True
                    response.message = status
                    self._publish_status(f"PX4 {name} calibration complete.")
                    return response
                if status.startswith("[cal]") and "fail" in lower_status:
                    response.success = False
                    response.message = status
                    self._publish_status(status)
                    return response
                if (
                    grace_deadline is not None
                    and assume_success_without_statustext
                    and not saw_cal_text
                    and time.monotonic() > grace_deadline
                ):
                    self._calibration_busy_until = deadline
                    response.success = True
                    response.message = (
                        f"PX4 {name} calibration command accepted, but completion could not "
                        "be observed through MAVROS. Assuming success as configured; further "
                        "calibration requests are held until the configured timeout expires."
                    )
                    self._publish_status(response.message)
                    return response
                time.sleep(0.1)

            response.success = False
            response.message = f"Timed out waiting for PX4 {name} calibration to finish."
            self._publish_status(response.message)
            return response
        finally:
            self._calibration_lock.release()

    def _send_preflight_calibration(self, param1: float, param5: float):
        request = CommandLong.Request()
        request.broadcast = False
        request.command = MAV_CMD_PREFLIGHT_CALIBRATION
        request.confirmation = 0
        request.param1 = param1
        request.param2 = 0.0
        request.param3 = 0.0
        request.param4 = 0.0
        request.param5 = param5
        request.param6 = 0.0
        request.param7 = 0.0

        future = self._command_client.call_async(request)
        start = time.monotonic()
        while rclpy.ok() and not future.done() and time.monotonic() - start < 10.0:
            time.sleep(0.05)
        return future.result() if future.done() else None


def main(args=None):
    rclpy.init(args=args)
    node = PX4CalibrationNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
