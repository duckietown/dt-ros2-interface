import argparse
import glob
import struct
import sys
import threading
import time

from pymavlink import mavutil


MAV_CMD_PREFLIGHT_CALIBRATION = 241
MAV_TYPE_GCS = mavutil.mavlink.MAV_TYPE_GCS
MAV_AUTOPILOT_INVALID = mavutil.mavlink.MAV_AUTOPILOT_INVALID


def _patch_pymavlink_add_message():
    original = mavutil.mavfile.add_message

    def safe_add_message(self, msg):
        try:
            return original(self, msg)
        except TypeError as e:
            if "NoneType" not in str(e):
                raise
            self.messages[msg.get_type()] = msg

    mavutil.mavfile.add_message = safe_add_message


def int_param_from_float(value):
    return struct.unpack("<i", struct.pack("<f", value))[0]


def float_param_from_int(value):
    return struct.unpack("<f", struct.pack("<i", value))[0]


def discover_port():
    candidates = []
    for pattern in ("/dev/ttyACM*", "/dev/serial/by-id/*", "/dev/cu.usbmodem*"):
        candidates.extend(sorted(glob.glob(pattern)))
    return candidates[0] if candidates else None


class GCSHeartbeat:
    def __init__(self, mav):
        self._mav = mav
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _run(self):
        while not self._stop.is_set():
            self._mav.mav.heartbeat_send(MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self._stop.wait(1.0)


def wait_for_calibration(mav, name, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    done_text = f"calibration done: {name}"
    while time.monotonic() < deadline:
        msg = mav.recv_match(type="STATUSTEXT", blocking=True, timeout=0.5)
        if msg is None:
            continue
        if isinstance(msg.text, bytes):
            text = msg.text.decode("utf-8", errors="replace")
        else:
            text = str(msg.text)
        text = text.strip("\x00").strip()
        if not text:
            continue
        print(text, flush=True)
        lower_text = text.lower()
        if done_text in lower_text:
            return True, text
        if text.startswith("[cal]") and "fail" in lower_text:
            return False, text
    return False, f"Timed out waiting for {name} calibration."


def send_preflight_calibration(mav, name):
    if name == "gyro":
        params = (1, 0, 0, 0, 0, 0, 0)
    elif name == "accel":
        params = (0, 0, 0, 0, 1, 0, 0)
    else:
        raise ValueError(name)

    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        MAV_CMD_PREFLIGHT_CALIBRATION,
        0,
        *params,
    )


def main():
    parser = argparse.ArgumentParser(description="Manual PX4 IMU calibration over MAVLink.")
    parser.add_argument("calibration", choices=["gyro", "accel", "both"])
    parser.add_argument(
        "--port",
        default=None,
        help="MAVLink serial/UDP endpoint. Defaults to first /dev/ttyACM*.",
    )
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    _patch_pymavlink_add_message()

    port = args.port or discover_port()
    if port is None:
        print("No PX4 serial port found. Pass --port explicitly.", file=sys.stderr)
        return 2

    mav = mavutil.mavlink_connection(port, baud=args.baud, autoreconnect=True)
    print(f"Connecting to PX4 on {port}...")
    mav.wait_heartbeat()
    print(f"Heartbeat from system={mav.target_system} component={mav.target_component}")

    heartbeat = GCSHeartbeat(mav)
    heartbeat.start()
    try:
        sequence = ["gyro", "accel"] if args.calibration == "both" else [args.calibration]
        for name in sequence:
            if name == "gyro":
                print("Place the drone still on a level surface.")
                timeout = 45
            else:
                print("Rotate through all six orientations. Hold each side still until PX4 accepts it.")
                timeout = 180
            send_preflight_calibration(mav, name)
            ok, message = wait_for_calibration(mav, name, timeout)
            if not ok:
                print(message, file=sys.stderr)
                return 1
        return 0
    finally:
        heartbeat.stop()


if __name__ == "__main__":
    raise SystemExit(main())
