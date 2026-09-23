#!/usr/bin/env python3

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import re
import shlex
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from threading import Thread
from typing import Any, Optional


def _bootstrap_workspace_paths() -> None:
    script_path = Path(__file__).resolve()
    parents = script_path.parents
    if len(parents) > 2:
        workspace_root = parents[2]
    else:
        workspace_root = script_path.parent
    python_version = f"python{sys.version_info.major}.{sys.version_info.minor}"
    ros_distro = os.environ.get("ROS_DISTRO", "jazzy")
    candidates = [
        Path("/opt/ros") / ros_distro / "lib" / python_version / "site-packages",
        workspace_root / "lib-dtps-http" / "src",
        workspace_root / "dt-commons" / "packages",
        workspace_root / "duckietown-messages" / "src",
    ]
    for candidate in candidates:
        candidate_str = str(candidate)
        if candidate.exists() and candidate_str not in sys.path:
            sys.path.insert(0, candidate_str)


_bootstrap_workspace_paths()


_CAPTURE_IMPORTS: Optional[dict[str, Any]] = None
_DOCKER_SDK_MODULE: Any = None
_DOCKER_SDK_UNAVAILABLE = False
_ROS2_CAPTURE_IMPORTS: Optional[dict[str, Any]] = None


def _load_capture_imports() -> dict[str, Any]:
    global _CAPTURE_IMPORTS
    if _CAPTURE_IMPORTS is not None:
        return _CAPTURE_IMPORTS
    try:
        from dtps import ContextConfig, context
        from dtps_http import RawData
        from duckietown_messages.sensors.compressed_image import CompressedImage
        from duckietown_messages.utils.exceptions import DataDecodingError
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Capture mode requires the DTPS and Duckietown message runtime dependencies. "
            "Run this script from the target runtime image or install the missing Python packages."
        ) from exc

    _CAPTURE_IMPORTS = {
        "CompressedImage": CompressedImage,
        "ContextConfig": ContextConfig,
        "DataDecodingError": DataDecodingError,
        "RawData": RawData,
        "context": context,
    }
    return _CAPTURE_IMPORTS


def _load_shm_reader_class() -> Any:
    try:
        from dtps_http.shm import ShmReader
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "SHM capture requires dtps_http.shm in the runtime environment."
        ) from exc
    return ShmReader


def _load_docker_sdk() -> Any:
    global _DOCKER_SDK_MODULE
    global _DOCKER_SDK_UNAVAILABLE
    if _DOCKER_SDK_MODULE is not None:
        return _DOCKER_SDK_MODULE
    if _DOCKER_SDK_UNAVAILABLE:
        raise RuntimeError("Docker SDK is not available.")
    try:
        import docker
    except ModuleNotFoundError as exc:
        _DOCKER_SDK_UNAVAILABLE = True
        raise RuntimeError(
            "Docker CLI is unavailable and the Python Docker SDK is not installed."
        ) from exc
    _DOCKER_SDK_MODULE = docker
    return _DOCKER_SDK_MODULE


def _load_ros2_capture_imports() -> dict[str, Any]:
    global _ROS2_CAPTURE_IMPORTS
    if _ROS2_CAPTURE_IMPORTS is not None:
        return _ROS2_CAPTURE_IMPORTS
    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node as ROS2Node
        from sensor_msgs.msg import CompressedImage as ROS2CompressedImage
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "ROS2 output capture requires rclpy and sensor_msgs in the runtime environment."
        ) from exc

    _ROS2_CAPTURE_IMPORTS = {
        "ROS2CompressedImage": ROS2CompressedImage,
        "ROS2Node": ROS2Node,
        "SingleThreadedExecutor": SingleThreadedExecutor,
        "rclpy": rclpy,
    }
    return _ROS2_CAPTURE_IMPORTS


def _ensure_ros_runtime_environment() -> None:
    if os.environ.get("DT_CAMERA_BENCHMARK_ROS_ENV_READY") == "1":
        return
    if os.environ.get("AMENT_PREFIX_PATH"):
        return

    ros_distro = os.environ.get("ROS_DISTRO", "jazzy")
    setup_script = Path("/opt/ros") / ros_distro / "setup.bash"
    if not setup_script.is_file():
        return

    command_parts = [
        sys.executable,
        str(Path(sys.argv[0]).resolve()),
        *sys.argv[1:],
    ]
    command_text = " ".join(shlex.quote(part) for part in command_parts)
    shell_command = f"source {shlex.quote(str(setup_script))} && exec {command_text}"
    env = dict(os.environ)
    env["DT_CAMERA_BENCHMARK_ROS_ENV_READY"] = "1"
    os.execvpe("bash", ["bash", "-lc", shell_command], env)


_SIZE_RE = re.compile(r"^(?P<value>[0-9]+(?:\.[0-9]+)?)(?P<unit>[A-Za-z]+)?$")
_SIZE_UNITS = {
    "B": 1,
    "KB": 1000,
    "MB": 1000**2,
    "GB": 1000**3,
    "TB": 1000**4,
    "KIB": 1024,
    "MIB": 1024**2,
    "GIB": 1024**3,
    "TIB": 1024**4,
}
_BENCHMARK_SCHEMA_VERSION = 2
_ROS_TRANSPORT = "ros2"
_SOURCE_TIMESTAMP_ORIGIN = "camera_source"
_TIMESTAMP_CLOCK = "host_wall_clock"
_INGRESS_TIMING_METRIC = "source_to_dtps_ingress_ms"
_OUTPUT_TIMING_METRIC = "source_to_ros_output_receive_ms"


@dataclass
class PendingFrame:
    received_at: float
    payload: bytes


@dataclass
class PendingTopicFrame:
    received_at: float
    payload_size_bytes: int
    header_stamp_sec: Optional[float]


@dataclass
class ResourceSample:
    timestamp: float
    cpu_percent: float
    mem_usage_bytes: float
    mem_limit_bytes: float
    mem_percent: float


@dataclass
class BenchmarkWindow:
    capture_started_at: float
    first_frame_at: float
    warmup_ended_at: float
    capture_ended_at: float


class FrameSink:
    def __init__(self, max_queue_size: int):
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._queue: asyncio.Queue[PendingFrame] = asyncio.Queue(maxsize=max_queue_size)
        self._dropped_count = 0

    def bind(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop

    async def on_http_frame(self, raw_data: Any) -> None:
        self._enqueue(raw_data.content, time.time())

    def on_shm_frame(self, payload: bytes) -> None:
        loop = self._loop
        if loop is None:
            return
        received_at = time.time()
        loop.call_soon_threadsafe(self._enqueue, payload, received_at)

    def _enqueue(self, payload: bytes, received_at: float) -> None:
        if self._queue.full():
            try:
                self._queue.get_nowait()
            except asyncio.QueueEmpty:
                pass
            else:
                self._dropped_count += 1
        self._queue.put_nowait(PendingFrame(received_at=received_at, payload=payload))

    async def get(self, timeout_s: float) -> PendingFrame:
        return await asyncio.wait_for(self._queue.get(), timeout=timeout_s)

    @property
    def dropped_count(self) -> int:
        return self._dropped_count


class TopicFrameSink:
    def __init__(self, max_queue_size: int):
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._queue: asyncio.Queue[PendingTopicFrame] = asyncio.Queue(maxsize=max_queue_size)
        self._dropped_count = 0

    def bind(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop

    def on_ros2_frame(self, payload_size_bytes: int, header_stamp_sec: Optional[float]) -> None:
        loop = self._loop
        if loop is None:
            return
        received_at = time.time()
        loop.call_soon_threadsafe(
            self._enqueue,
            PendingTopicFrame(
                received_at=received_at,
                payload_size_bytes=payload_size_bytes,
                header_stamp_sec=header_stamp_sec,
            ),
        )

    def _enqueue(self, frame: PendingTopicFrame) -> None:
        if self._queue.full():
            try:
                self._queue.get_nowait()
            except asyncio.QueueEmpty:
                pass
            else:
                self._dropped_count += 1
        self._queue.put_nowait(frame)

    async def get(self, timeout_s: float) -> PendingTopicFrame:
        return await asyncio.wait_for(self._queue.get(), timeout=timeout_s)

    @property
    def dropped_count(self) -> int:
        return self._dropped_count


def _log(message: str) -> None:
    print(message, flush=True)


def _warn(message: str) -> None:
    print(f"WARN: {message}", file=sys.stderr, flush=True)


def _err(message: str) -> None:
    print(f"ERROR: {message}", file=sys.stderr, flush=True)


def _parse_percent(value: str) -> float:
    text = value.strip()
    if not text:
        return 0.0
    if text.endswith("%"):
        text = text[:-1]
    return float(text)


def _parse_size_to_bytes(value: str) -> float:
    text = value.strip()
    if not text:
        return 0.0
    match = _SIZE_RE.match(text)
    if match is None:
        raise ValueError(f"Unsupported size format: '{value}'")
    amount = float(match.group("value"))
    unit = (match.group("unit") or "B").upper()
    if unit not in _SIZE_UNITS:
        raise ValueError(f"Unsupported size unit '{unit}' in '{value}'")
    return amount * _SIZE_UNITS[unit]


def _parse_mem_usage(value: str) -> tuple[float, float]:
    used_text, _, limit_text = value.partition("/")
    used_bytes = _parse_size_to_bytes(used_text)
    limit_bytes = _parse_size_to_bytes(limit_text) if limit_text.strip() else 0.0
    return used_bytes, limit_bytes


def _percentile(values: list[float], percentile: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * (percentile / 100.0)
    lower_index = int(position)
    upper_index = min(lower_index + 1, len(ordered) - 1)
    weight = position - lower_index
    lower = ordered[lower_index]
    upper = ordered[upper_index]
    return lower + ((upper - lower) * weight)


def _summarize(values: list[float]) -> dict[str, Optional[float]]:
    if not values:
        return {
            "count": 0,
            "min": None,
            "mean": None,
            "p50": None,
            "p95": None,
            "max": None,
            "stdev": None,
        }
    stdev = 0.0
    if len(values) > 1:
        stdev = statistics.pstdev(values)
    return {
        "count": len(values),
        "min": min(values),
        "mean": statistics.fmean(values),
        "p50": _percentile(values, 50.0),
        "p95": _percentile(values, 95.0),
        "max": max(values),
        "stdev": stdev,
    }


def _bytes_to_mib(value: Optional[float]) -> Optional[float]:
    if value is None:
        return None
    return value / (1024**2)


def _cpu_percent_from_stats(stats: dict[str, Any]) -> float:
    cpu_stats = stats.get("cpu_stats", {})
    precpu_stats = stats.get("precpu_stats", {})
    cpu_usage = cpu_stats.get("cpu_usage", {})
    precpu_usage = precpu_stats.get("cpu_usage", {})
    total_usage = float(cpu_usage.get("total_usage", 0.0))
    pre_total_usage = float(precpu_usage.get("total_usage", 0.0))
    system_usage = float(cpu_stats.get("system_cpu_usage", 0.0))
    pre_system_usage = float(precpu_stats.get("system_cpu_usage", 0.0))
    online_cpus = cpu_stats.get("online_cpus")
    if online_cpus is None:
        percpu_usage = cpu_usage.get("percpu_usage", [])
        online_cpus = len(percpu_usage)

    cpu_delta = total_usage - pre_total_usage
    system_delta = system_usage - pre_system_usage
    if cpu_delta <= 0 or system_delta <= 0 or not online_cpus:
        return 0.0
    return (cpu_delta / system_delta) * float(online_cpus) * 100.0


def _read_docker_stats_via_sdk(containers: list[str]) -> dict[str, ResourceSample]:
    docker_sdk = _load_docker_sdk()
    client = docker_sdk.from_env()
    try:
        timestamp = time.time()
        samples: dict[str, ResourceSample] = {}
        for container_name in containers:
            container = client.containers.get(container_name)
            stats = container.stats(stream=False)
            memory_stats = stats.get("memory_stats", {})
            mem_usage_bytes = float(memory_stats.get("usage", 0.0))
            mem_limit_bytes = float(memory_stats.get("limit", 0.0))
            mem_percent = 0.0
            if mem_limit_bytes > 0:
                mem_percent = (mem_usage_bytes / mem_limit_bytes) * 100.0
            sample = ResourceSample(
                timestamp=timestamp,
                cpu_percent=_cpu_percent_from_stats(stats),
                mem_usage_bytes=mem_usage_bytes,
                mem_limit_bytes=mem_limit_bytes,
                mem_percent=mem_percent,
            )
            samples[container_name] = sample
        return samples
    finally:
        client.close()


def _read_container_env_via_sdk(container_name: str) -> dict[str, str]:
    docker_sdk = _load_docker_sdk()
    client = docker_sdk.from_env()
    try:
        container = client.containers.get(container_name)
        attrs = container.attrs
        config = attrs.get("Config", {})
        env_lines = config.get("Env", [])
        env: dict[str, str] = {}
        for line in env_lines:
            key, separator, value = line.partition("=")
            if separator:
                env[key] = value
        return env
    finally:
        client.close()


def _read_docker_stats(containers: list[str]) -> dict[str, ResourceSample]:
    if not containers:
        return {}
    command = [
        "docker",
        "stats",
        "--no-stream",
        "--format",
        "{{json .}}",
    ]
    command.extend(containers)
    try:
        result = subprocess.run(command, capture_output=True, text=True)
    except FileNotFoundError:
        return _read_docker_stats_via_sdk(containers)
    if result.returncode != 0:
        stderr = result.stderr.strip()
        raise RuntimeError(f"docker stats failed: {stderr or result.stdout.strip()}")

    samples: dict[str, ResourceSample] = {}
    timestamp = time.time()
    for line in result.stdout.splitlines():
        text = line.strip()
        if not text:
            continue
        payload = json.loads(text)
        container_name = payload.get("Name") or payload.get("Container")
        if not container_name:
            continue
        mem_usage_bytes, mem_limit_bytes = _parse_mem_usage(str(payload.get("MemUsage", "0B / 0B")))
        sample = ResourceSample(
            timestamp=timestamp,
            cpu_percent=_parse_percent(str(payload.get("CPUPerc", "0"))),
            mem_usage_bytes=mem_usage_bytes,
            mem_limit_bytes=mem_limit_bytes,
            mem_percent=_parse_percent(str(payload.get("MemPerc", "0"))),
        )
        samples[container_name] = sample
    return samples


def _read_container_env(container_name: str) -> dict[str, str]:
    command = [
        "docker",
        "inspect",
        "--format",
        "{{range .Config.Env}}{{println .}}{{end}}",
        container_name,
    ]
    try:
        result = subprocess.run(command, capture_output=True, text=True)
    except FileNotFoundError:
        return _read_container_env_via_sdk(container_name)
    if result.returncode != 0:
        stderr = result.stderr.strip()
        raise RuntimeError(f"docker inspect failed for '{container_name}': {stderr or result.stdout.strip()}")
    env: dict[str, str] = {}
    for line in result.stdout.splitlines():
        key, separator, value = line.partition("=")
        if separator:
            env[key] = value
    return env


def _validate_mode(args: argparse.Namespace) -> None:
    if (
        args.command == "capture"
        and args.transport == "shm"
        and not args.allow_direct_shm_reader
    ):
        raise RuntimeError(
            "Direct SHM capture competes with the live SHM consumer for FIFO "
            "wake-ups. Stop the consumer and pass --allow-direct-shm-reader "
            "only for an isolated DTPS-ingress measurement."
        )
    mode_container = args.mode_container
    if mode_container:
        env = _read_container_env(mode_container)
        shm_value = env.get("DT_CAMERA_SHM_IN_PATH", "").strip()
        consumer_shm_only = env.get(
            "DT_CAMERA_SHM_ONLY_JPEG",
            "0",
        ).strip()
        consumer_uses_shm_only = (
            shm_value != ""
            and consumer_shm_only == "1"
        )
        if args.transport == "shm" and not consumer_uses_shm_only:
            raise RuntimeError(
                f"Container '{mode_container}' is not configured for SHM-only JPEG input. "
                "Set DT_CAMERA_SHM_IN_PATH and "
                "DT_CAMERA_SHM_ONLY_JPEG=1 before running the SHM benchmark."
            )
        if args.transport == "http" and consumer_uses_shm_only:
            raise RuntimeError(
                f"Container '{mode_container}' is configured for SHM-only JPEG input at '{shm_value}'. "
                "Set DT_CAMERA_SHM_ONLY_JPEG=0 before running the HTTP benchmark."
            )

    producer_container = args.producer_container
    if args.transport != "http" or not producer_container:
        return
    producer_env = _read_container_env(producer_container)
    producer_shm_path = producer_env.get("DT_CAMERA_SHM_OUT_PATH", "").strip()
    producer_shm_only = producer_env.get(
        "DT_CAMERA_SHM_ONLY_JPEG",
        "0",
    ).strip()
    producer_has_shm_output = producer_shm_path != ""
    producer_uses_shm_only = (
        producer_has_shm_output
        and producer_shm_only == "1"
    )
    if producer_uses_shm_only:
        raise RuntimeError(
            f"Producer '{producer_container}' is configured for SHM-only JPEG output at "
            f"'{producer_shm_path}'. Set DT_CAMERA_SHM_ONLY_JPEG=0 "
            "before running the HTTP benchmark."
        )


async def _sample_resources(
    containers: list[str],
    interval_s: float,
    stop_event: asyncio.Event,
    samples_by_container: dict[str, list[ResourceSample]],
) -> None:
    if not containers:
        return
    while not stop_event.is_set():
        started_at = time.time()
        samples = await asyncio.to_thread(_read_docker_stats, containers)
        for container_name, sample in samples.items():
            container_samples = samples_by_container.setdefault(container_name, [])
            container_samples.append(sample)

        elapsed = time.time() - started_at
        remaining = interval_s - elapsed
        if remaining <= 0:
            continue
        try:
            await asyncio.wait_for(stop_event.wait(), timeout=remaining)
        except asyncio.TimeoutError:
            continue


async def _stream_http_frames(
    context_name: str,
    robot_name: str,
    camera_name: str,
    topic_timeout_s: float,
    sink: FrameSink,
    stop_event: asyncio.Event,
) -> None:
    imports = _load_capture_imports()
    context_fn = imports["context"]
    context_config = imports["ContextConfig"]
    switchboard = await context_fn(context_name)
    robot_context = switchboard.navigate(robot_name)
    camera_context = robot_context / "sensor" / "camera" / camera_name / "jpeg"
    jpeg_queue = await camera_context.until_ready(timeout=topic_timeout_s)
    jpeg_queue = jpeg_queue.configure(context_config(patient=True))
    await jpeg_queue.subscribe(sink.on_http_frame)
    await stop_event.wait()


async def _stream_shm_frames(
    shm_path: str,
    sink: FrameSink,
    stop_event: asyncio.Event,
) -> None:
    reader_cls = _load_shm_reader_class()
    reader = reader_cls(shm_path, sink.on_shm_frame, _warn, _err)
    reader.start()
    try:
        await stop_event.wait()
    finally:
        reader.stop()


def _message_stamp_to_seconds(message: Any) -> Optional[float]:
    header = getattr(message, "header", None)
    if header is None:
        return None
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    sec = int(getattr(stamp, "sec", 0))
    nanosec = int(getattr(stamp, "nanosec", 0))
    return _timestamp_to_seconds(
        float(sec) + (float(nanosec) / 1_000_000_000.0),
    )


def _timestamp_to_seconds(timestamp: Any) -> Optional[float]:
    """Normalize a positive source timestamp represented in seconds."""
    if timestamp is None:
        return None
    try:
        timestamp_seconds = float(timestamp)
    except (TypeError, ValueError):
        return None
    if timestamp_seconds <= 0 or not math.isfinite(timestamp_seconds):
        return None
    return timestamp_seconds


def _append_timestamp_latency(
    timestamp_seconds: Optional[float],
    received_at: float,
    latency_values_ms: list[float],
) -> str:
    """Append a valid timestamp delay and identify invalid clock relationships."""
    if timestamp_seconds is None:
        return "missing"
    latency_ms = (received_at - timestamp_seconds) * 1000.0
    if latency_ms < 0:
        return "negative"
    latency_values_ms.append(latency_ms)
    return "recorded"


async def _stream_ros2_topic_frames(
    topic_name: str,
    ros_domain_id: Optional[int],
    sink: TopicFrameSink,
    stop_event: asyncio.Event,
) -> None:
    imports = _load_ros2_capture_imports()
    rclpy = imports["rclpy"]
    executor_cls = imports["SingleThreadedExecutor"]
    node_cls = imports["ROS2Node"]
    compressed_image_cls = imports["ROS2CompressedImage"]

    if ros_domain_id is not None:
        os.environ["ROS_DOMAIN_ID"] = str(ros_domain_id)

    did_init = False
    if not rclpy.ok():
        rclpy.init(args=None)
        did_init = True

    node = node_cls("camera_transport_benchmark")
    executor = executor_cls()
    executor.add_node(node)

    def _callback(message: Any) -> None:
        header_stamp_sec = _message_stamp_to_seconds(message)
        payload_size_bytes = len(message.data)
        sink.on_ros2_frame(payload_size_bytes, header_stamp_sec)

    subscription = node.create_subscription(
        compressed_image_cls,
        topic_name,
        _callback,
        10,
    )
    spin_thread = Thread(
        target=executor.spin,
        daemon=True,
        name="Ros2TopicCapture",
    )
    spin_thread.start()

    try:
        await stop_event.wait()
    finally:
        _ = subscription
        shutdown_ok = executor.shutdown(timeout_sec=1)
        if not shutdown_ok:
            _warn("ROS2 benchmark executor did not shut down within 1 second.")
        spin_thread.join(timeout=1)
        if spin_thread.is_alive():
            _warn("ROS2 benchmark executor thread is still running; skipping node destruction.")
        else:
            node.destroy_node()
        if did_init and rclpy.ok():
            rclpy.shutdown()


async def _collect_frame_metrics(
    sink: FrameSink,
    capture_started_at: float,
    startup_timeout_s: float,
    warmup_s: float,
    duration_s: float,
) -> tuple[dict[str, Any], BenchmarkWindow]:
    imports = _load_capture_imports()
    compressed_image_cls = imports["CompressedImage"]
    data_decoding_error = imports["DataDecodingError"]
    raw_data_cls = imports["RawData"]
    first_frame = await sink.get(startup_timeout_s)
    warmup_ended_at = first_frame.received_at + warmup_s
    capture_ended_at = warmup_ended_at + duration_s
    window = BenchmarkWindow(
        capture_started_at=capture_started_at,
        first_frame_at=first_frame.received_at,
        warmup_ended_at=warmup_ended_at,
        capture_ended_at=capture_ended_at,
    )

    source_to_dtps_ingress_ms_values: list[float] = []
    jpeg_sizes_bytes: list[float] = []
    frame_intervals_ms: list[float] = []
    measured_frame_count = 0
    decode_error_count = 0
    missing_timestamp_count = 0
    negative_timestamp_delta_count = 0
    last_measured_frame_at: Optional[float] = None
    last_error: Optional[str] = None
    pending_frame: Optional[PendingFrame] = first_frame

    while True:
        frame = pending_frame
        pending_frame = None
        if frame is None:
            remaining = capture_ended_at - time.time()
            if remaining <= 0:
                break
            try:
                frame = await sink.get(remaining)
            except asyncio.TimeoutError:
                break

        if frame.received_at > capture_ended_at:
            break
        if frame.received_at < warmup_ended_at:
            continue

        raw_data = raw_data_cls(frame.payload, "application/cbor")
        try:
            jpeg = compressed_image_cls.from_rawdata(raw_data)
        except data_decoding_error as exc:
            decode_error_count += 1
            last_error = exc.message
            continue
        except Exception as exc:  # pragma: no cover - defensive guard for malformed payloads
            decode_error_count += 1
            last_error = str(exc)
            continue

        timestamp_seconds = _timestamp_to_seconds(jpeg.header.timestamp)
        timestamp_status = _append_timestamp_latency(
            timestamp_seconds,
            frame.received_at,
            source_to_dtps_ingress_ms_values,
        )
        if timestamp_status == "missing":
            missing_timestamp_count += 1
        elif timestamp_status == "negative":
            negative_timestamp_delta_count += 1

        jpeg_sizes_bytes.append(float(len(jpeg.data)))
        if last_measured_frame_at is not None:
            interval_ms = (frame.received_at - last_measured_frame_at) * 1000.0
            frame_intervals_ms.append(interval_ms)
        last_measured_frame_at = frame.received_at
        measured_frame_count += 1

    if measured_frame_count == 0:
        raise RuntimeError(
            "No frames were captured during the measurement window. "
            "Increase --startup-timeout/--warmup or check the active transport path."
        )

    return {
        "frame_count": measured_frame_count,
        "decode_error_count": decode_error_count,
        "decode_error": last_error,
        "missing_timestamp_count": missing_timestamp_count,
        "negative_timestamp_delta_count": negative_timestamp_delta_count,
        "queue_dropped_count": sink.dropped_count,
        "effective_fps": measured_frame_count / duration_s,
        "source_to_dtps_ingress_ms": _summarize(
            source_to_dtps_ingress_ms_values,
        ),
        "jpeg_size_bytes": _summarize(jpeg_sizes_bytes),
        "frame_interval_ms": _summarize(frame_intervals_ms),
    }, window


async def _collect_output_topic_metrics(
    sink: TopicFrameSink,
    capture_started_at: float,
    startup_timeout_s: float,
    warmup_s: float,
    duration_s: float,
) -> tuple[dict[str, Any], BenchmarkWindow]:
    first_frame = await sink.get(startup_timeout_s)
    warmup_ended_at = first_frame.received_at + warmup_s
    capture_ended_at = warmup_ended_at + duration_s
    window = BenchmarkWindow(
        capture_started_at=capture_started_at,
        first_frame_at=first_frame.received_at,
        warmup_ended_at=warmup_ended_at,
        capture_ended_at=capture_ended_at,
    )

    source_to_ros_output_receive_ms_values: list[float] = []
    jpeg_sizes_bytes: list[float] = []
    frame_intervals_ms: list[float] = []
    measured_frame_count = 0
    missing_timestamp_count = 0
    negative_timestamp_delta_count = 0
    last_measured_frame_at: Optional[float] = None
    pending_frame: Optional[PendingTopicFrame] = first_frame

    while True:
        frame = pending_frame
        pending_frame = None
        if frame is None:
            remaining = capture_ended_at - time.time()
            if remaining <= 0:
                break
            try:
                frame = await sink.get(remaining)
            except asyncio.TimeoutError:
                break

        if frame.received_at > capture_ended_at:
            break
        if frame.received_at < warmup_ended_at:
            continue

        timestamp_status = _append_timestamp_latency(
            frame.header_stamp_sec,
            frame.received_at,
            source_to_ros_output_receive_ms_values,
        )
        if timestamp_status == "missing":
            missing_timestamp_count += 1
        elif timestamp_status == "negative":
            negative_timestamp_delta_count += 1

        jpeg_sizes_bytes.append(float(frame.payload_size_bytes))
        if last_measured_frame_at is not None:
            interval_ms = (frame.received_at - last_measured_frame_at) * 1000.0
            frame_intervals_ms.append(interval_ms)
        last_measured_frame_at = frame.received_at
        measured_frame_count += 1

    if measured_frame_count == 0:
        raise RuntimeError(
            "No ROS2 topic frames were captured during the measurement window. "
            "Increase --startup-timeout/--warmup or check the ROS2 topic name and domain."
        )

    return {
        "frame_count": measured_frame_count,
        "decode_error_count": 0,
        "decode_error": None,
        "missing_timestamp_count": missing_timestamp_count,
        "negative_timestamp_delta_count": negative_timestamp_delta_count,
        "queue_dropped_count": sink.dropped_count,
        "effective_fps": measured_frame_count / duration_s,
        "source_to_ros_output_receive_ms": _summarize(
            source_to_ros_output_receive_ms_values,
        ),
        "jpeg_size_bytes": _summarize(jpeg_sizes_bytes),
        "frame_interval_ms": _summarize(frame_intervals_ms),
    }, window


def _summarize_resources(
    samples_by_container: dict[str, list[ResourceSample]],
    window: BenchmarkWindow,
) -> dict[str, Any]:
    summaries: dict[str, Any] = {}
    for container_name, samples in samples_by_container.items():
        filtered = []
        for sample in samples:
            if sample.timestamp < window.warmup_ended_at:
                continue
            if sample.timestamp > window.capture_ended_at:
                continue
            filtered.append(sample)

        cpu_values = [sample.cpu_percent for sample in filtered]
        mem_usage_values = [sample.mem_usage_bytes for sample in filtered]
        mem_usage_mib_values = [_bytes_to_mib(value) or 0.0 for value in mem_usage_values]
        mem_percent_values = [sample.mem_percent for sample in filtered]
        summaries[container_name] = {
            "sample_count": len(filtered),
            "cpu_percent": _summarize(cpu_values),
            "mem_usage_bytes": _summarize(mem_usage_values),
            "mem_usage_mib": _summarize(mem_usage_mib_values),
            "mem_percent": _summarize(mem_percent_values),
        }
    return summaries


def _print_capture_summary(result: dict[str, Any]) -> None:
    frame_metrics = result["frame_metrics"]
    measurement_surface = result.get("measurement_surface", "transport_ingress")
    _log("")
    _log(f"Capture label: {result['label']}")
    _log(f"DTPS transport: {result['dtps_transport']}")
    _log(f"ROS transport: {result['ros_transport']}")
    _log(f"Capture surface: {measurement_surface}")
    _log(f"Frames: {frame_metrics['frame_count']} ({frame_metrics['effective_fps']:.2f} fps over the measurement window)")
    if measurement_surface == "ros2_output_topic":
        source_to_output = frame_metrics.get(
            "source_to_ros_output_receive_ms",
            {},
        )
        _log(
            "Camera source->ROS2 receive ms: "
            f"mean={_format_metric(source_to_output.get('mean'))}, "
            f"p50={_format_metric(source_to_output.get('p50'))}, "
            f"p95={_format_metric(source_to_output.get('p95'))}, "
            f"max={_format_metric(source_to_output.get('max'))}"
        )
    else:
        source_to_ingress = frame_metrics.get(
            "source_to_dtps_ingress_ms",
            _summarize([]),
        )
        _log(
            "Camera source->DTPS ingress ms: "
            f"mean={_format_metric(source_to_ingress['mean'])}, "
            f"p50={_format_metric(source_to_ingress['p50'])}, "
            f"p95={_format_metric(source_to_ingress['p95'])}, "
            f"max={_format_metric(source_to_ingress['max'])}"
        )
    _log(
        "Timestamp gaps: "
        f"missing={frame_metrics['missing_timestamp_count']} "
        f"negative={frame_metrics['negative_timestamp_delta_count']}"
    )
    for container_name, summary in result["resource_metrics"].items():
        cpu = summary["cpu_percent"]
        memory = summary["mem_usage_mib"]
        _log(
            f"{container_name}: "
            f"cpu mean={_format_metric(cpu['mean'])}% "
            f"p95={_format_metric(cpu['p95'])}% | "
            f"mem mean={_format_metric(memory['mean'])} MiB "
            f"max={_format_metric(memory['max'])} MiB"
        )


def _format_metric(value: Optional[float]) -> str:
    if value is None:
        return "n/a"
    return f"{value:.3f}"


def _build_capture_result(
    args: argparse.Namespace,
    capture_started_at: float,
    capture_finished_at: float,
    frame_metrics: dict[str, Any],
    resource_metrics: dict[str, Any],
    window: BenchmarkWindow,
    measurement_surface: str,
    observed_topic: str = "",
    notes: Optional[list[str]] = None,
) -> dict[str, Any]:
    if measurement_surface == "ros2_output_topic":
        ros_transport = _ROS_TRANSPORT
        transport_path = f"DTPS {args.transport.upper()} -> ROS 2"
    else:
        ros_transport = "none"
        transport_path = f"DTPS {args.transport.upper()} ingress"
    result = {
        "benchmark_schema_version": _BENCHMARK_SCHEMA_VERSION,
        "label": args.label or args.transport,
        "transport": args.transport,
        "dtps_transport": args.transport,
        "ros_transport": ros_transport,
        "transport_path": transport_path,
        "timestamp_origin": _SOURCE_TIMESTAMP_ORIGIN,
        "timestamp_clock": _TIMESTAMP_CLOCK,
        "measurement_surface": measurement_surface,
        "robot_name": args.robot_name,
        "camera_name": args.camera_name,
        "context_name": getattr(args, "context_name", ""),
        "shm_path": getattr(args, "shm_path", ""),
        "observed_topic": observed_topic,
        "containers": args.containers,
        "capture_started_at": capture_started_at,
        "capture_finished_at": capture_finished_at,
        "measurement_window": {
            "capture_started_at": window.capture_started_at,
            "first_frame_at": window.first_frame_at,
            "warmup_ended_at": window.warmup_ended_at,
            "capture_ended_at": window.capture_ended_at,
            "warmup_s": args.warmup,
            "duration_s": args.duration,
        },
        "frame_metrics": frame_metrics,
        "resource_metrics": resource_metrics,
    }
    if notes:
        result["notes"] = notes
    return result


async def _run_capture(args: argparse.Namespace) -> dict[str, Any]:
    _validate_mode(args)
    sink = FrameSink(args.queue_size)
    loop = asyncio.get_running_loop()
    sink.bind(loop)
    stop_event = asyncio.Event()
    resource_samples: dict[str, list[ResourceSample]] = {}
    capture_started_at = time.time()

    if args.transport == "http":
        stream_task = asyncio.create_task(
            _stream_http_frames(
                args.context_name,
                args.robot_name,
                args.camera_name,
                args.topic_timeout,
                sink,
                stop_event,
            )
        )
    else:
        stream_task = asyncio.create_task(
            _stream_shm_frames(
                args.shm_path,
                sink,
                stop_event,
            )
        )
    resource_task = asyncio.create_task(
        _sample_resources(
            args.containers,
            args.sample_interval,
            stop_event,
            resource_samples,
        )
    )

    try:
        frame_metrics, window = await _collect_frame_metrics(
            sink,
            capture_started_at,
            args.startup_timeout,
            args.warmup,
            args.duration,
        )
    finally:
        stop_event.set()
        results = await asyncio.gather(stream_task, resource_task, return_exceptions=True)
        for result in results:
            if isinstance(result, Exception):
                raise result

    capture_finished_at = time.time()
    resource_metrics = _summarize_resources(resource_samples, window)
    return _build_capture_result(
        args,
        capture_started_at,
        capture_finished_at,
        frame_metrics,
        resource_metrics,
        window,
        measurement_surface="transport_ingress",
    )


def _resolve_output_topic(args: argparse.Namespace) -> str:
    if args.ros_topic:
        return args.ros_topic
    return f"/{args.robot_name}/camera_node/image/compressed"


async def _run_output_capture(args: argparse.Namespace) -> dict[str, Any]:
    _validate_mode(args)
    sink = TopicFrameSink(args.queue_size)
    loop = asyncio.get_running_loop()
    sink.bind(loop)
    stop_event = asyncio.Event()
    resource_samples: dict[str, list[ResourceSample]] = {}
    capture_started_at = time.time()
    observed_topic = _resolve_output_topic(args)

    stream_task = asyncio.create_task(
        _stream_ros2_topic_frames(
            observed_topic,
            args.ros_domain_id,
            sink,
            stop_event,
        )
    )
    resource_task = asyncio.create_task(
        _sample_resources(
            args.containers,
            args.sample_interval,
            stop_event,
            resource_samples,
        )
    )

    try:
        frame_metrics, window = await _collect_output_topic_metrics(
            sink,
            capture_started_at,
            args.startup_timeout,
            args.warmup,
            args.duration,
        )
    finally:
        stop_event.set()
        results = await asyncio.gather(stream_task, resource_task, return_exceptions=True)
        for result in results:
            if isinstance(result, Exception):
                raise result

    capture_finished_at = time.time()
    resource_metrics = _summarize_resources(resource_samples, window)
    notes = [
        "ROS2 output capture avoids opening the SHM FIFO, so it does not compete with the live SHM consumer.",
    ]
    return _build_capture_result(
        args,
        capture_started_at,
        capture_finished_at,
        frame_metrics,
        resource_metrics,
        window,
        measurement_surface="ros2_output_topic",
        observed_topic=observed_topic,
        notes=notes,
    )


def _write_json(path: str, payload: dict[str, Any]) -> None:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(payload, indent=2, sort_keys=True)
    output_path.write_text(text + "\n", encoding="utf-8")


def _load_json(path: str) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as infile:
        return json.load(infile)


def _metric_from_path(payload: dict[str, Any], path: tuple[str, ...]) -> Optional[float]:
    current: Any = payload
    for part in path:
        if not isinstance(current, dict):
            return None
        current = current.get(part)
    if current is None:
        return None
    return float(current)


def _get_timing_summary(payload: dict[str, Any]) -> tuple[str, dict[str, Any]]:
    """Return the timing metric whose boundary matches a capture result."""
    measurement_surface = payload.get("measurement_surface")
    if measurement_surface == "transport_ingress":
        metric_name = _INGRESS_TIMING_METRIC
    elif measurement_surface == "ros2_output_topic":
        metric_name = _OUTPUT_TIMING_METRIC
    else:
        raise ValueError(
            "Result has an unknown measurement surface: "
            f"{measurement_surface!r}."
        )
    frame_metrics = payload.get("frame_metrics")
    if not isinstance(frame_metrics, dict):
        raise ValueError("Result does not contain frame_metrics.")
    summary = frame_metrics.get(metric_name)
    if not isinstance(summary, dict):
        raise ValueError(
            f"Result does not contain the {metric_name!r} timing metric. "
            "Rerun it with the current benchmark script."
        )
    return metric_name, summary


def _validate_comparison_result(result: dict[str, Any]) -> None:
    """Ensure a result belongs to this ROS2 benchmark and timing contract."""
    if result.get("benchmark_schema_version") != _BENCHMARK_SCHEMA_VERSION:
        raise ValueError(
            "Results must use benchmark schema version "
            f"{_BENCHMARK_SCHEMA_VERSION}; rerun the capture first."
        )

    if result.get("measurement_surface") != "ros2_output_topic":
        raise ValueError(
            "Compare requires ROS2 output-topic captures; use capture-output "
            "for both the HTTP and SHM result."
        )
    if result.get("ros_transport") != _ROS_TRANSPORT:
        raise ValueError(
            "Result has an unexpected ROS transport for this ROS2 benchmark: "
            f"{result.get('ros_transport')!r}."
        )
    if result.get("timestamp_origin") != _SOURCE_TIMESTAMP_ORIGIN:
        raise ValueError("Result does not use camera-source timestamps.")
    if result.get("dtps_transport") not in ("http", "shm"):
        raise ValueError(
            "Result has an unexpected DTPS transport: "
            f"{result.get('dtps_transport')!r}."
        )

    _, timing_summary = _get_timing_summary(result)
    if timing_summary.get("count", 0) <= 0:
        raise ValueError("Result contains no valid timing samples.")
    if result.get("frame_metrics", {}).get("negative_timestamp_delta_count", 0) != 0:
        raise ValueError(
            "Result contains negative source-to-receive delays; synchronize "
            "clocks before comparing it."
        )


def _timing_metric_prefix(metric_name: str) -> str:
    """Return a metric name without its millisecond unit suffix."""
    if metric_name.endswith("_ms"):
        return metric_name[:-3]
    return metric_name


def _compare_row(name: str, left: Optional[float], right: Optional[float]) -> dict[str, Any]:
    delta = None
    delta_percent = None
    if left is not None and right is not None:
        delta = right - left
        if left != 0:
            delta_percent = (delta / left) * 100.0
    return {
        "metric": name,
        "left": left,
        "right": right,
        "delta": delta,
        "delta_percent": delta_percent,
    }


def _print_comparison(left: dict[str, Any], right: dict[str, Any], rows: list[dict[str, Any]]) -> None:
    left_label = left.get("label", "left")
    right_label = right.get("label", "right")
    _log("")
    _log(f"Comparing {left_label} -> {right_label}")
    header = f"{'Metric':<30} {left_label:>12} {right_label:>12} {'Delta':>12} {'Delta %':>10}"
    _log(header)
    _log("-" * len(header))
    for row in rows:
        left_value = _format_metric(row["left"])
        right_value = _format_metric(row["right"])
        delta_value = _format_metric(row["delta"])
        delta_percent = row["delta_percent"]
        delta_percent_text = "n/a"
        if delta_percent is not None:
            delta_percent_text = f"{delta_percent:.2f}"
        _log(
            f"{row['metric']:<30} {left_value:>12} {right_value:>12} {delta_value:>12} {delta_percent_text:>10}"
        )


def _build_comparison_rows(left: dict[str, Any], right: dict[str, Any]) -> list[dict[str, Any]]:
    _validate_comparison_result(left)
    _validate_comparison_result(right)
    if left.get("measurement_surface") != right.get("measurement_surface"):
        raise ValueError(
            "Cannot compare results captured at different measurement surfaces."
        )
    if left.get("timestamp_origin") != right.get("timestamp_origin"):
        raise ValueError(
            "Cannot compare results with different timestamp origins."
        )
    timing_metric, left_timing = _get_timing_summary(left)
    right_timing_metric, right_timing = _get_timing_summary(right)
    if timing_metric != right_timing_metric:
        raise ValueError("Cannot compare results with different timing metrics.")
    if {left.get("dtps_transport"), right.get("dtps_transport")} != {"http", "shm"}:
        raise ValueError("Compare one HTTP result with one SHM result.")
    timing_prefix = _timing_metric_prefix(timing_metric)
    rows = [
        _compare_row("fps", left["frame_metrics"].get("effective_fps"), right["frame_metrics"].get("effective_fps")),
        _compare_row(
            f"{timing_prefix}_mean_ms",
            left_timing.get("mean"),
            right_timing.get("mean"),
        ),
        _compare_row(
            f"{timing_prefix}_p50_ms",
            left_timing.get("p50"),
            right_timing.get("p50"),
        ),
        _compare_row(
            f"{timing_prefix}_p95_ms",
            left_timing.get("p95"),
            right_timing.get("p95"),
        ),
        _compare_row(
            f"{timing_prefix}_max_ms",
            left_timing.get("max"),
            right_timing.get("max"),
        ),
        _compare_row(
            "missing_timestamp_count",
            left["frame_metrics"].get("missing_timestamp_count"),
            right["frame_metrics"].get("missing_timestamp_count"),
        ),
        _compare_row(
            "negative_timestamp_delta_count",
            left["frame_metrics"].get("negative_timestamp_delta_count"),
            right["frame_metrics"].get("negative_timestamp_delta_count"),
        ),
        _compare_row(
            "queue_dropped_count",
            left["frame_metrics"].get("queue_dropped_count"),
            right["frame_metrics"].get("queue_dropped_count"),
        ),
    ]

    container_names = set(left.get("resource_metrics", {}).keys())
    container_names.update(right.get("resource_metrics", {}).keys())
    for container_name in sorted(container_names):
        rows.append(
            _compare_row(
                f"{container_name}_cpu_mean_pct",
                _metric_from_path(left, ("resource_metrics", container_name, "cpu_percent", "mean")),
                _metric_from_path(right, ("resource_metrics", container_name, "cpu_percent", "mean")),
            )
        )
        rows.append(
            _compare_row(
                f"{container_name}_cpu_p95_pct",
                _metric_from_path(left, ("resource_metrics", container_name, "cpu_percent", "p95")),
                _metric_from_path(right, ("resource_metrics", container_name, "cpu_percent", "p95")),
            )
        )
        rows.append(
            _compare_row(
                f"{container_name}_mem_mean_mib",
                _metric_from_path(left, ("resource_metrics", container_name, "mem_usage_mib", "mean")),
                _metric_from_path(right, ("resource_metrics", container_name, "mem_usage_mib", "mean")),
            )
        )
        rows.append(
            _compare_row(
                f"{container_name}_mem_max_mib",
                _metric_from_path(left, ("resource_metrics", container_name, "mem_usage_mib", "max")),
                _metric_from_path(right, ("resource_metrics", container_name, "mem_usage_mib", "max")),
            )
        )
    return rows


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Benchmark camera transport latency and container resource usage for a single transport mode, "
            "then compare HTTP and SHM result files."
        )
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    capture_parser = subparsers.add_parser("capture", help="Capture one benchmark run")
    capture_parser.add_argument(
        "--transport",
        choices=("http", "shm"),
        required=True,
        help="Transport mode.",
    )
    capture_parser.add_argument("--label", default="", help="Optional label stored in the result file")
    capture_parser.add_argument("--robot-name", required=True)
    capture_parser.add_argument("--camera-name", default="front_center")
    capture_parser.add_argument("--context-name", default="switchboard")
    capture_parser.add_argument("--shm-path", default="/dtps-shm/front_center")
    capture_parser.add_argument("--duration", type=float, default=30.0)
    capture_parser.add_argument("--warmup", type=float, default=5.0)
    capture_parser.add_argument("--startup-timeout", type=float, default=20.0)
    capture_parser.add_argument("--topic-timeout", type=float, default=20.0)
    capture_parser.add_argument("--sample-interval", type=float, default=1.0)
    capture_parser.add_argument("--queue-size", type=int, default=256)
    capture_parser.add_argument(
        "--allow-direct-shm-reader",
        action="store_true",
        help=(
            "Allow direct SHM ingress capture only when no live consumer is "
            "sharing the FIFO."
        ),
    )
    capture_parser.add_argument(
        "--containers",
        nargs="*",
        default=["driver-camera", "ros2-camera"],
        help="Container names to sample with docker stats",
    )
    capture_parser.add_argument(
        "--mode-container",
        default="ros2-camera",
        help="Container whose DT_CAMERA_SHM_IN_PATH env is validated against --transport",
    )
    capture_parser.add_argument(
        "--producer-container",
        default="driver-camera",
        help="Producer whose DT_CAMERA_SHM_ONLY_JPEG env is checked for HTTP mode",
    )
    capture_parser.add_argument("--output-json", default="")

    output_parser = subparsers.add_parser(
        "capture-output",
        help="Capture throughput from the ROS2 output topic without opening the SHM transport directly",
    )
    output_parser.add_argument(
        "--transport",
        choices=("http", "shm"),
        required=True,
        help="Transport mode.",
    )
    output_parser.add_argument("--label", default="", help="Optional label stored in the result file")
    output_parser.add_argument("--robot-name", required=True)
    output_parser.add_argument("--camera-name", default="front_center")
    output_parser.add_argument("--ros-topic", default="")
    output_parser.add_argument("--ros-domain-id", type=int, default=None)
    output_parser.add_argument("--duration", type=float, default=30.0)
    output_parser.add_argument("--warmup", type=float, default=5.0)
    output_parser.add_argument("--startup-timeout", type=float, default=20.0)
    output_parser.add_argument("--sample-interval", type=float, default=1.0)
    output_parser.add_argument("--queue-size", type=int, default=256)
    output_parser.add_argument(
        "--containers",
        nargs="*",
        default=["driver-camera", "ros2-camera"],
        help="Container names to sample with docker stats",
    )
    output_parser.add_argument(
        "--mode-container",
        default="ros2-camera",
        help="Container whose DT_CAMERA_SHM_IN_PATH env is validated against --transport",
    )
    output_parser.add_argument(
        "--producer-container",
        default="driver-camera",
        help="Producer whose DT_CAMERA_SHM_ONLY_JPEG env is checked for HTTP mode",
    )
    output_parser.add_argument("--output-json", default="")

    compare_parser = subparsers.add_parser("compare", help="Compare two benchmark result files")
    compare_parser.add_argument("left_json")
    compare_parser.add_argument("right_json")
    compare_parser.add_argument("--output-json", default="")
    return parser


def _run_compare(args: argparse.Namespace) -> dict[str, Any]:
    left = _load_json(args.left_json)
    right = _load_json(args.right_json)
    rows = _build_comparison_rows(left, right)
    comparison = {
        "left_label": left.get("label", "left"),
        "right_label": right.get("label", "right"),
        "rows": rows,
    }
    _print_comparison(left, right, rows)
    if args.output_json:
        _write_json(args.output_json, comparison)
    return comparison


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()

    if args.command == "compare":
        _run_compare(args)
        return 0

    if args.command == "capture-output":
        _ensure_ros_runtime_environment()
        observed_topic = _resolve_output_topic(args)
        _log(
            f"Starting isolated {args.transport.upper()} output capture on '{observed_topic}' "
            f"with a {args.duration:.1f}s measurement window."
        )
        result = asyncio.run(_run_output_capture(args))
        _print_capture_summary(result)
        if args.output_json:
            _write_json(args.output_json, result)
            _log(f"Saved result to {args.output_json}")
        return 0

    _log(
        f"Starting {args.transport.upper()} capture for robot '{args.robot_name}' "
        f"camera '{args.camera_name}' with {args.duration:.1f}s measurement window."
    )
    result = asyncio.run(_run_capture(args))
    _print_capture_summary(result)
    if args.output_json:
        _write_json(args.output_json, result)
        _log(f"Saved result to {args.output_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
