#!/usr/bin/env python3

import json
from pathlib import Path
import runpy
import shutil
import sys
from numbers import Real
from typing import Any


def _normalize_legacy_name(value: Any) -> Any:
    if isinstance(value, str) and value.startswith("//"):
        return value.lstrip("/")
    return value


def _normalize_service_name(value: Any) -> Any:
    value = _normalize_legacy_name(value)
    if isinstance(value, str) and value.startswith("/rosapi/"):
        return value.lstrip("/")
    return value


def _normalize_throttle_rate(value: Any) -> Any:
    if isinstance(value, Real) and not isinstance(value, bool):
        return max(0, int(round(float(value))))
    return value


def _normalize_message(payload: Any) -> Any:
    if isinstance(payload, dict):
        normalized = {}
        for key, value in payload.items():
            if key in {"topic", "action"}:
                normalized[key] = _normalize_legacy_name(value)
            elif key == "service":
                normalized[key] = _normalize_service_name(value)
            elif key == "throttle_rate":
                normalized[key] = _normalize_throttle_rate(value)
            else:
                normalized[key] = _normalize_message(value)
        return normalized

    if isinstance(payload, list):
        return [_normalize_message(item) for item in payload]

    return payload


def _patch_rosbridge_websocket() -> None:
    from rosbridge_server.websocket_handler import RosbridgeWebSocket

    original_on_message = RosbridgeWebSocket.on_message

    def patched_on_message(self, message: str | bytes) -> None:
        if isinstance(message, bytes):
            message = message.decode("utf-8")

        try:
            payload = json.loads(message)
        except Exception:
            original_on_message(self, message)
            return

        normalized = _normalize_message(payload)
        if normalized != payload:
            message = json.dumps(normalized, separators=(",", ":"))

        original_on_message(self, message)

    RosbridgeWebSocket.on_message = patched_on_message


def _find_upstream_executable() -> str:
    candidates: list[Path] = []

    try:
        from ament_index_python.packages import get_package_prefix
    except ImportError:
        get_package_prefix = None

    if get_package_prefix is not None:
        package_prefix = get_package_prefix("rosbridge_server")
        prefix_path = Path(package_prefix)
        libexec_dir = prefix_path / "lib" / "rosbridge_server"
        candidates.append(libexec_dir / "rosbridge_websocket")
        candidates.append(libexec_dir / "rosbridge_websocket.py")

    executable_path = shutil.which("rosbridge_websocket")
    if executable_path is not None:
        candidates.append(Path(executable_path))

    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)

    raise RuntimeError("Could not locate upstream rosbridge_websocket executable")


def main() -> None:
    upstream = _find_upstream_executable()

    _patch_rosbridge_websocket()

    original_argv0 = sys.argv[0]
    try:
        sys.argv[0] = upstream
        runpy.run_path(upstream, run_name="__main__")
    finally:
        sys.argv[0] = original_argv0


if __name__ == "__main__":
    main()
