#!/usr/bin/env python3

import argparse
import json
import threading
import time
from typing import Any

import rclpy
from mrs_msgs.msg import SystemHealthInfo
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict


SENSOR_LEVEL_MAP = {
    0: "OK",
    1: "WARN",
    2: "ERROR",
    3: "STALE",
}

SENSOR_TYPE_MAP = {
    0: "UNKNOWN",
    1: "AUTOPILOT",
    2: "RANGEFINDER",
    3: "GNSS",
    4: "IMU",
    5: "BAROMETER",
    6: "MAGNETOMETER",
    7: "CAMERA",
    8: "LIDAR",
    9: "REMOTE_CONTROLLER",
}


class SystemHealthBridge(Node):
    def __init__(self, topic_name: str):
        super().__init__("system_health_http_bridge")
        self._lock = threading.Lock()
        self._latest_payload: dict[str, Any] | None = None
        self._latest_ros_time: dict[str, int] | None = None
        self._latest_wall_time: float | None = None
        self.create_subscription(SystemHealthInfo, topic_name, self._callback, 10)

    def _callback(self, msg: SystemHealthInfo) -> None:
        payload = dict(message_to_ordereddict(msg))
        ros_stamp = None

        if hasattr(msg, "stamp"):
            ros_stamp = {"sec": msg.stamp.sec, "nanosec": msg.stamp.nanosec}
        elif hasattr(msg, "header") and hasattr(msg.header, "stamp"):
            ros_stamp = {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec}
        else:
            # SystemHealthInfo in this package has no top-level stamp.
            ros_stamp = payload.get("stamp")

        with self._lock:
            self._latest_payload = payload
            self._latest_ros_time = ros_stamp
            self._latest_wall_time = time.time()

    def get_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "available": self._latest_payload is not None,
                "received_at_unix": self._latest_wall_time,
                "ros_stamp": self._latest_ros_time,
                "data": self._latest_payload,
            }

    def get_available_sensors(self) -> dict[str, Any]:
        with self._lock:
            if self._latest_payload is None:
                return {
                    "available": False,
                    "count": 0,
                    "names": [],
                    "sensors": [],
                    "received_at_unix": self._latest_wall_time,
                    "ros_stamp": self._latest_ros_time,
                }

            sensors = self._latest_payload.get("available_sensors", [])
            names = [sensor.get("name") for sensor in sensors if isinstance(sensor, dict) and sensor.get("name")]
            mapped_sensors = []
            for sensor in sensors:
                if not isinstance(sensor, dict):
                    continue
                level_code = sensor.get("level")
                type_code = sensor.get("type")
                mapped_sensors.append(
                    {
                        "name": sensor.get("name"),
                        "topic": sensor.get("topic"),
                        "ready": sensor.get("ready"),
                        "rate": sensor.get("rate"),
                        "message": sensor.get("message"),
                        "level": level_code,
                        "level_label": SENSOR_LEVEL_MAP.get(level_code, "UNKNOWN"),
                        "type": type_code,
                        "type_label": SENSOR_TYPE_MAP.get(type_code, "UNKNOWN"),
                        "details": sensor.get("details", []),
                    }
                )
            return {
                "available": True,
                "count": len(sensors),
                "names": names,
                "sensors": sensors,
                "mapped_sensors": mapped_sensors,
                "received_at_unix": self._latest_wall_time,
                "ros_stamp": self._latest_ros_time,
            }


class ASGIApp:
    def __init__(self, bridge: SystemHealthBridge):
        self._bridge = bridge

    async def __call__(self, scope, receive, send) -> None:
        if scope["type"] != "http":
            return

        method = scope.get("method", "GET")
        path = scope.get("path", "/")

        if method == "OPTIONS":
            await self._send_json(send, 204, {})
            return

        if method != "GET":
            await self._send_json(send, 405, {"error": "Only GET is supported"})
            return

        if path == "/health":
            await self._send_json(send, 200, {"status": "ok"})
            return

        if path in ("/available_sensors", "/sensors"):
            await self._send_json(send, 200, self._bridge.get_available_sensors())
            return

        if path in ("/", "/system_health_info"):
            await self._send_json(send, 200, self._bridge.get_snapshot())
            return

        await self._send_json(send, 404, {"error": "Not found"})

    async def _send_json(self, send, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=True).encode("utf-8")
        headers = [
            (b"content-type", b"application/json"),
            (b"access-control-allow-origin", b"*"),
            (b"access-control-allow-methods", b"GET, OPTIONS"),
            (b"access-control-allow-headers", b"*"),
        ]
        await send({"type": "http.response.start", "status": status, "headers": headers})
        await send({"type": "http.response.body", "body": body})


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Expose ROS system_health_info topic over HTTP.")
    parser.add_argument("--robot-name", required=True, help="Robot namespace, e.g. uav1.")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind host.")
    parser.add_argument("--port", type=int, default=8081, help="HTTP bind port.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    topic_name = f"/{args.robot_name}/diagnostics_manager/system_health_info"

    rclpy.init()
    bridge = SystemHealthBridge(topic_name=topic_name)

    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = ASGIApp(bridge)

    try:
        import uvicorn
    except ImportError:
        bridge.get_logger().error(
            "uvicorn is required for system_health_http_enabled:=true. "
            "Install it with: pip install uvicorn"
        )
        executor.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()
        return 1

    try:
        uvicorn.run(app, host=args.host, port=args.port, log_level="info")
    finally:
        executor.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
