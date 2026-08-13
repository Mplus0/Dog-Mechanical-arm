#!/usr/bin/env python3
"""Persistent read-only RoArm-M3 ROS 2 serial driver.

This first hardware-validation version deliberately has no serial write path.
It opens the controller once, publishes fresh T=1051 feedback, and rejects all
commands so serial-open reset behaviour can be observed independently.
"""

import json
import math
import threading
import time
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from apriltag_block_grasp.core.roarm_serial_readonly import RoArmSerialStateReader


class RoArmDriverNode(Node):
    """Own the RoArm serial device for the lifetime of one ROS 2 node."""

    def __init__(self) -> None:
        super().__init__("apriltag_roarm_driver_node")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("serial_timeout_s", 0.2)
        self.declare_parameter("serial_settle_time_s", 2.0)
        self.declare_parameter("state_topic", "/roarm_m3/state")
        self.declare_parameter("command_topic", "/roarm_m3/cmd")
        self.declare_parameter("state_publish_period_s", 0.2)
        self.declare_parameter("state_stale_timeout_s", 1.0)

        self.port = str(self.get_parameter("port").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.state_stale_timeout_s = self._positive_parameter(
            "state_stale_timeout_s"
        )
        publish_period_s = self._positive_parameter("state_publish_period_s")
        settle_time_s = self._nonnegative_parameter("serial_settle_time_s")
        serial_timeout_s = self._positive_parameter("serial_timeout_s")
        baudrate = int(self.get_parameter("baudrate").value)
        if baudrate <= 0:
            raise ValueError("baudrate must be positive")

        self.reader = RoArmSerialStateReader(
            port=self.port,
            baudrate=baudrate,
            timeout_s=serial_timeout_s,
        )
        self.publisher = self.create_publisher(String, self.state_topic, 10)
        self.subscription = self.create_subscription(
            String, self.command_topic, self._on_command, 10
        )

        self._state_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._reader_thread: Optional[threading.Thread] = None
        self._latest_state: Optional[Dict[str, Any]] = None
        self._latest_state_monotonic: Optional[float] = None
        self._state_frame_count = 0
        self._empty_read_count = 0
        self._read_error: Optional[str] = None
        self._rejected_command_count = 0
        self._serial_open_count = 0
        self._serial_bytes_transmitted = 0
        self._started_monotonic = time.monotonic()

        self.get_logger().warning(
            "Opening the RoArm serial port once. The ESP32/OLED may reset during "
            "this initial open; no motion command will be transmitted."
        )
        self.reader.connect(settle_time_s=settle_time_s)
        self._serial_open_count = 1
        self.get_logger().info(
            "RoArm serial connection is persistent and read-only: "
            f"port={self.port}, command_topic={self.command_topic}, "
            f"state_topic={self.state_topic}."
        )

        self._reader_thread = threading.Thread(
            target=self._read_loop,
            name="apriltag-roarm-state-reader",
            daemon=True,
        )
        self._reader_thread.start()
        self.timer = self.create_timer(publish_period_s, self._publish_state)

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return value

    def _nonnegative_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and nonnegative")
        return value

    def _read_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                state = self.reader.read_state(timeout_s=0.5)
            except Exception as exc:
                with self._state_lock:
                    self._read_error = f"{type(exc).__name__}: {exc}"
                self.get_logger().error(
                    "RoArm serial read failed; automatic reconnect is disabled "
                    "to prevent another controller reset."
                )
                return
            if state is None:
                with self._state_lock:
                    self._empty_read_count += 1
                continue
            with self._state_lock:
                self._latest_state = dict(state)
                self._latest_state_monotonic = time.monotonic()
                self._state_frame_count += 1

    def _on_command(self, msg: String) -> None:
        self._rejected_command_count += 1
        command_type = None
        parse_error = None
        try:
            command = json.loads(msg.data)
            if isinstance(command, dict):
                command_type = command.get("type", command.get("T"))
        except Exception as exc:
            parse_error = f"{type(exc).__name__}: {exc}"
        self.get_logger().warning(
            "Rejected /roarm_m3/cmd because motion_commands_enabled=false: "
            f"command_type={command_type!r}, parse_error={parse_error!r}."
        )

    @staticmethod
    def _normalize_state(state: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        if state is None:
            return None
        output: Dict[str, Any] = {}
        for key, value in state.items():
            if isinstance(value, (int, float)):
                number = float(value)
                output[key] = number if math.isfinite(number) else None
            else:
                output[key] = value
        return output

    def _publish_state(self) -> None:
        now_monotonic = time.monotonic()
        with self._state_lock:
            latest_state = (
                dict(self._latest_state) if self._latest_state is not None else None
            )
            latest_monotonic = self._latest_state_monotonic
            state_frame_count = self._state_frame_count
            empty_read_count = self._empty_read_count
            read_error = self._read_error
        age_s = (
            None
            if latest_monotonic is None
            else max(0.0, now_monotonic - latest_monotonic)
        )
        state_valid = (
            latest_state is not None
            and age_s is not None
            and age_s <= self.state_stale_timeout_s
            and read_error is None
        )
        payload = {
            "stamp": time.time(),
            "connected": self.reader.connected,
            "state_valid": state_valid,
            "state_age_s": age_s,
            "state": self._normalize_state(latest_state),
            "driver": {
                "mode": "persistent_read_only_validation",
                "motion_commands_enabled": False,
                "automatic_reconnect_enabled": False,
                "serial_open_can_reset_controller": True,
                "serial_open_count": self._serial_open_count,
                "serial_bytes_transmitted": self._serial_bytes_transmitted,
                "state_frame_count": state_frame_count,
                "empty_read_count": empty_read_count,
                "rejected_command_count": self._rejected_command_count,
                "read_error": read_error,
                "uptime_s": max(0.0, now_monotonic - self._started_monotonic),
            },
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.publisher.publish(message)

    def destroy_node(self) -> None:
        self._stop_event.set()
        thread = self._reader_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self.reader.close()
        self.get_logger().info(
            "Persistent RoArm serial connection closed; transmitted_byte_count=0."
        )
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[RoArmDriverNode] = None
    try:
        node = RoArmDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
