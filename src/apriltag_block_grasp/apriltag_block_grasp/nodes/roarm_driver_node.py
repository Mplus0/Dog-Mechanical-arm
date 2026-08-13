#!/usr/bin/env python3
"""Persistent RoArm-M3 state driver with an explicitly gated B-only command.

The former gated T=1041 hold diagnostic is retained only for report/history
compatibility.  Hardware testing proved that copying stable T=1051 Cartesian
feedback into T=1041 is not an identity operation, so new attempts are rejected.
"""

from collections import deque
import json
import math
import threading
import time
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

from apriltag_block_grasp.core.b_joint_command import (
    BJointCommandLimits,
    validate_b_joint_request,
)
from apriltag_block_grasp.core.observation_motion import (
    OBSERVATION_COMMAND_TYPE,
    load_observation_motion_config,
    validate_observation_request,
)
from apriltag_block_grasp.core.roarm_serial_control import RoArmCartesianController


HOLD_TEST_COMMAND_TYPE = "diagnostic_hold_current_pose"
HOLD_TEST_CONFIRMATION = "I_ACCEPT_SINGLE_T1041_HOLD_TEST"
HOLD_TEST_HARDWARE_ENABLED = False


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
        self.declare_parameter("command_result_topic", "/roarm_m3/cmd_result")
        self.declare_parameter("state_publish_period_s", 0.2)
        self.declare_parameter("state_stale_timeout_s", 1.0)
        self.declare_parameter("enable_b_joint_motion", False)
        self.declare_parameter("enable_observation_motion", False)
        self.declare_parameter("observation_max_state_age_s", 0.25)
        share = Path(get_package_share_directory("apriltag_block_grasp"))
        self.declare_parameter(
            "motion_config_path", str(share / "config" / "motion_control.json")
        )
        self.declare_parameter("b_joint_min_deg", -20.0)
        self.declare_parameter("b_joint_max_deg", 20.0)
        self.declare_parameter("b_joint_max_delta_deg", 10.0)
        self.declare_parameter("b_joint_max_speed_deg_s", 35.0)
        self.declare_parameter("b_joint_max_acceleration", 35.0)
        self.declare_parameter("b_joint_max_state_age_s", 0.25)
        self.declare_parameter("hold_test_report_topic", "/apriltag_grasp/hold_test_report")
        self.declare_parameter("enable_diagnostic_hold_test", False)
        self.declare_parameter("hold_test_min_uptime_s", 5.0)
        self.declare_parameter("hold_test_max_state_age_s", 0.25)
        self.declare_parameter("hold_test_stability_window_s", 1.0)
        self.declare_parameter("hold_test_min_stability_samples", 10)
        self.declare_parameter("hold_test_max_xyz_peak_to_peak_mm", 1.0)
        self.declare_parameter("hold_test_max_angle_peak_to_peak_rad", 0.01)
        self.declare_parameter("hold_test_trace_duration_s", 12.0)
        self.declare_parameter("hold_test_trace_interval_s", 0.25)

        self.port = str(self.get_parameter("port").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.command_result_topic = str(
            self.get_parameter("command_result_topic").value
        )
        self.enable_b_joint_motion = bool(
            self.get_parameter("enable_b_joint_motion").value
        )
        self.enable_observation_motion = bool(
            self.get_parameter("enable_observation_motion").value
        )
        self.observation_motion_config = load_observation_motion_config(
            str(self.get_parameter("motion_config_path").value)
        )
        self.observation_max_state_age_s = self._positive_parameter(
            "observation_max_state_age_s"
        )
        self.b_joint_limits = BJointCommandLimits(
            minimum_deg=float(self.get_parameter("b_joint_min_deg").value),
            maximum_deg=float(self.get_parameter("b_joint_max_deg").value),
            maximum_delta_deg=float(
                self.get_parameter("b_joint_max_delta_deg").value
            ),
            maximum_speed_deg_s=float(
                self.get_parameter("b_joint_max_speed_deg_s").value
            ),
            maximum_acceleration=float(
                self.get_parameter("b_joint_max_acceleration").value
            ),
        )
        self.b_joint_max_state_age_s = self._positive_parameter(
            "b_joint_max_state_age_s"
        )
        self.hold_test_report_topic = str(
            self.get_parameter("hold_test_report_topic").value
        )
        self.enable_diagnostic_hold_test = bool(
            self.get_parameter("enable_diagnostic_hold_test").value
        )
        self.state_stale_timeout_s = self._positive_parameter(
            "state_stale_timeout_s"
        )
        self.hold_test_min_uptime_s = self._positive_parameter(
            "hold_test_min_uptime_s"
        )
        self.hold_test_max_state_age_s = self._positive_parameter(
            "hold_test_max_state_age_s"
        )
        self.hold_test_stability_window_s = self._positive_parameter(
            "hold_test_stability_window_s"
        )
        self.hold_test_min_stability_samples = int(
            self.get_parameter("hold_test_min_stability_samples").value
        )
        if self.hold_test_min_stability_samples < 2:
            raise ValueError("hold_test_min_stability_samples must be at least 2")
        self.hold_test_max_xyz_p2p_mm = self._positive_parameter(
            "hold_test_max_xyz_peak_to_peak_mm"
        )
        self.hold_test_max_angle_p2p_rad = self._positive_parameter(
            "hold_test_max_angle_peak_to_peak_rad"
        )
        self.hold_test_trace_duration_s = self._positive_parameter(
            "hold_test_trace_duration_s"
        )
        self.hold_test_trace_interval_s = self._positive_parameter(
            "hold_test_trace_interval_s"
        )
        publish_period_s = self._positive_parameter("state_publish_period_s")
        settle_time_s = self._nonnegative_parameter("serial_settle_time_s")
        serial_timeout_s = self._positive_parameter("serial_timeout_s")
        baudrate = int(self.get_parameter("baudrate").value)
        if baudrate <= 0:
            raise ValueError("baudrate must be positive")

        self.reader = RoArmCartesianController(
            port=self.port,
            baudrate=baudrate,
            timeout_s=serial_timeout_s,
        )
        self.publisher = self.create_publisher(String, self.state_topic, 10)
        self.command_result_publisher = self.create_publisher(
            String, self.command_result_topic, 10
        )
        self.hold_test_report_publisher = self.create_publisher(
            String, self.hold_test_report_topic, 10
        )
        self.subscription = self.create_subscription(
            String, self.command_topic, self._on_command, 10
        )

        self._state_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._reader_thread: Optional[threading.Thread] = None
        self._latest_state: Optional[Dict[str, Any]] = None
        self._latest_state_monotonic: Optional[float] = None
        self._state_history: Deque[Tuple[float, Dict[str, Any]]] = deque(maxlen=500)
        self._state_frame_count = 0
        self._empty_read_count = 0
        self._read_error: Optional[str] = None
        self._rejected_command_count = 0
        self._accepted_b_joint_command_count = 0
        self._accepted_observation_command_count = 0
        self._last_command_result: Optional[Dict[str, Any]] = None
        self._command_lock = threading.Lock()
        self._serial_open_count = 0
        self._serial_bytes_transmitted = 0
        self._hold_test_attempted = False
        self._hold_test_command_sent = False
        self._hold_test: Optional[Dict[str, Any]] = None
        self._hold_test_report: Optional[Dict[str, Any]] = None
        self._last_hold_report_publish_monotonic = 0.0
        self._started_monotonic = time.monotonic()

        self.get_logger().warning(
            "Opening the RoArm serial port once. The ESP32/OLED may reset during "
            "this initial open; no motion command is sent automatically."
        )
        self.reader.connect(settle_time_s=settle_time_s)
        self._serial_open_count = 1
        self.get_logger().info(
            "RoArm serial connection is persistent: "
            f"port={self.port}, command_topic={self.command_topic}, "
            f"state_topic={self.state_topic}, "
            f"B_joint_motion_enabled={self.enable_b_joint_motion}."
        )
        if self.enable_b_joint_motion:
            self.get_logger().warning(
                "B-JOINT MOTION IS ENABLED: only absolute joint=1 commands within "
                "the configured range and delta limits are accepted."
            )
        if self.enable_observation_motion:
            self.get_logger().warning(
                "FIXED OBSERVATION MOTION IS ENABLED: only the configured "
                "B/T/R/S/E sequence is accepted; the gripper is never commanded."
            )
        if self.enable_diagnostic_hold_test:
            self.get_logger().warning(
                "Single hold-current-pose diagnostic is ARMED. It still requires "
                f"type={HOLD_TEST_COMMAND_TYPE!r} and the exact confirmation token."
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
                self._state_history.append(
                    (self._latest_state_monotonic, dict(state))
                )
                self._capture_hold_test_sample_locked(
                    self._latest_state_monotonic, state
                )

    def _on_command(self, msg: String) -> None:
        command: Any = None
        try:
            command = json.loads(msg.data)
        except Exception as exc:
            self._reject_command(None, f"invalid_json: {type(exc).__name__}: {exc}")
            return
        if not isinstance(command, dict):
            self._reject_command(None, "command_must_be_json_object")
            return
        command_type = command.get("type")
        if command_type == OBSERVATION_COMMAND_TYPE:
            self._handle_observation_command(command)
            return
        if command_type == "move_b_joint":
            self._handle_b_joint_command(command)
            return
        if command_type != HOLD_TEST_COMMAND_TYPE:
            self._reject_command(command_type, "unsupported_command_type")
            return
        if not HOLD_TEST_HARDWARE_ENABLED:
            self._reject_command(
                command_type,
                "diagnostic_hold_test_retired_after_non_identity_hardware_result",
            )
            return
        if not self.enable_diagnostic_hold_test:
            self._reject_command(command_type, "diagnostic_hold_test_not_enabled")
            return
        if command.get("confirmation") != HOLD_TEST_CONFIRMATION:
            self._reject_command(command_type, "confirmation_token_mismatch")
            return
        if self._hold_test_attempted:
            self._reject_command(command_type, "hold_test_already_attempted")
            return

        self._hold_test_attempted = True
        preflight = self._hold_test_preflight()
        if not preflight["valid"]:
            self._hold_test_report = {
                "tool": "apriltag_block_grasp.roarm_driver_hold_test",
                "valid": False,
                "reason": "preflight_rejected",
                "preflight": preflight,
                "motion_command_sent": False,
            }
            self._publish_hold_test_report(force=True)
            self.get_logger().error(
                "Hold-current-pose diagnostic rejected by preflight; restart "
                "the node before any new attempt."
            )
            return

        initial_state = dict(preflight["selected_state"])
        try:
            sent_command = self.reader.send_cartesian_command(
                x_mm=float(initial_state["x"]),
                y_mm=float(initial_state["y"]),
                z_mm=float(initial_state["z"]),
                pitch_rad=float(initial_state["tit"]),
                roll_rad=float(initial_state["r"]),
                gripper_rad=float(initial_state["g"]),
            )
        except Exception as exc:
            self._serial_bytes_transmitted = self.reader.transmitted_byte_count
            self._hold_test_report = {
                "tool": "apriltag_block_grasp.roarm_driver_hold_test",
                "valid": False,
                "reason": "serial_send_failed",
                "preflight": preflight,
                "error": f"{type(exc).__name__}: {exc}",
                "motion_command_sent": self._serial_bytes_transmitted > 0,
                "serial_bytes_transmitted": self._serial_bytes_transmitted,
            }
            self._publish_hold_test_report(force=True)
            return

        sent_monotonic = time.monotonic()
        self._serial_bytes_transmitted = self.reader.transmitted_byte_count
        self._hold_test_command_sent = True
        with self._state_lock:
            self._hold_test = {
                "sent_monotonic": sent_monotonic,
                "initial_state": initial_state,
                "sent_command": sent_command,
                "preflight": preflight,
                "trace": [
                    {
                        "elapsed_s": 0.0,
                        "state": self._compact_pose_state(initial_state),
                    }
                ],
                "last_trace_monotonic": sent_monotonic,
            }
        self.get_logger().warning(
            "Sent exactly one T=1041 hold-current-pose diagnostic command; "
            "all further commands remain disabled."
        )

    def _reject_command(self, command_type: Any, reason: str) -> None:
        self._rejected_command_count += 1
        self.get_logger().warning(
            f"Rejected /roarm_m3/cmd: command_type={command_type!r}, reason={reason}."
        )

    def _publish_command_result(self, payload: Dict[str, Any]) -> None:
        result = {"stamp": time.time(), **payload}
        self._last_command_result = result
        message = String()
        message.data = json.dumps(result, ensure_ascii=False, separators=(",", ":"))
        self.command_result_publisher.publish(message)

    def _reject_b_joint_command(self, command: Dict[str, Any], reason: str) -> None:
        self._reject_command(command.get("type"), reason)
        self._publish_command_result(
            {
                "command_id": command.get("command_id"),
                "type": "move_b_joint",
                "accepted": False,
                "reason": reason,
                "motion_command_sent": False,
            }
        )

    def _reject_observation_command(
        self, command: Dict[str, Any], reason: str
    ) -> None:
        self._reject_command(command.get("type"), reason)
        self._publish_command_result(
            {
                "command_id": command.get("command_id"),
                "type": OBSERVATION_COMMAND_TYPE,
                "accepted": False,
                "reason": reason,
                "motion_command_sent": False,
                "gripper_commanded": False,
            }
        )

    def _handle_observation_command(self, command: Dict[str, Any]) -> None:
        if not self.enable_observation_motion:
            self._reject_observation_command(
                command, "observation_motion_not_enabled"
            )
            return
        try:
            command_id = validate_observation_request(command)
        except Exception as exc:
            self._reject_observation_command(
                command, f"validation_failed:{type(exc).__name__}:{exc}"
            )
            return

        now = time.monotonic()
        with self._state_lock:
            latest_state = self._latest_state
            latest_time = self._latest_state_monotonic
            read_error = self._read_error
        if read_error is not None:
            self._reject_observation_command(command, "serial_read_error")
            return
        if latest_state is None or latest_time is None:
            self._reject_observation_command(command, "arm_state_unavailable")
            return
        state_age_s = max(0.0, now - latest_time)
        if state_age_s > self.observation_max_state_age_s:
            self._reject_observation_command(command, "arm_state_stale")
            return

        sent_commands: List[Dict[str, Any]] = []
        try:
            commands = self.observation_motion_config.serial_commands()
            with self._command_lock:
                for index, item in enumerate(commands):
                    sent_commands.append(
                        self.reader.send_joint_command(
                            item["joint"], item["angle"], item["spd"], item["acc"]
                        )
                    )
                    if (
                        index + 1 < len(commands)
                        and self.observation_motion_config.command_interval_s > 0.0
                    ):
                        time.sleep(
                            self.observation_motion_config.command_interval_s
                        )
        except Exception as exc:
            self._serial_bytes_transmitted = self.reader.transmitted_byte_count
            self._publish_command_result(
                {
                    "command_id": command_id,
                    "type": OBSERVATION_COMMAND_TYPE,
                    "accepted": False,
                    "reason": f"serial_send_failed:{type(exc).__name__}:{exc}",
                    "motion_command_sent": bool(sent_commands),
                    "gripper_commanded": False,
                    "sent_commands": sent_commands,
                }
            )
            return

        self._accepted_observation_command_count += 1
        self._serial_bytes_transmitted = self.reader.transmitted_byte_count
        time.sleep(self.observation_motion_config.timed_wait_s)
        self._publish_command_result(
            {
                "command_id": command_id,
                "type": OBSERVATION_COMMAND_TYPE,
                "accepted": True,
                "reason": "timed_wait_complete",
                "motion_command_sent": True,
                "gripper_commanded": False,
                "completion_mode": "timed",
                "timed_wait_s": self.observation_motion_config.timed_wait_s,
                "state_age_s_at_send": state_age_s,
                "sent_commands": sent_commands,
            }
        )

    def _handle_b_joint_command(self, command: Dict[str, Any]) -> None:
        if not self.enable_b_joint_motion:
            self._reject_b_joint_command(command, "b_joint_motion_not_enabled")
            return
        now = time.monotonic()
        with self._state_lock:
            latest = None if self._latest_state is None else dict(self._latest_state)
            latest_time = self._latest_state_monotonic
            read_error = self._read_error
        if read_error is not None:
            self._reject_b_joint_command(command, "serial_read_error")
            return
        if latest is None or latest_time is None:
            self._reject_b_joint_command(command, "arm_state_unavailable")
            return
        state_age_s = max(0.0, now - latest_time)
        if state_age_s > self.b_joint_max_state_age_s:
            self._reject_b_joint_command(command, "arm_state_stale")
            return
        try:
            current_b_deg = math.degrees(self._finite_state_value(latest, "b"))
            validated = validate_b_joint_request(
                command, current_b_deg, self.b_joint_limits
            )
        except Exception as exc:
            self._reject_b_joint_command(
                command, f"validation_failed:{type(exc).__name__}:{exc}"
            )
            return

        try:
            with self._command_lock:
                sent = self.reader.send_b_joint_command(
                    validated["target_b_deg"],
                    validated["speed_deg_s"],
                    validated["acceleration"],
                )
        except Exception as exc:
            self._serial_bytes_transmitted = self.reader.transmitted_byte_count
            self._publish_command_result(
                {
                    "command_id": command.get("command_id"),
                    "type": "move_b_joint",
                    "accepted": False,
                    "reason": f"serial_send_failed:{type(exc).__name__}:{exc}",
                    "motion_command_sent": self._serial_bytes_transmitted > 0,
                }
            )
            return

        self._accepted_b_joint_command_count += 1
        self._serial_bytes_transmitted = self.reader.transmitted_byte_count
        self._publish_command_result(
            {
                "command_id": command.get("command_id"),
                "type": "move_b_joint",
                "accepted": True,
                "reason": "command_sent",
                "motion_command_sent": True,
                "state_age_s_at_send": state_age_s,
                "current_b_deg_at_send": validated["current_b_deg"],
                "target_b_deg": validated["target_b_deg"],
                "requested_delta_deg": validated["requested_delta_deg"],
                "sent_command": sent,
            }
        )

    @staticmethod
    def _finite_state_value(state: Dict[str, Any], key: str) -> float:
        if key not in state:
            raise KeyError(f"state is missing {key!r}")
        value = float(state[key])
        if not math.isfinite(value):
            raise ValueError(f"state.{key} must be finite")
        return value

    @classmethod
    def _compact_pose_state(cls, state: Dict[str, Any]) -> Dict[str, float]:
        return {
            key: cls._finite_state_value(state, key)
            for key in ("x", "y", "z", "tit", "b", "s", "e", "t", "r", "g")
            if key in state
        }

    def _hold_test_preflight(self) -> Dict[str, Any]:
        now = time.monotonic()
        uptime_s = now - self._started_monotonic
        with self._state_lock:
            latest = None if self._latest_state is None else dict(self._latest_state)
            latest_time = self._latest_state_monotonic
            history = [
                dict(state)
                for timestamp, state in self._state_history
                if timestamp >= now - self.hold_test_stability_window_s
            ]
            read_error = self._read_error
        reasons: List[str] = []
        age_s = None if latest_time is None else max(0.0, now - latest_time)
        if uptime_s < self.hold_test_min_uptime_s:
            reasons.append("driver_uptime_too_short")
        if read_error is not None:
            reasons.append("serial_read_error")
        if latest is None or age_s is None:
            reasons.append("no_current_state")
        elif age_s > self.hold_test_max_state_age_s:
            reasons.append("current_state_stale")
        if len(history) < self.hold_test_min_stability_samples:
            reasons.append("insufficient_stability_samples")

        required_keys = ("x", "y", "z", "tit", "r", "g")
        statistics: Dict[str, Any] = {}
        try:
            if latest is not None:
                for key in required_keys:
                    self._finite_state_value(latest, key)
            for key in required_keys:
                values = [self._finite_state_value(state, key) for state in history]
                if not values:
                    continue
                peak_to_peak = max(values) - min(values)
                limit = (
                    self.hold_test_max_xyz_p2p_mm
                    if key in ("x", "y", "z")
                    else self.hold_test_max_angle_p2p_rad
                )
                statistics[key] = {
                    "count": len(values),
                    "min": min(values),
                    "max": max(values),
                    "peak_to_peak": peak_to_peak,
                    "limit": limit,
                }
                if peak_to_peak > limit:
                    reasons.append(f"unstable_{key}")
        except (KeyError, TypeError, ValueError) as exc:
            reasons.append(f"invalid_state: {type(exc).__name__}: {exc}")

        return {
            "valid": not reasons,
            "reasons": reasons,
            "driver_uptime_s": uptime_s,
            "minimum_driver_uptime_s": self.hold_test_min_uptime_s,
            "state_age_s": age_s,
            "maximum_state_age_s": self.hold_test_max_state_age_s,
            "stability_window_s": self.hold_test_stability_window_s,
            "stability_sample_count": len(history),
            "minimum_stability_sample_count": self.hold_test_min_stability_samples,
            "stability": statistics,
            "selected_state": latest,
        }

    def _capture_hold_test_sample_locked(
        self, timestamp: float, state: Dict[str, Any]
    ) -> None:
        test = self._hold_test
        if test is None or self._hold_test_report is not None:
            return
        elapsed_s = timestamp - float(test["sent_monotonic"])
        if elapsed_s < 0.0 or elapsed_s > self.hold_test_trace_duration_s:
            return
        if (
            timestamp - float(test["last_trace_monotonic"])
            < self.hold_test_trace_interval_s
        ):
            return
        test["trace"].append(
            {"elapsed_s": elapsed_s, "state": self._compact_pose_state(state)}
        )
        test["last_trace_monotonic"] = timestamp

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

    def _finalize_hold_test_if_due(self, now_monotonic: float) -> None:
        with self._state_lock:
            test = self._hold_test
            if test is None or self._hold_test_report is not None:
                return
            elapsed_s = now_monotonic - float(test["sent_monotonic"])
            if elapsed_s < self.hold_test_trace_duration_s:
                return
            trace = list(test["trace"])
            initial_state = dict(test["initial_state"])
            sent_command = dict(test["sent_command"])
            preflight = dict(test["preflight"])
        pose_keys = ("x", "y", "z", "tit", "r", "g")
        initial_pose = self._compact_pose_state(initial_state)
        final_pose = None if not trace else dict(trace[-1]["state"])
        deltas: Optional[Dict[str, float]] = None
        extrema: Dict[str, Any] = {}
        if final_pose is not None:
            deltas = {
                key: float(final_pose[key]) - float(initial_pose[key])
                for key in pose_keys
            }
            for key in pose_keys:
                values = [float(item["state"][key]) for item in trace]
                extrema[key] = {
                    "min": min(values),
                    "max": max(values),
                    "peak_to_peak": max(values) - min(values),
                    "maximum_absolute_delta_from_initial": max(
                        abs(value - float(initial_pose[key])) for value in values
                    ),
                }
        report = {
            "tool": "apriltag_block_grasp.roarm_driver_hold_test",
            "valid": bool(trace),
            "reason": "trace_complete" if trace else "no_feedback_after_command",
            "test_definition": (
                "one T=1041 target copied exactly from fresh stable T=1051 "
                "x/y/z/tit/r/g feedback on the same persistent serial connection"
            ),
            "user_supplied_pose_accepted": False,
            "motion_command_sent": True,
            "transmitted_command_count": self.reader.transmitted_command_count,
            "serial_bytes_transmitted": self.reader.transmitted_byte_count,
            "preflight": preflight,
            "initial_state": initial_pose,
            "sent_command": sent_command,
            "trace_duration_s": self.hold_test_trace_duration_s,
            "trace_sample_count": len(trace),
            "trace": trace,
            "final_state": final_pose,
            "final_delta_from_initial": deltas,
            "trace_extrema": extrema,
        }
        with self._state_lock:
            if self._hold_test_report is None:
                self._hold_test_report = report
        self.get_logger().warning(
            "Hold-current-pose diagnostic trace is complete; inspect "
            f"{self.hold_test_report_topic}."
        )

    def _publish_hold_test_report(self, force: bool = False) -> None:
        with self._state_lock:
            report = (
                None
                if self._hold_test_report is None
                else dict(self._hold_test_report)
            )
        if report is None:
            return
        now = time.monotonic()
        if not force and now - self._last_hold_report_publish_monotonic < 1.0:
            return
        message = String()
        message.data = json.dumps(report, ensure_ascii=False, separators=(",", ":"))
        self.hold_test_report_publisher.publish(message)
        self._last_hold_report_publish_monotonic = now

    def _publish_state(self) -> None:
        now_monotonic = time.monotonic()
        self._finalize_hold_test_if_due(now_monotonic)
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
                "mode": (
                    "persistent_b_and_observation_motion_enabled"
                    if self.enable_b_joint_motion and self.enable_observation_motion
                    else (
                        "persistent_b_joint_motion_enabled"
                        if self.enable_b_joint_motion
                        else (
                            "persistent_observation_motion_enabled"
                            if self.enable_observation_motion
                            else "persistent_read_only_validation"
                        )
                    )
                ),
                "motion_commands_enabled": (
                    self.enable_b_joint_motion or self.enable_observation_motion
                ),
                "b_joint_motion_enabled": self.enable_b_joint_motion,
                "observation_motion_enabled": self.enable_observation_motion,
                "other_joint_motion_enabled": self.enable_observation_motion,
                "accepted_b_joint_command_count": self._accepted_b_joint_command_count,
                "accepted_observation_command_count": (
                    self._accepted_observation_command_count
                ),
                "last_command_result": self._last_command_result,
                "diagnostic_hold_test_enabled": self.enable_diagnostic_hold_test,
                "diagnostic_hold_test_attempted": self._hold_test_attempted,
                "diagnostic_hold_test_command_sent": self._hold_test_command_sent,
                "diagnostic_hold_test_report_ready": self._hold_test_report is not None,
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
        self._publish_hold_test_report()

    def destroy_node(self) -> None:
        self._stop_event.set()
        thread = self._reader_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self.reader.close()
        self.get_logger().info(
            "Persistent RoArm serial connection closed; "
            f"transmitted_command_count={self.reader.transmitted_command_count}, "
            f"transmitted_byte_count={self.reader.transmitted_byte_count}."
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
