#!/usr/bin/env python3
"""Command-driven localization with explicitly gated B-only target search."""

import json
import math
import time
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

from apriltag_block_grasp.core.b_search import load_b_search_config
from apriltag_block_grasp.core.localization_task import LocalizationTaskSession
from apriltag_block_grasp.core.target_lock import (
    StableTargetLock,
    load_target_stability_config,
)


class ManipulationTaskNode(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_manipulation_task_node")
        share = Path(get_package_share_directory("apriltag_block_grasp"))
        self.declare_parameter("task_cmd_topic", "/apriltag_grasp/task_cmd")
        self.declare_parameter("task_state_topic", "/apriltag_grasp/task_state")
        self.declare_parameter("task_result_topic", "/apriltag_grasp/task_result")
        self.declare_parameter("candidate_topic", "/apriltag_grasp/target_candidates")
        self.declare_parameter("arm_state_topic", "/roarm_m3/state")
        self.declare_parameter("arm_command_topic", "/roarm_m3/cmd")
        self.declare_parameter("arm_command_result_topic", "/roarm_m3/cmd_result")
        self.declare_parameter("enable_b_search_motion", False)
        self.declare_parameter(
            "stability_config_path", str(share / "config" / "target_stability.json")
        )
        self.declare_parameter(
            "motion_config_path", str(share / "config" / "motion_control.json")
        )
        self.declare_parameter("timeout_check_period_s", 0.05)

        self.task_cmd_topic = str(self.get_parameter("task_cmd_topic").value)
        self.task_state_topic = str(self.get_parameter("task_state_topic").value)
        self.task_result_topic = str(self.get_parameter("task_result_topic").value)
        self.candidate_topic = str(self.get_parameter("candidate_topic").value)
        self.arm_state_topic = str(self.get_parameter("arm_state_topic").value)
        self.arm_command_topic = str(self.get_parameter("arm_command_topic").value)
        self.arm_command_result_topic = str(
            self.get_parameter("arm_command_result_topic").value
        )
        self.enable_b_search_motion = bool(
            self.get_parameter("enable_b_search_motion").value
        )
        timeout_check_period_s = float(
            self.get_parameter("timeout_check_period_s").value
        )
        if timeout_check_period_s <= 0.0:
            raise ValueError("timeout_check_period_s must be positive")

        config = load_target_stability_config(
            str(self.get_parameter("stability_config_path").value)
        )
        self.b_search_config = load_b_search_config(
            str(self.get_parameter("motion_config_path").value)
        )
        self.session = LocalizationTaskSession(StableTargetLock(config))
        self.state_publisher = self.create_publisher(String, self.task_state_topic, 10)
        self.result_publisher = self.create_publisher(String, self.task_result_topic, 10)
        self.arm_command_publisher = self.create_publisher(
            String, self.arm_command_topic, 10
        )
        self.command_subscription = self.create_subscription(
            String, self.task_cmd_topic, self.on_task_command, 10
        )
        self.candidate_subscription = self.create_subscription(
            String, self.candidate_topic, self.on_candidates, 10
        )
        self.arm_state_subscription = self.create_subscription(
            String, self.arm_state_topic, self.on_arm_state, 10
        )
        self.arm_command_result_subscription = self.create_subscription(
            String, self.arm_command_result_topic, self.on_arm_command_result, 10
        )
        self.latest_arm_state_payload: Optional[Dict[str, Any]] = None
        self.latest_arm_state_received_monotonic: Optional[float] = None
        self.cycle_observation_b_deg: Optional[float] = None
        self.search_index = 0
        self.search_saw_target = False
        self.search_route = []
        self.route_final_search_index: Optional[int] = None
        self.route_returns_to_b0 = False
        self.pending_b_target_deg: Optional[float] = None
        self.pending_b_command_id: Optional[str] = None
        self.pending_b_sent_monotonic: Optional[float] = None
        self.pending_b_command_accepted = False
        self.b_arrival_stable_count = 0
        self.b_command_sequence = 0
        self.pending_terminal_b_failure: Optional[Dict[str, Any]] = None
        self.timeout_timer = self.create_timer(
            timeout_check_period_s, self.on_timeout_check
        )
        self.publish_state("idle", "ready")
        self.get_logger().info(
            "Manipulation task node started: command-driven localization; "
            f"B_search_motion_enabled={self.enable_b_search_motion}; "
            "observation motion, gripper and grasp motions remain disabled."
        )

    @staticmethod
    def parse_json(message: String) -> Dict[str, Any]:
        data = json.loads(message.data)
        if not isinstance(data, dict):
            raise ValueError("message must contain a JSON object")
        return data

    def safety_fields(self) -> Dict[str, Any]:
        return {
            "picked_ids": [],
            "placed_ids": [],
            "carrying_id": None,
            "target_snapshot_only": True,
            "pick_motion_executed": False,
            "observation_motion_enabled": False,
            "b_search_enabled": self.enable_b_search_motion,
            "gripper_commands_enabled": False,
            "motion_commands_enabled": self.enable_b_search_motion,
            "motion_scope": "B_joint_search_only" if self.enable_b_search_motion else "none",
            "b_search_maximum_index": (
                self.b_search_config.maximum_automatic_search_index
                if self.enable_b_search_motion
                else None
            ),
        }

    def publish_json(self, publisher, payload: Dict[str, Any]) -> None:
        output = String()
        output.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        publisher.publish(output)

    def publish_state(
        self, state: str, reason: str, extra: Optional[Dict[str, Any]] = None
    ) -> None:
        payload = {
            "stamp": time.time(),
            **self.session.state_payload(),
            "state": state,
            "reason": reason,
            **self.safety_fields(),
            "cycle_observation_b_deg": self.cycle_observation_b_deg,
            "b_search_offset_deg": self.current_search_offset_deg(),
            "b_search_index": self.search_index,
        }
        if extra:
            payload.update(extra)
        self.publish_json(self.state_publisher, payload)

    def publish_result(
        self,
        task_id: Any,
        result: str,
        reason: str,
        extra: Optional[Dict[str, Any]] = None,
    ) -> None:
        payload = {
            "stamp": time.time(),
            "task_id": task_id,
            "cmd": "pick",
            "result": result,
            "reason": reason,
            **self.safety_fields(),
            "cycle_observation_b_deg": self.cycle_observation_b_deg,
            "b_search_offset_deg": self.current_search_offset_deg(),
        }
        if extra:
            payload.update(extra)
        self.publish_json(self.result_publisher, payload)

    def on_task_command(self, message: String) -> None:
        try:
            data = self.parse_json(message)
        except Exception as exc:
            self.publish_result(
                None,
                "task_rejected",
                f"invalid_json:{type(exc).__name__}:{exc}",
            )
            return

        decision = self.session.accept_command(data, time.monotonic())
        if decision.action == "accepted":
            if self.enable_b_search_motion:
                if not self.prepare_b_search_for_accepted_task():
                    task_id = self.session.active_task_id
                    self.publish_result(task_id, "task_rejected", "b_search_preflight_failed")
                    self.session.finish_terminal()
                    return
            self.publish_state("localizing", decision.reason)
        elif decision.action in ("resume", "replace"):
            if self.enable_b_search_motion:
                self.reset_search_attempt_preserving_b0()
            self.publish_state("localizing", decision.reason)
        elif decision.action == "ignore":
            self.publish_state(
                self.session.state,
                decision.reason,
                {"ignored_task_id": decision.task_id},
            )
        elif decision.action == "busy":
            self.publish_result(decision.task_id, "task_rejected", decision.reason)
        else:
            self.publish_result(decision.task_id, "task_rejected", decision.reason)

    def on_candidates(self, message: String) -> None:
        if not self.session.active or self.session.state != "localizing":
            return
        try:
            payload = self.parse_json(message)
        except Exception as exc:
            self.publish_state(
                "localizing", f"invalid_candidate_json:{type(exc).__name__}:{exc}"
            )
            return
        result = self.session.update_candidates(payload, time.monotonic())
        if result is not None:
            self.handle_localization_result(result)

    def on_timeout_check(self) -> None:
        if not self.session.active or self.session.state == "reposition_required":
            return
        if self.session.active and self.session.state == "waiting_b_motion":
            self.check_b_motion_timeout()
            return
        if self.session.state != "localizing":
            return
        result = self.session.check_timeout(time.monotonic())
        if result is not None:
            self.handle_localization_result(result)

    def handle_localization_result(self, result: Dict[str, Any]) -> None:
        status = str(result.get("status", ""))
        if status == "stable":
            task_id = self.session.active_task_id
            snapshot = {
                "selected_tag_id": result.get("locked_id"),
                "base_object_median_mm": result.get("base_object_median_mm"),
                "xyz_peak_to_peak_mm": result.get("xyz_peak_to_peak_mm"),
                "collected_frame_count": result.get("collected_frame_count"),
                "sample_pnp_stamp_first": result.get("sample_pnp_stamp_first"),
                "sample_pnp_stamp_last": result.get("sample_pnp_stamp_last"),
                "cycle_observation_b_deg": self.cycle_observation_b_deg,
                "b_search_offset_deg": self.current_search_offset_deg(),
            }
            self.publish_state("snapshot_ready", "stable_target_ready", snapshot)
            self.publish_result(
                task_id, "target_snapshot_ready", "stable_target_ready", snapshot
            )
            self.session.finish_terminal()
            return

        if status == "failed":
            if self.enable_b_search_motion:
                self.search_saw_target = self.search_saw_target or bool(
                    result.get("ever_saw_allowed_target", False)
                )
                if self.advance_b_search():
                    return
                self.start_return_to_b0()
                return
            task_id = self.session.active_task_id
            diagnostics = {
                key: result[key]
                for key in (
                    "locked_id",
                    "collected_frame_count",
                    "window_reset_count",
                    "failure_detail",
                    "last_xyz_peak_to_peak_mm",
                    "best_xyz_peak_to_peak_mm",
                    "best_max_threshold_ratio",
                    "threshold_exceeded_axes",
                )
                if key in result
            }
            reason = str(result.get("reason", "localization_failed"))
            self.publish_state("localization_failed", reason, diagnostics)
            self.publish_result(task_id, "localization_failed", reason, diagnostics)
            self.session.finish_terminal()
            return

        self.publish_state("localizing", str(result.get("reason", "collecting")), result)

    def current_search_offset_deg(self) -> Optional[float]:
        if self.cycle_observation_b_deg is None:
            return None
        return self.b_search_config.offsets_deg[self.search_index]

    def on_arm_state(self, message: String) -> None:
        try:
            payload = self.parse_json(message)
        except Exception:
            return
        self.latest_arm_state_payload = payload
        self.latest_arm_state_received_monotonic = time.monotonic()
        if self.session.active and self.session.state == "waiting_b_motion":
            self.update_b_arrival(payload)

    def latest_b_deg_for_preflight(self) -> Optional[float]:
        payload = self.latest_arm_state_payload
        received = self.latest_arm_state_received_monotonic
        if payload is None or received is None or not bool(payload.get("state_valid", False)):
            return None
        if time.monotonic() - received > 0.5:
            return None
        state = payload.get("state")
        driver = payload.get("driver")
        if not isinstance(state, dict) or not isinstance(driver, dict):
            return None
        if not bool(driver.get("b_joint_motion_enabled", False)):
            return None
        try:
            b_deg = math.degrees(float(state["b"]))
        except (KeyError, TypeError, ValueError):
            return None
        return b_deg if math.isfinite(b_deg) else None

    def initialize_b_search(self) -> bool:
        b0 = self.latest_b_deg_for_preflight()
        if b0 is None:
            self.get_logger().error("B-search preflight failed: no fresh enabled B state")
            return False
        try:
            self.b_search_config.absolute_targets(b0)
        except ValueError as exc:
            self.get_logger().error(f"B-search preflight failed: {exc}")
            return False
        self.cycle_observation_b_deg = b0
        self.reset_search_attempt_preserving_b0()
        return True

    def prepare_b_search_for_accepted_task(self) -> bool:
        """Capture B0 once per node-lifetime task cycle, then preserve it."""
        if self.cycle_observation_b_deg is None:
            return self.initialize_b_search()

        current_b_deg = self.latest_b_deg_for_preflight()
        if current_b_deg is None:
            self.get_logger().error(
                "B-search preflight failed: no fresh enabled B state"
            )
            return False
        if (
            abs(current_b_deg - self.cycle_observation_b_deg)
            > self.b_search_config.arrival_tolerance_deg
        ):
            self.get_logger().error(
                "B-search preflight failed: current B is not at the preserved "
                f"cycle B0 (current={current_b_deg:.3f}, "
                f"B0={self.cycle_observation_b_deg:.3f})"
            )
            return False
        self.reset_search_attempt_preserving_b0()
        return True

    def reset_search_attempt_preserving_b0(self) -> None:
        self.search_index = 0
        self.search_saw_target = False
        self.clear_b_route()
        self.session.restart_localization(time.monotonic(), "b_search_offset_ready")

    def advance_b_search(self) -> bool:
        next_index = self.search_index + 1
        if (
            next_index >= len(self.b_search_config.offsets_deg)
            or next_index > self.b_search_config.maximum_automatic_search_index
        ):
            return False
        route = self.b_search_config.route_between(
            self.cycle_observation_b_deg, self.search_index, next_index
        )
        self.start_b_route(route, next_index, False)
        return True

    def start_return_to_b0(self) -> None:
        route = self.b_search_config.route_to_b0(
            self.cycle_observation_b_deg, self.search_index
        )
        if not route:
            self.complete_return_to_b0()
            return
        self.start_b_route(route, None, True)

    def start_b_route(
        self, route, final_search_index: Optional[int], returns_to_b0: bool
    ) -> None:
        self.search_route = list(route)
        self.route_final_search_index = final_search_index
        self.route_returns_to_b0 = returns_to_b0
        self.session.state = "waiting_b_motion"
        self.session.last_reason = "b_search_motion_requested"
        self.send_next_b_route_target()

    def send_next_b_route_target(self) -> None:
        if not self.search_route:
            if self.route_returns_to_b0:
                self.complete_return_to_b0()
            else:
                self.search_index = int(self.route_final_search_index)
                self.clear_b_route()
                self.session.restart_localization(
                    time.monotonic(), "b_search_offset_ready"
                )
                self.publish_state("localizing", "b_search_offset_ready")
            return
        target = float(self.search_route.pop(0))
        self.b_command_sequence += 1
        command_id = f"task-{self.session.active_task_id}-b-{self.b_command_sequence}"
        command = {
            "command_id": command_id,
            "type": "move_b_joint",
            "joint": 1,
            "angle": target,
            "speed": self.b_search_config.speed_deg_s,
            "acceleration": self.b_search_config.acceleration,
        }
        self.pending_b_target_deg = target
        self.pending_b_command_id = command_id
        self.pending_b_sent_monotonic = time.monotonic()
        self.pending_b_command_accepted = False
        self.b_arrival_stable_count = 0
        self.publish_json(self.arm_command_publisher, command)
        self.publish_state(
            "waiting_b_motion",
            "b_search_motion_requested",
            {"pending_b_target_deg": target, "pending_b_command_id": command_id},
        )

    def update_b_arrival(self, payload: Dict[str, Any]) -> None:
        if (
            self.pending_b_target_deg is None
            or not self.pending_b_command_accepted
            or not bool(payload.get("state_valid", False))
        ):
            return
        state = payload.get("state")
        if not isinstance(state, dict):
            return
        try:
            actual = math.degrees(float(state["b"]))
        except (KeyError, TypeError, ValueError):
            return
        error = actual - self.pending_b_target_deg
        if abs(error) <= self.b_search_config.arrival_tolerance_deg:
            self.b_arrival_stable_count += 1
        else:
            self.b_arrival_stable_count = 0
        if self.b_arrival_stable_count >= self.b_search_config.arrival_stable_samples:
            self.pending_b_target_deg = None
            self.pending_b_command_id = None
            self.pending_b_sent_monotonic = None
            self.pending_b_command_accepted = False
            self.b_arrival_stable_count = 0
            self.send_next_b_route_target()

    def on_arm_command_result(self, message: String) -> None:
        if self.pending_b_command_id is None:
            return
        try:
            result = self.parse_json(message)
        except Exception:
            return
        if result.get("command_id") != self.pending_b_command_id:
            return
        if not bool(result.get("accepted", False)):
            self.fail_b_search("b_command_rejected", result)
            return
        self.pending_b_command_accepted = True

    def check_b_motion_timeout(self) -> None:
        if self.pending_b_sent_monotonic is None:
            return
        if (
            time.monotonic() - self.pending_b_sent_monotonic
            >= self.b_search_config.motion_timeout_s
        ):
            self.fail_b_search("b_motion_timeout", {})

    def fail_b_search(self, reason: str, detail: Dict[str, Any]) -> None:
        failure_detail = {
            **detail,
            "failed_b_target_deg": self.pending_b_target_deg,
            "failed_b_command_id": self.pending_b_command_id,
            "arrival_stable_count": self.b_arrival_stable_count,
        }
        if self.pending_terminal_b_failure is not None:
            original = dict(self.pending_terminal_b_failure)
            terminal_detail = {
                **failure_detail,
                "original_b_failure_reason": original.get("reason"),
                "b0_recovery_attempted": True,
                "b0_recovery_succeeded": False,
            }
            self.finish_b_search_failure("b0_recovery_failed", terminal_detail)
            return

        actual_b_deg = self.latest_b_deg_for_preflight()
        if actual_b_deg is None or self.cycle_observation_b_deg is None:
            failure_detail.update(
                {
                    "b0_recovery_attempted": False,
                    "b0_recovery_succeeded": False,
                    "b0_recovery_failure": "no_fresh_b_feedback_or_cycle_b0",
                }
            )
            self.finish_b_search_failure(reason, failure_detail)
            return

        try:
            recovery_route = self.b_search_config.route_from_actual_to_b0(
                self.cycle_observation_b_deg, actual_b_deg
            )
        except ValueError as exc:
            failure_detail.update(
                {
                    "actual_b_deg_at_failure": actual_b_deg,
                    "b0_recovery_attempted": False,
                    "b0_recovery_succeeded": False,
                    "b0_recovery_failure": str(exc),
                }
            )
            self.finish_b_search_failure(reason, failure_detail)
            return

        self.pending_terminal_b_failure = {
            "reason": reason,
            **failure_detail,
            "actual_b_deg_at_failure": actual_b_deg,
            "b0_recovery_attempted": True,
        }
        self.clear_b_route()
        self.publish_state(
            "recovering_b0_after_failure",
            reason,
            {
                "actual_b_deg_at_failure": actual_b_deg,
                "b0_recovery_route_deg": recovery_route,
            },
        )
        if not recovery_route:
            self.complete_return_to_b0()
            return
        self.start_b_route(recovery_route, None, True)

    def finish_b_search_failure(self, reason: str, detail: Dict[str, Any]) -> None:
        task_id = self.session.active_task_id
        self.publish_state("b_search_failed", reason, detail)
        self.publish_result(task_id, "localization_failed", reason, detail)
        self.clear_b_route()
        self.pending_terminal_b_failure = None
        self.session.finish_terminal()

    def complete_return_to_b0(self) -> None:
        if self.pending_terminal_b_failure is not None:
            failure = dict(self.pending_terminal_b_failure)
            reason = str(failure.pop("reason"))
            failure.update(
                {
                    "returned_to_cycle_b0": True,
                    "b0_recovery_succeeded": True,
                }
            )
            self.finish_b_search_failure(reason, failure)
            return
        reason = "target_unstable" if self.search_saw_target else "target_not_found"
        self.clear_b_route()
        self.search_index = 0
        self.session.mark_reposition_required(reason)
        extra = {
            "search_offsets_deg": list(self.b_search_config.offsets_deg),
            "enabled_search_offsets_deg": list(
                self.b_search_config.offsets_deg[
                    : self.b_search_config.maximum_automatic_search_index + 1
                ]
            ),
            "returned_to_cycle_b0": True,
            "reposition_required": True,
        }
        self.publish_state("reposition_required", reason, extra)
        self.publish_result(
            self.session.active_task_id, "reposition_required", reason, extra
        )

    def clear_b_route(self) -> None:
        self.search_route = []
        self.route_final_search_index = None
        self.route_returns_to_b0 = False
        self.pending_b_target_deg = None
        self.pending_b_command_id = None
        self.pending_b_sent_monotonic = None
        self.pending_b_command_accepted = False
        self.b_arrival_stable_count = 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[ManipulationTaskNode] = None
    try:
        node = ManipulationTaskNode()
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
