#!/usr/bin/env python3
"""Stage-4C command-driven localization task node with no motion authority."""

import json
import time
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

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
        self.declare_parameter(
            "stability_config_path", str(share / "config" / "target_stability.json")
        )
        self.declare_parameter("timeout_check_period_s", 0.05)

        self.task_cmd_topic = str(self.get_parameter("task_cmd_topic").value)
        self.task_state_topic = str(self.get_parameter("task_state_topic").value)
        self.task_result_topic = str(self.get_parameter("task_result_topic").value)
        self.candidate_topic = str(self.get_parameter("candidate_topic").value)
        timeout_check_period_s = float(
            self.get_parameter("timeout_check_period_s").value
        )
        if timeout_check_period_s <= 0.0:
            raise ValueError("timeout_check_period_s must be positive")

        config = load_target_stability_config(
            str(self.get_parameter("stability_config_path").value)
        )
        self.session = LocalizationTaskSession(StableTargetLock(config))
        self.state_publisher = self.create_publisher(String, self.task_state_topic, 10)
        self.result_publisher = self.create_publisher(String, self.task_result_topic, 10)
        self.command_subscription = self.create_subscription(
            String, self.task_cmd_topic, self.on_task_command, 10
        )
        self.candidate_subscription = self.create_subscription(
            String, self.candidate_topic, self.on_candidates, 10
        )
        self.timeout_timer = self.create_timer(
            timeout_check_period_s, self.on_timeout_check
        )
        self.publish_state("idle", "ready")
        self.get_logger().info(
            "Stage-4C manipulation task node started: command-driven localization only; "
            "observation motion, B search, gripper and all arm commands disabled."
        )

    @staticmethod
    def parse_json(message: String) -> Dict[str, Any]:
        data = json.loads(message.data)
        if not isinstance(data, dict):
            raise ValueError("message must contain a JSON object")
        return data

    @staticmethod
    def safety_fields() -> Dict[str, Any]:
        return {
            "picked_ids": [],
            "placed_ids": [],
            "carrying_id": None,
            "target_snapshot_only": True,
            "pick_motion_executed": False,
            "observation_motion_enabled": False,
            "b_search_enabled": False,
            "gripper_commands_enabled": False,
            "motion_commands_enabled": False,
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
        if not self.session.active:
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
            }
            self.publish_state("snapshot_ready", "stable_target_ready", snapshot)
            self.publish_result(
                task_id, "target_snapshot_ready", "stable_target_ready", snapshot
            )
            self.session.finish_terminal()
            return

        if status == "failed":
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
