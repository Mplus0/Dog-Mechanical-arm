#!/usr/bin/env python3
"""Read-only Stage-4B target selection, locking and XYZ stability node."""

import json
import time
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

from apriltag_block_grasp.core.target_lock import (
    StableTargetLock,
    load_target_stability_config,
)


class StableTargetNode(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_stable_target_node")
        share = Path(get_package_share_directory("apriltag_block_grasp"))
        self.declare_parameter("candidate_topic", "/apriltag_grasp/target_candidates")
        self.declare_parameter("stable_target_topic", "/apriltag_grasp/stable_target")
        self.declare_parameter(
            "stability_config_path", str(share / "config" / "target_stability.json")
        )

        self.candidate_topic = str(self.get_parameter("candidate_topic").value)
        self.stable_target_topic = str(self.get_parameter("stable_target_topic").value)
        config_path = str(self.get_parameter("stability_config_path").value)
        self.config = load_target_stability_config(config_path)
        self.lock = StableTargetLock(self.config)
        self.publisher = self.create_publisher(String, self.stable_target_topic, 10)
        self.subscription = self.create_subscription(
            String, self.candidate_topic, self.on_candidates, 10
        )
        self.get_logger().info(
            "Stable target node started read-only: selection/locking/stability enabled; "
            "camera, arm serial, B search and all motion commands disabled."
        )

    @staticmethod
    def parse_json(message: String) -> Dict[str, Any]:
        data = json.loads(message.data)
        if not isinstance(data, dict):
            raise ValueError("candidate message must be a JSON object")
        return data

    def on_candidates(self, message: String) -> None:
        received_at = time.time()
        try:
            candidate_payload = self.parse_json(message)
            result = self.lock.update(candidate_payload, time.monotonic())
        except Exception as exc:
            result = {
                "status": "input_error",
                "reason": f"input_error: {type(exc).__name__}: {exc}",
                "locked_id": self.lock.locked_id,
                "collected_frame_count": len(self.lock.samples),
                "required_frame_count": self.config.stable_frame_count,
            }

        payload = {
            "stamp": received_at,
            "valid": result.get("status") == "stable",
            **result,
            "selection_order": list(self.config.selection_order),
            "xyz_peak_to_peak_threshold_mm": list(
                self.config.xyz_peak_to_peak_threshold_mm
            ),
            "stable_timeout_s": self.config.stable_timeout_s,
            "max_pnp_arm_stamp_delta_s": self.config.max_pnp_arm_stamp_delta_s,
            "max_arm_state_reported_age_s": self.config.max_arm_state_reported_age_s,
            "selection_enabled": True,
            "locking_enabled": True,
            "stability_check_enabled": True,
            "snapshot_frozen": result.get("status") == "stable",
            "b_search_enabled": False,
            "task_cycle_enabled": False,
            "camera_opened": False,
            "arm_serial_opened": False,
            "motion_commands_enabled": False,
        }
        output = String()
        output.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[StableTargetNode] = None
    try:
        node = StableTargetNode()
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
