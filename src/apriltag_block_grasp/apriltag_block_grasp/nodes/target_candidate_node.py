#!/usr/bin/env python3
"""Read-only Stage-4A base-object candidate publisher without target locking."""

import json
import math
import time
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

from apriltag_block_grasp.core.handeye import load_handeye_calibration
from apriltag_block_grasp.core.tag_to_object import load_tag_to_object_calibration
from apriltag_block_grasp.core.target_candidate import build_base_object_candidates


class TargetCandidateNode(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_target_candidate_node")
        share = Path(get_package_share_directory("apriltag_block_grasp"))
        self.declare_parameter("pnp_topic", "/apriltag_grasp/pnp")
        self.declare_parameter("arm_state_topic", "/roarm_m3/state")
        self.declare_parameter("candidate_topic", "/apriltag_grasp/target_candidates")
        self.declare_parameter("allowed_ids", [0, 1])
        self.declare_parameter("handeye_path", str(share / "config" / "handeye_cam_to_eef.json"))
        self.declare_parameter("tag_to_object_path", str(share / "config" / "tag_to_object.json"))

        self.pnp_topic = str(self.get_parameter("pnp_topic").value)
        self.arm_state_topic = str(self.get_parameter("arm_state_topic").value)
        self.candidate_topic = str(self.get_parameter("candidate_topic").value)
        self.allowed_ids = tuple(
            int(value) for value in self.get_parameter("allowed_ids").value
        )
        if not self.allowed_ids or any(value not in (0, 1) for value in self.allowed_ids):
            raise ValueError("allowed_ids must contain only ID 0 and/or ID 1")

        self.handeye = load_handeye_calibration(
            str(self.get_parameter("handeye_path").value)
        )
        self.tag_to_object = load_tag_to_object_calibration(
            str(self.get_parameter("tag_to_object_path").value)
        )
        self.latest_arm_state: Optional[Dict[str, Any]] = None
        self.latest_arm_stamp: Optional[float] = None
        self.latest_arm_age_s: Optional[float] = None
        self.arm_state_valid = False
        self.publisher = self.create_publisher(String, self.candidate_topic, 10)
        self.arm_subscription = self.create_subscription(
            String, self.arm_state_topic, self.on_arm_state, 10
        )
        self.pnp_subscription = self.create_subscription(
            String, self.pnp_topic, self.on_pnp, 10
        )
        self.get_logger().info(
            "Target candidate node started read-only: no camera, serial or motion access; "
            "selection, locking and stability thresholds are disabled."
        )

    @staticmethod
    def parse_json(message: String) -> Dict[str, Any]:
        data = json.loads(message.data)
        if not isinstance(data, dict):
            raise ValueError("message data must be a JSON object")
        return data

    def on_arm_state(self, message: String) -> None:
        try:
            data = self.parse_json(message)
            state = data.get("state")
            valid = bool(data.get("state_valid", False))
            stamp = float(data["stamp"])
            age = data.get("state_age_s")
            age_s = None if age is None else float(age)
            if (
                not valid
                or not isinstance(state, dict)
                or not math.isfinite(stamp)
                or (age_s is not None and not math.isfinite(age_s))
            ):
                raise ValueError("arm state is invalid")
        except Exception:
            self.arm_state_valid = False
            return
        self.latest_arm_state = state
        self.latest_arm_stamp = stamp
        self.latest_arm_age_s = age_s
        self.arm_state_valid = True

    def on_pnp(self, message: String) -> None:
        received_at = time.time()
        reason = "ok"
        candidates = []
        pnp_stamp = None
        pnp_reason = None
        try:
            data = self.parse_json(message)
            pnp_stamp = float(data["stamp"])
            pnp_reason = str(data.get("reason", ""))
            detections = data.get("detections", [])
            if not math.isfinite(pnp_stamp) or not isinstance(detections, list):
                raise ValueError("invalid PnP message")
            if not self.arm_state_valid or self.latest_arm_state is None:
                reason = "arm_state_unavailable"
            else:
                candidates = build_base_object_candidates(
                    detections,
                    self.latest_arm_state,
                    self.handeye.matrix_eef_camera,
                    self.tag_to_object.matrix_tag_object,
                    self.allowed_ids,
                )
                reason = "ok" if candidates else "no_valid_candidate"
        except Exception as exc:
            reason = f"candidate_build_failed: {type(exc).__name__}: {exc}"

        arm_stamp = self.latest_arm_stamp
        arm_state_snapshot = None
        if isinstance(self.latest_arm_state, dict):
            arm_state_snapshot = {
                key: self.latest_arm_state.get(key)
                for key in ("x", "y", "z", "tit", "b", "s", "e", "t", "r", "g")
                if key in self.latest_arm_state
            }
        payload = {
            "stamp": received_at,
            "valid": reason in ("ok", "no_valid_candidate"),
            "reason": reason,
            "pnp_stamp": pnp_stamp,
            "arm_state_stamp": arm_stamp,
            "pnp_arm_stamp_delta_s": (
                None
                if pnp_stamp is None or arm_stamp is None
                else float(pnp_stamp - arm_stamp)
            ),
            "arm_state_reported_age_s": self.latest_arm_age_s,
            "arm_state_snapshot": arm_state_snapshot,
            "pnp_reason": pnp_reason,
            "allowed_ids": list(self.allowed_ids),
            "count": len(candidates),
            "candidates": candidates,
            "selection_enabled": False,
            "locking_enabled": False,
            "stability_check_enabled": False,
            "base_position_correction_applied": False,
            "handeye_applied": True,
            "tag_to_object_applied": True,
            "depth_enabled": False,
            "camera_opened": False,
            "arm_serial_opened": False,
            "motion_commands_enabled": False,
        }
        output = String()
        output.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[TargetCandidateNode] = None
    try:
        node = TargetCandidateNode()
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
