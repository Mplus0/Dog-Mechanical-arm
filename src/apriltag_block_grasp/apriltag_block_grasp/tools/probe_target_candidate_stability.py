#!/usr/bin/env python3
"""Collect raw Stage-4A target candidates without applying pass thresholds."""

import argparse
import json
import math
import sys
import time
from typing import Any, Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def distribution(values: List[float]) -> Dict[str, Any]:
    if not values:
        return {"count": 0}
    data = np.asarray(values, dtype=np.float64)
    return {
        "count": int(data.size),
        "min": float(np.min(data)),
        "median": float(np.median(data)),
        "max": float(np.max(data)),
        "peak_to_peak": float(np.ptp(data)),
        "std": float(np.std(data)),
    }


def finite(value: Any) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("value must be finite")
    return number


def summarize_samples(samples: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Build threshold-free statistics without depending on ROS runtime state."""

    positions = np.asarray(
        [
            [sample["base_object_mm"][axis] for axis in ("x", "y", "z")]
            for sample in samples
        ],
        dtype=np.float64,
    )
    median = None
    radial = []
    if positions.size:
        median_array = np.median(positions, axis=0)
        median = {
            axis: float(median_array[index])
            for index, axis in enumerate(("x", "y", "z"))
        }
        radial = np.linalg.norm(positions - median_array, axis=1).tolist()
    arm_fields = ("x", "y", "z", "tit", "b", "s", "e", "t", "r", "g")
    return {
        "base_object_mm": {
            axis: distribution(
                [sample["base_object_mm"][axis] for sample in samples]
            )
            for axis in ("x", "y", "z")
        },
        "base_object_median_mm": median,
        "distance_from_median_mm": distribution(radial),
        "pnp_arm_stamp_delta_s": distribution(
            [sample["pnp_arm_stamp_delta_s"] for sample in samples]
        ),
        "arm_state_reported_age_s": distribution(
            [sample["arm_state_reported_age_s"] for sample in samples]
        ),
        "reprojection_error_px": distribution(
            [
                sample["reprojection_error_px"]
                for sample in samples
                if sample["reprojection_error_px"] is not None
            ]
        ),
        "area_px2": distribution(
            [sample["area_px2"] for sample in samples if sample["area_px2"] is not None]
        ),
        "arm_state_stability": {
            field: distribution(
                [
                    sample["arm_state"][field]
                    for sample in samples
                    if field in sample["arm_state"]
                ]
            )
            for field in arm_fields
        },
    }


class CandidateStabilityProbe(Node):
    def __init__(self, tag_id: int, sample_count: int, topic: str) -> None:
        super().__init__("apriltag_target_candidate_stability_probe")
        self.tag_id = int(tag_id)
        self.sample_count = int(sample_count)
        self.topic = str(topic)
        self.received_message_count = 0
        self.invalid_message_count = 0
        self.target_absent_message_count = 0
        self.samples: List[Dict[str, Any]] = []
        self.subscription = self.create_subscription(
            String, self.topic, self.on_message, 10
        )

    def on_message(self, message: String) -> None:
        self.received_message_count += 1
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                raise ValueError("candidate message must be an object")
            candidates = data.get("candidates")
            if not isinstance(candidates, list):
                raise ValueError("candidates must be a list")
            target = next(
                (
                    item
                    for item in candidates
                    if isinstance(item, dict)
                    and int(item.get("tag_id", -1)) == self.tag_id
                ),
                None,
            )
            if target is None:
                self.target_absent_message_count += 1
                return
            position = target["base_object_mm"]
            arm = data.get("arm_state_snapshot")
            if not isinstance(position, dict) or not isinstance(arm, dict):
                raise ValueError("candidate position or arm snapshot is missing")
            sample = {
                "index": len(self.samples),
                "candidate_stamp": finite(data["stamp"]),
                "pnp_stamp": finite(data["pnp_stamp"]),
                "arm_state_stamp": finite(data["arm_state_stamp"]),
                "pnp_arm_stamp_delta_s": finite(data["pnp_arm_stamp_delta_s"]),
                "arm_state_reported_age_s": finite(
                    data["arm_state_reported_age_s"]
                ),
                "base_object_mm": {
                    axis: finite(position[axis]) for axis in ("x", "y", "z")
                },
                "reprojection_error_px": (
                    None
                    if target.get("reprojection_error_px") is None
                    else finite(target["reprojection_error_px"])
                ),
                "area_px2": (
                    None
                    if target.get("area_px2") is None
                    else finite(target["area_px2"])
                ),
                "arm_state": {
                    key: finite(arm[key])
                    for key in ("x", "y", "z", "tit", "b", "s", "e", "t", "r", "g")
                    if key in arm
                },
            }
        except Exception:
            self.invalid_message_count += 1
            return
        self.samples.append(sample)

    @property
    def complete(self) -> bool:
        return len(self.samples) >= self.sample_count

    def report(self, timeout_s: float) -> Dict[str, Any]:
        report = {
            "tool": "apriltag_block_grasp.probe_target_candidate_stability",
            "read_only": True,
            "topic": self.topic,
            "tag_id": self.tag_id,
            "requested_sample_count": self.sample_count,
            "timeout_s": float(timeout_s),
            "thresholds_applied": False,
            "selection_enabled": False,
            "locking_enabled": False,
            "camera_opened": False,
            "arm_serial_opened": False,
            "motion_commands_enabled": False,
            "samples": self.samples,
            "summary": {
                "valid": self.complete,
                "reason": "requested_samples_collected" if self.complete else "sample_timeout",
                "received_message_count": self.received_message_count,
                "valid_target_sample_count": len(self.samples),
                "target_absent_message_count": self.target_absent_message_count,
                "invalid_message_count": self.invalid_message_count,
                "motion_command_sent": False,
            },
        }
        report.update(summarize_samples(self.samples))
        return report


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Collect raw base-object stability data from Stage-4A candidates."
    )
    parser.add_argument("--tag-id", type=int, choices=(0, 1), required=True)
    parser.add_argument("--sample-count", type=int, default=100)
    parser.add_argument("--timeout-s", type=float, default=20.0)
    parser.add_argument(
        "--topic", default="/apriltag_grasp/target_candidates"
    )
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    if args.sample_count < 1:
        raise ValueError("sample-count must be positive")
    if not math.isfinite(args.timeout_s) or args.timeout_s <= 0.0:
        raise ValueError("timeout-s must be finite and positive")
    rclpy.init()
    node: Optional[CandidateStabilityProbe] = None
    try:
        node = CandidateStabilityProbe(args.tag_id, args.sample_count, args.topic)
        deadline = time.monotonic() + args.timeout_s
        while not node.complete and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        report = node.report(args.timeout_s)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report["summary"]["valid"] else 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
