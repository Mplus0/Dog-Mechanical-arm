#!/usr/bin/env python3
"""Read a finite set of official RoArm poses without sending motion commands."""

import json
import time
from typing import Any, Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from roarm_msgs.srv import GetPoseCmd

from apriltag_block_grasp.core.rigid_transform import (
    make_transform,
    rotation_from_rpy,
    rotation_quality,
)


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
    }


class ArmPoseProbe(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_arm_pose_probe")
        self.declare_parameter("service_name", "/get_pose_cmd")
        self.declare_parameter("position_scale_to_mm", 1000.0)
        self.declare_parameter("sample_count", 20)
        self.declare_parameter("service_wait_timeout_s", 5.0)
        self.declare_parameter("response_timeout_s", 1.0)
        self.declare_parameter("sample_interval_s", 0.10)

        self.service_name = str(self.get_parameter("service_name").value).strip()
        self.position_scale_to_mm = float(
            self.get_parameter("position_scale_to_mm").value
        )
        self.sample_count = max(1, int(self.get_parameter("sample_count").value))
        self.service_wait_timeout_s = max(
            0.1, float(self.get_parameter("service_wait_timeout_s").value)
        )
        self.response_timeout_s = max(
            0.1, float(self.get_parameter("response_timeout_s").value)
        )
        self.sample_interval_s = max(
            0.0, float(self.get_parameter("sample_interval_s").value)
        )
        if not self.service_name:
            raise ValueError("service_name must not be empty")
        if not np.isfinite(self.position_scale_to_mm) or self.position_scale_to_mm <= 0.0:
            raise ValueError("position_scale_to_mm must be finite and positive")
        self.client = self.create_client(GetPoseCmd, self.service_name)

    def read_samples(self) -> Dict[str, Any]:
        report: Dict[str, Any] = {
            "tool": "apriltag_block_grasp.probe_arm_pose",
            "read_only": True,
            "service_name": self.service_name,
            "request_type": "roarm_msgs/srv/GetPoseCmd",
            "request_has_no_motion_fields": True,
            "position_input_unit": "m",
            "position_output_unit": "mm",
            "position_scale_to_mm": self.position_scale_to_mm,
            "angle_unit": "rad",
            "rotation_order": "Rz(yaw) @ Ry(pitch) @ Rx(roll)",
            "camera_opened": False,
            "handeye_loaded": False,
            "motion_commands_enabled": False,
            "samples": [],
            "summary": {"valid": False},
        }
        if not self.client.wait_for_service(timeout_sec=self.service_wait_timeout_s):
            report["summary"] = {
                "valid": False,
                "reason": "get_pose_service_unavailable",
            }
            return report

        samples: List[Dict[str, Any]] = []
        failed_requests = 0
        for index in range(self.sample_count):
            future = self.client.call_async(GetPoseCmd.Request())
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=self.response_timeout_s
            )
            if not future.done() or future.cancelled() or future.exception() is not None:
                failed_requests += 1
                continue
            response = future.result()
            if response is None:
                failed_requests += 1
                continue
            raw_values = np.array(
                [
                    response.x,
                    response.y,
                    response.z,
                    response.roll,
                    response.pitch,
                    response.yaw,
                ],
                dtype=np.float64,
            )
            if not np.all(np.isfinite(raw_values)):
                failed_requests += 1
                continue

            position_mm = raw_values[:3] * self.position_scale_to_mm
            rotation = rotation_from_rpy(*raw_values[3:])
            transform = make_transform(rotation, position_mm)
            orthogonality_error, determinant = rotation_quality(rotation)
            samples.append(
                {
                    "index": index,
                    "raw_pose_m_rad": {
                        "x": float(raw_values[0]),
                        "y": float(raw_values[1]),
                        "z": float(raw_values[2]),
                        "roll": float(raw_values[3]),
                        "pitch": float(raw_values[4]),
                        "yaw": float(raw_values[5]),
                    },
                    "pose_mm_rad": {
                        "x": float(position_mm[0]),
                        "y": float(position_mm[1]),
                        "z": float(position_mm[2]),
                        "roll": float(raw_values[3]),
                        "pitch": float(raw_values[4]),
                        "yaw": float(raw_values[5]),
                    },
                    "T_base_eef": transform.tolist(),
                    "rotation_quality": {
                        "orthogonality_error_frobenius": orthogonality_error,
                        "determinant": determinant,
                    },
                }
            )
            if index + 1 < self.sample_count and self.sample_interval_s > 0.0:
                time.sleep(self.sample_interval_s)

        report["samples"] = samples
        fields = ("x", "y", "z", "roll", "pitch", "yaw")
        report["stability"] = {
            field: distribution([sample["pose_mm_rad"][field] for sample in samples])
            for field in fields
        }
        report["summary"] = {
            "valid": len(samples) > 0 and failed_requests == 0,
            "requested_count": self.sample_count,
            "valid_count": len(samples),
            "failed_request_count": failed_requests,
            "all_values_finite": bool(samples),
            "motion_command_sent": False,
        }
        return report


def main(args=None) -> int:
    rclpy.init(args=args)
    node: Optional[ArmPoseProbe] = None
    try:
        node = ArmPoseProbe()
        report = node.read_samples()
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report["summary"].get("valid", False) else 1
    except Exception as exc:
        print(
            json.dumps(
                {
                    "tool": "apriltag_block_grasp.probe_arm_pose",
                    "read_only": True,
                    "camera_opened": False,
                    "motion_commands_enabled": False,
                    "summary": {
                        "valid": False,
                        "error": f"{type(exc).__name__}: {exc}",
                    },
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
