#!/usr/bin/env python3
"""Paired hand-eye check around one bounded B-joint command."""

import argparse
import json
import math
import time
from collections import defaultdict
from typing import Any, Dict, List, Optional

import numpy as np
from ament_index_python.packages import get_package_share_directory

from apriltag_block_grasp.core.apriltag_detector import OpenCvAprilTag25h9Detector
from apriltag_block_grasp.core.camera_calibration import read_orbbec_color_calibration
from apriltag_block_grasp.core.camera_color_orbbec import OrbbecColorCamera
from apriltag_block_grasp.core.handeye import load_handeye_calibration
from apriltag_block_grasp.core.pose_estimator import AprilTagPoseEstimator
from apriltag_block_grasp.core.roarm_serial_control import RoArmBJointController
from apriltag_block_grasp.core.roarm_state import cartesian_pose_from_state
from apriltag_block_grasp.tools.move_b_joint_safe import (
    b_degrees_from_state,
    compact_state,
    finite_float,
    validate_motion_request,
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
        "std": float(np.std(data)),
    }


def summarize_samples(samples: List[Dict[str, Any]]) -> Dict[str, Any]:
    summary = {
        "sample_count": len(samples),
        "base_tag_mm": {},
        "camera_tag_mm": {},
        "arm_pose_mm_rad": {},
        "reprojection_error_px": distribution(
            [sample["reprojection_error_px"] for sample in samples]
        ),
    }
    for field in ("base_tag_mm", "camera_tag_mm"):
        summary[field] = {
            axis: distribution([sample[field][axis] for sample in samples])
            for axis in ("x", "y", "z")
        }
    summary["arm_pose_mm_rad"] = {
        key: distribution([sample["arm_pose_mm_rad"][key] for sample in samples])
        for key in ("x", "y", "z", "roll", "pitch", "yaw")
    }
    return summary


def median_xyz(group: Dict[str, Any], field: str) -> np.ndarray:
    return np.asarray(
        [group[field][axis]["median"] for axis in ("x", "y", "z")],
        dtype=np.float64,
    )


def collect_group(
    *,
    name: str,
    arm: RoArmBJointController,
    camera: OrbbecColorCamera,
    detector: OpenCvAprilTag25h9Detector,
    estimator: AprilTagPoseEstimator,
    matrix_eef_camera: np.ndarray,
    tag_id: int,
    sample_count: int,
    max_attempts: int,
    state_timeout_s: float,
    frame_timeout_ms: int,
) -> Dict[str, Any]:
    samples: List[Dict[str, Any]] = []
    failures = defaultdict(int)
    for attempt_index in range(max_attempts):
        if len(samples) >= sample_count:
            break
        state = arm.read_state(timeout_s=state_timeout_s)
        if state is None:
            failures["empty_arm_state"] += 1
            continue
        try:
            arm_pose = cartesian_pose_from_state(state)
        except Exception:
            failures["invalid_arm_state"] += 1
            continue
        frame = camera.read(timeout_ms=frame_timeout_ms)
        if frame is None:
            failures["empty_color_frame"] += 1
            continue
        height, width = frame.bgr.shape[:2]
        if not estimator.calibration.matches_frame(width, height):
            failures["calibration_resolution_mismatch"] += 1
            continue
        batch = detector.detect(frame.bgr)
        detection = next(
            (item for item in batch.detections if int(item.tag_id) == tag_id), None
        )
        if detection is None:
            failures["requested_tag_not_detected"] += 1
            continue
        try:
            pose = estimator.estimate(detection)
        except Exception:
            failures["pnp_failed"] += 1
            continue
        matrix_camera_tag = np.eye(4, dtype=np.float64)
        matrix_camera_tag[:3, :3] = pose.rotation_matrix
        matrix_camera_tag[:3, 3] = pose.translation_mm
        matrix_base_camera = arm_pose.matrix_base_eef @ matrix_eef_camera
        matrix_base_tag = matrix_base_camera @ matrix_camera_tag
        if not np.all(np.isfinite(matrix_base_tag)):
            failures["non_finite_chain"] += 1
            continue
        samples.append(
            {
                "sample_index": len(samples),
                "read_attempt_index": attempt_index,
                "tag_id": tag_id,
                "arm_pose_mm_rad": {
                    "x": arm_pose.x_mm,
                    "y": arm_pose.y_mm,
                    "z": arm_pose.z_mm,
                    "roll": arm_pose.roll_rad,
                    "pitch": arm_pose.pitch_rad,
                    "yaw": arm_pose.yaw_rad,
                },
                "camera_tag_mm": {
                    "x": float(pose.translation_mm[0]),
                    "y": float(pose.translation_mm[1]),
                    "z": float(pose.translation_mm[2]),
                },
                "base_tag_mm": {
                    "x": float(matrix_base_tag[0, 3]),
                    "y": float(matrix_base_tag[1, 3]),
                    "z": float(matrix_base_tag[2, 3]),
                },
                "reprojection_error_px": float(pose.reprojection_error_px),
            }
        )
    return {
        "name": name,
        "valid": len(samples) == sample_count,
        "requested_sample_count": sample_count,
        "valid_sample_count": len(samples),
        "failure_counts": dict(sorted(failures.items())),
        "statistics": summarize_samples(samples),
        "last_sample": samples[-1] if samples else None,
    }


def parse_arguments():
    default_handeye = (
        get_package_share_directory("apriltag_block_grasp")
        + "/config/handeye_cam_to_eef.json"
    )
    parser = argparse.ArgumentParser(
        description="Collect hand-eye samples before and after one B-joint command."
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--handeye-path", default=default_handeye)
    parser.add_argument("--tag-id", type=int, choices=(0, 1), required=True)
    parser.add_argument("--target-b-deg", type=float, required=True)
    parser.add_argument("--sample-count", type=int, default=20)
    parser.add_argument("--max-attempts-per-group", type=int, default=80)
    parser.add_argument("--state-timeout-s", type=float, default=0.5)
    parser.add_argument("--frame-timeout-ms", type=int, default=500)
    parser.add_argument("--tag-size-mm", type=float, default=38.9)
    parser.add_argument("--speed-deg-s", type=float, default=10.0)
    parser.add_argument("--acceleration", type=float, default=10.0)
    parser.add_argument("--min-b-deg", type=float, default=-20.0)
    parser.add_argument("--max-b-deg", type=float, default=20.0)
    parser.add_argument("--max-delta-deg", type=float, default=10.0)
    parser.add_argument("--motion-observe-s", type=float, default=3.0)
    parser.add_argument(
        "--enable-motion",
        action="store_true",
        help="Enable one B command after a complete baseline group.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    sample_count = max(1, int(args.sample_count))
    max_attempts = max(sample_count, int(args.max_attempts_per_group))
    tag_id = int(args.tag_id)
    camera = OrbbecColorCamera()
    arm = RoArmBJointController(port=args.port, timeout_s=0.2)
    report: Dict[str, Any] = {
        "tool": "apriltag_block_grasp.probe_handeye_pair_b",
        "formula": "T_base_tag = T_base_eef @ T_eef_camera @ T_camera_tag",
        "same_camera_connection": True,
        "same_serial_connection": True,
        "tag_id": tag_id,
        "tag_size_mm": float(args.tag_size_mm),
        "dry_run": not args.enable_motion,
        "enable_motion": bool(args.enable_motion),
        "maximum_motion_command_count": 1,
        "automatic_retry_enabled": False,
        "automatic_recovery_enabled": False,
        "other_joints_commanded": False,
        "gripper_commanded": False,
        "fill_light_commanded": False,
        "groups": {},
        "summary": {"valid": False},
    }
    try:
        target_b_deg = finite_float(args.target_b_deg, "target_b_deg")
        motion_observe_s = finite_float(args.motion_observe_s, "motion_observe_s")
        if motion_observe_s <= 0.0:
            raise ValueError("motion_observe_s must be positive")
        handeye = load_handeye_calibration(args.handeye_path)
        arm.connect()
        camera.start()
        calibration = read_orbbec_color_calibration(camera)
        detector = OpenCvAprilTag25h9Detector(allowed_ids=(tag_id,))
        estimator = AprilTagPoseEstimator(float(args.tag_size_mm), calibration)

        baseline = collect_group(
            name="baseline",
            arm=arm,
            camera=camera,
            detector=detector,
            estimator=estimator,
            matrix_eef_camera=handeye.matrix_eef_camera,
            tag_id=tag_id,
            sample_count=sample_count,
            max_attempts=max_attempts,
            state_timeout_s=float(args.state_timeout_s),
            frame_timeout_ms=int(args.frame_timeout_ms),
        )
        report["groups"]["baseline"] = baseline
        if not baseline["valid"]:
            report["summary"] = {
                "valid": False,
                "reason": "baseline_incomplete_no_motion_sent",
                "motion_command_sent": False,
            }
            print(json.dumps(report, ensure_ascii=False, indent=2))
            return 1

        initial_state = arm.read_state(timeout_s=float(args.state_timeout_s))
        if initial_state is None:
            raise RuntimeError("no arm state received after baseline")
        initial_b_deg = b_degrees_from_state(initial_state)
        requested_delta_deg = validate_motion_request(
            current_b_deg=initial_b_deg,
            target_b_deg=target_b_deg,
            minimum_b_deg=args.min_b_deg,
            maximum_b_deg=args.max_b_deg,
            maximum_delta_deg=args.max_delta_deg,
        )
        planned_command = arm.build_b_joint_command(
            target_b_deg,
            finite_float(args.speed_deg_s, "speed_deg_s"),
            finite_float(args.acceleration, "acceleration"),
        )
        report.update(
            {
                "initial_state_after_baseline": compact_state(initial_state),
                "initial_b_deg_after_baseline": initial_b_deg,
                "target_b_deg": target_b_deg,
                "requested_delta_deg": requested_delta_deg,
                "planned_command": planned_command,
            }
        )
        if not args.enable_motion:
            report["summary"] = {
                "valid": True,
                "reason": "baseline_complete_dry_run_checks_passed",
                "motion_command_sent": False,
                "comparison_complete": False,
            }
            print(json.dumps(report, ensure_ascii=False, indent=2))
            return 0

        sent_command = arm.send_b_joint_command(
            target_b_deg, float(args.speed_deg_s), float(args.acceleration)
        )
        observe_start = time.monotonic()
        observed_b_deg: List[float] = []
        final_motion_state = None
        while time.monotonic() - observe_start < motion_observe_s:
            state = arm.read_state(timeout_s=0.25)
            if state is None:
                continue
            final_motion_state = state
            observed_b_deg.append(b_degrees_from_state(state))
        report["motion"] = {
            "sent_command": sent_command,
            "transmitted_command_count": arm.transmitted_command_count,
            "transmitted_byte_count": arm.transmitted_byte_count,
            "observed_b_deg": distribution(observed_b_deg),
            "final_state_before_second_group": compact_state(final_motion_state),
        }

        second = collect_group(
            name="after_motion",
            arm=arm,
            camera=camera,
            detector=detector,
            estimator=estimator,
            matrix_eef_camera=handeye.matrix_eef_camera,
            tag_id=tag_id,
            sample_count=sample_count,
            max_attempts=max_attempts,
            state_timeout_s=float(args.state_timeout_s),
            frame_timeout_ms=int(args.frame_timeout_ms),
        )
        report["groups"]["after_motion"] = second
        if second["valid"]:
            baseline_base = median_xyz(baseline["statistics"], "base_tag_mm")
            second_base = median_xyz(second["statistics"], "base_tag_mm")
            delta = second_base - baseline_base
            report["comparison"] = {
                "base_tag_median_delta_mm": {
                    "x": float(delta[0]),
                    "y": float(delta[1]),
                    "z": float(delta[2]),
                    "norm": float(np.linalg.norm(delta)),
                },
                "baseline_base_tag_median_mm": baseline_base.tolist(),
                "after_motion_base_tag_median_mm": second_base.tolist(),
            }
        complete = baseline["valid"] and second["valid"]
        report["summary"] = {
            "valid": complete,
            "reason": "paired_groups_complete" if complete else "after_motion_incomplete",
            "motion_command_sent": True,
            "comparison_complete": complete,
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if complete else 1
    except Exception as exc:
        report["summary"] = {
            "valid": False,
            "reason": "validation_or_execution_failed",
            "motion_command_sent": arm.transmitted_byte_count > 0,
            "error": f"{type(exc).__name__}: {exc}",
        }
        report["transmitted_command_count"] = arm.transmitted_command_count
        report["transmitted_byte_count"] = arm.transmitted_byte_count
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 1
    finally:
        camera.stop()
        arm.close()


if __name__ == "__main__":
    raise SystemExit(main())
