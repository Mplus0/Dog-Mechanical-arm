#!/usr/bin/env python3
"""Read-only validation of base<-eef<-camera<-tag transform chaining."""

import argparse
import json
from collections import defaultdict
from typing import Any, Dict, List

import numpy as np
from ament_index_python.packages import get_package_share_directory

from apriltag_block_grasp.core.apriltag_detector import OpenCvAprilTag25h9Detector
from apriltag_block_grasp.core.camera_calibration import read_orbbec_color_calibration
from apriltag_block_grasp.core.camera_color_orbbec import OrbbecColorCamera
from apriltag_block_grasp.core.handeye import load_handeye_calibration
from apriltag_block_grasp.core.pose_estimator import AprilTagPoseEstimator
from apriltag_block_grasp.core.roarm_serial_readonly import RoArmSerialStateReader
from apriltag_block_grasp.core.roarm_state import cartesian_pose_from_state
from apriltag_block_grasp.core.tag_to_object import load_tag_to_object_calibration


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


def parse_arguments():
    default_handeye = (
        get_package_share_directory("apriltag_block_grasp")
        + "/config/handeye_cam_to_eef.json"
    )
    default_tag_to_object = (
        get_package_share_directory("apriltag_block_grasp")
        + "/config/tag_to_object.json"
    )
    parser = argparse.ArgumentParser(
        description="Read-only validation of T_base_eef*T_eef_camera*T_camera_tag."
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--handeye-path", default=default_handeye)
    parser.add_argument("--tag-to-object-path", default=default_tag_to_object)
    parser.add_argument("--sample-count", type=int, default=20)
    parser.add_argument("--max-attempts", type=int, default=80)
    parser.add_argument("--state-timeout-s", type=float, default=0.5)
    parser.add_argument("--frame-timeout-ms", type=int, default=500)
    parser.add_argument("--tag-size-mm", type=float, default=38.9)
    parser.add_argument(
        "--wait-for-ready",
        action="store_true",
        help=(
            "open the arm serial port first, then wait for Enter before opening "
            "the camera and collecting samples"
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    requested_count = max(1, int(args.sample_count))
    max_attempts = max(requested_count, int(args.max_attempts))
    camera = OrbbecColorCamera()
    arm = RoArmSerialStateReader(port=args.port, timeout_s=0.2)
    report: Dict[str, Any] = {
        "tool": "apriltag_block_grasp.probe_handeye_chain",
        "read_only": True,
        "formula": {
            "base_tag": "T_base_tag = T_base_eef @ T_eef_camera @ T_camera_tag",
            "base_object": "T_base_object = T_base_tag @ T_tag_object",
        },
        "tag_size_mm": float(args.tag_size_mm),
        "tag_to_object_applied": True,
        "base_position_correction_applied": False,
        "legacy_base_z_plus_100_applied": False,
        "depth_enabled": False,
        "serial_bytes_transmitted": 0,
        "serial_open_can_reset_controller": True,
        "serial_input_buffer_cleared_after_ready": False,
        "motion_commands_enabled": False,
        "samples": [],
        "summary": {"valid": False},
    }
    try:
        handeye = load_handeye_calibration(args.handeye_path)
        tag_to_object = load_tag_to_object_calibration(args.tag_to_object_path)
        report["handeye"] = {
            "path": handeye.path,
            "definition": "X_eef = T_eef_camera * X_camera",
            "translation_unit": "mm",
            "T_eef_camera": handeye.matrix_eef_camera.tolist(),
            "rotation_orthogonality_error": handeye.orthogonality_error,
            "rotation_determinant": handeye.determinant,
            "metadata": handeye.metadata,
        }
        report["tag_to_object"] = {
            "path": tag_to_object.path,
            "definition": "X_tag = T_tag_object * X_object",
            "translation_unit": "mm",
            "rotation_unit": "deg",
            "translation_mm": tag_to_object.translation_mm.tolist(),
            "rotation_rpy_deg": tag_to_object.rotation_rpy_deg.tolist(),
            "T_tag_object": tag_to_object.matrix_tag_object.tolist(),
            "rotation_orthogonality_error": tag_to_object.orthogonality_error,
            "rotation_determinant": tag_to_object.determinant,
            "metadata": tag_to_object.metadata,
        }

        arm.connect()
        if args.wait_for_ready:
            print(
                "\nArm serial is now open. RoArm-M3 may have reset during open().\n"
                "Adjust the arm to the observation pose now and place the fixed "
                "Tag/block pair.\n"
                "Stop any other program using the Orbbec camera, then press Enter "
                "to start read-only sampling.\n",
                flush=True,
            )
            input()
            # State frames continue arriving while the operator adjusts the arm.
            # They describe old poses and must never be paired with camera frames
            # captured after Enter is pressed.
            arm.reset_input_buffer()
            report["serial_input_buffer_cleared_after_ready"] = True
        camera.start()
        calibration = read_orbbec_color_calibration(camera)
        detector = OpenCvAprilTag25h9Detector(allowed_ids=(0, 1))
        estimator = AprilTagPoseEstimator(float(args.tag_size_mm), calibration)

        samples = []
        failure_counts = defaultdict(int)
        for attempt_index in range(max_attempts):
            if len(samples) >= requested_count:
                break
            state = arm.read_state(timeout_s=float(args.state_timeout_s))
            if state is None:
                failure_counts["empty_arm_state"] += 1
                continue
            try:
                arm_pose = cartesian_pose_from_state(state)
            except Exception:
                failure_counts["invalid_arm_state"] += 1
                continue
            frame = camera.read(timeout_ms=int(args.frame_timeout_ms))
            if frame is None:
                failure_counts["empty_color_frame"] += 1
                continue
            height, width = frame.bgr.shape[:2]
            if not calibration.matches_frame(width, height):
                failure_counts["calibration_resolution_mismatch"] += 1
                continue
            batch = detector.detect(frame.bgr)
            if not batch.detections:
                failure_counts["no_allowed_tag"] += 1
                continue

            for detection in batch.detections:
                if len(samples) >= requested_count:
                    break
                try:
                    pose = estimator.estimate(detection)
                except Exception:
                    failure_counts["pnp_failed"] += 1
                    continue
                matrix_camera_tag = np.eye(4, dtype=np.float64)
                matrix_camera_tag[:3, :3] = pose.rotation_matrix
                matrix_camera_tag[:3, 3] = pose.translation_mm
                matrix_base_camera = (
                    arm_pose.matrix_base_eef @ handeye.matrix_eef_camera
                )
                matrix_base_tag = matrix_base_camera @ matrix_camera_tag
                matrix_base_object = (
                    matrix_base_tag @ tag_to_object.matrix_tag_object
                )
                if not np.all(np.isfinite(matrix_base_object)):
                    failure_counts["non_finite_chain"] += 1
                    continue
                samples.append(
                    {
                        "sample_index": len(samples),
                        "read_attempt_index": attempt_index,
                        "tag_id": int(detection.tag_id),
                        "arm_pose_mm_rad": {
                            "x": arm_pose.x_mm,
                            "y": arm_pose.y_mm,
                            "z": arm_pose.z_mm,
                            "roll": arm_pose.roll_rad,
                            "pitch": arm_pose.pitch_rad,
                            "yaw": arm_pose.yaw_rad,
                            "pitch_source_field": arm_pose.pitch_source_field,
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
                        "base_object_mm": {
                            "x": float(matrix_base_object[0, 3]),
                            "y": float(matrix_base_object[1, 3]),
                            "z": float(matrix_base_object[2, 3]),
                        },
                        "reprojection_error_px": float(pose.reprojection_error_px),
                        "T_base_eef": arm_pose.matrix_base_eef.tolist(),
                        "T_base_camera": matrix_base_camera.tolist(),
                        "T_camera_tag": matrix_camera_tag.tolist(),
                        "T_base_tag": matrix_base_tag.tolist(),
                        "T_tag_object": tag_to_object.matrix_tag_object.tolist(),
                        "T_base_object": matrix_base_object.tolist(),
                    }
                )

        by_id = defaultdict(
            lambda: {
                "tag_x": [],
                "tag_y": [],
                "tag_z": [],
                "object_x": [],
                "object_y": [],
                "object_z": [],
                "error": [],
            }
        )
        for sample in samples:
            values = by_id[sample["tag_id"]]
            for axis in ("x", "y", "z"):
                values[f"tag_{axis}"].append(sample["base_tag_mm"][axis])
                values[f"object_{axis}"].append(sample["base_object_mm"][axis])
            values["error"].append(sample["reprojection_error_px"])
        report["samples"] = samples[-10:]
        report["per_id_stability"] = {
            str(tag_id): {
                "base_tag_mm": {
                    axis: distribution(values[f"tag_{axis}"])
                    for axis in ("x", "y", "z")
                },
                "base_object_mm": {
                    axis: distribution(values[f"object_{axis}"])
                    for axis in ("x", "y", "z")
                },
                "reprojection_error_px": distribution(values["error"]),
            }
            for tag_id, values in sorted(by_id.items())
        }
        report["calibration_source"] = calibration.source
        report["pnp_distortion_mode"] = estimator.distortion_mode
        report["pnp_distortion_coefficients"] = (
            estimator.distortion_coefficients.reshape(-1).tolist()
        )
        report["failure_counts"] = dict(sorted(failure_counts.items()))
        report["summary"] = {
            "valid": len(samples) == requested_count,
            "requested_sample_count": requested_count,
            "valid_chain_sample_count": len(samples),
            "max_attempts": max_attempts,
            "observed_tag_ids": sorted(int(value) for value in by_id.keys()),
            "motion_command_sent": False,
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report["summary"]["valid"] else 1
    except Exception as exc:
        report["summary"] = {
            "valid": False,
            "motion_command_sent": False,
            "error": f"{type(exc).__name__}: {exc}",
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 1
    finally:
        camera.stop()
        arm.close()
        print("Camera and read-only arm serial connection closed; no motion command was sent.", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
