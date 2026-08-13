#!/usr/bin/env python3
"""Compare AprilTag PnP Z with aligned RGBD depth without arm access."""

import json
from collections import defaultdict
from typing import Any, Dict, List

import numpy as np

from apriltag_block_grasp.core.apriltag_detector import OpenCvAprilTag25h9Detector
from apriltag_block_grasp.core.camera_calibration import read_orbbec_color_calibration
from apriltag_block_grasp.core.camera_rgbd_orbbec import OrbbecRgbdCamera
from apriltag_block_grasp.core.pose_estimator import AprilTagPoseEstimator


def sample_center_depth(
    depth_mm: np.ndarray, u: float, v: float, radius_px: int = 2
) -> Dict[str, Any]:
    height, width = depth_mm.shape
    center_u = int(round(u))
    center_v = int(round(v))
    x0 = max(0, center_u - radius_px)
    x1 = min(width, center_u + radius_px + 1)
    y0 = max(0, center_v - radius_px)
    y1 = min(height, center_v + radius_px + 1)
    region = depth_mm[y0:y1, x0:x1]
    valid = region[np.isfinite(region) & (region > 0.0)]
    sample_count = int(region.size)
    valid_count = int(valid.size)
    return {
        "sample_count": sample_count,
        "valid_count": valid_count,
        "valid_ratio": (float(valid_count) / sample_count) if sample_count else 0.0,
        "median_depth_mm": float(np.median(valid)) if valid_count else None,
    }


def distribution(values: List[float]) -> Dict[str, Any]:
    if not values:
        return {"count": 0}
    data = np.asarray(values, dtype=np.float64)
    return {
        "count": int(data.size),
        "min": float(np.min(data)),
        "median": float(np.median(data)),
        "max": float(np.max(data)),
        "mean": float(np.mean(data)),
    }


def main() -> int:
    camera = OrbbecRgbdCamera()
    report: Dict[str, Any] = {
        "tool": "apriltag_block_grasp.probe_pnp_depth_consistency",
        "read_only": True,
        "tag_size_mm": 38.9,
        "pnp_position_source": True,
        "rgbd_usage": "validation_only",
        "depth_rejection_threshold_enabled": False,
        "arm_connected": False,
        "motion_commands_enabled": False,
        "samples": [],
        "per_id_summary": {},
        "summary": {"comparison_observed": False},
    }
    try:
        camera.start()
        if not camera.alignment_requested:
            raise RuntimeError(
                "software depth alignment request failed: "
                + str(camera.alignment_error)
            )
        calibration = read_orbbec_color_calibration(camera)
        detector = OpenCvAprilTag25h9Detector(allowed_ids=(0, 1))
        estimator = AprilTagPoseEstimator(38.9, calibration)

        valid_rgbd_frames = 0
        empty_frames = 0
        resolution_mismatch_frames = 0
        pnp_failures = 0
        no_depth_samples = 0
        samples: List[Dict[str, Any]] = []

        for frame_index in range(60):
            frame = camera.read(timeout_ms=500)
            if frame is None:
                empty_frames += 1
                continue
            valid_rgbd_frames += 1
            color_height, color_width = frame.color.bgr.shape[:2]
            depth_height, depth_width = frame.depth_mm.shape
            if (color_width, color_height) != (depth_width, depth_height):
                resolution_mismatch_frames += 1
                continue
            if not calibration.matches_frame(color_width, color_height):
                resolution_mismatch_frames += 1
                continue

            batch = detector.detect(frame.color.bgr)
            for detection in batch.detections:
                try:
                    pose = estimator.estimate(detection)
                except Exception:
                    pnp_failures += 1
                    continue
                depth = sample_center_depth(
                    frame.depth_mm, detection.center[0], detection.center[1]
                )
                rgbd_depth_mm = depth["median_depth_mm"]
                if rgbd_depth_mm is None:
                    no_depth_samples += 1
                    continue
                pnp_z_mm = float(pose.translation_mm[2])
                signed_difference_mm = float(pnp_z_mm - rgbd_depth_mm)
                samples.append(
                    {
                        "frame_index": int(frame_index),
                        "tag_id": int(detection.tag_id),
                        "center": {
                            "u": float(detection.center[0]),
                            "v": float(detection.center[1]),
                        },
                        "pnp_z_mm": pnp_z_mm,
                        "rgbd_depth_mm": float(rgbd_depth_mm),
                        "pnp_minus_rgbd_mm": signed_difference_mm,
                        "absolute_difference_mm": abs(signed_difference_mm),
                        "depth_valid_ratio": depth["valid_ratio"],
                        "reprojection_error_px": float(pose.reprojection_error_px),
                    }
                )

        values_by_id = defaultdict(lambda: {"signed": [], "absolute": [], "pnp": [], "rgbd": []})
        for sample in samples:
            values = values_by_id[sample["tag_id"]]
            values["signed"].append(sample["pnp_minus_rgbd_mm"])
            values["absolute"].append(sample["absolute_difference_mm"])
            values["pnp"].append(sample["pnp_z_mm"])
            values["rgbd"].append(sample["rgbd_depth_mm"])

        report["calibration_source"] = calibration.source
        report["pnp_distortion_mode"] = estimator.distortion_mode
        report["pnp_distortion_coefficients"] = (
            estimator.distortion_coefficients.reshape(-1).tolist()
        )
        report["frames"] = {
            "requested_count": 60,
            "valid_rgbd_count": valid_rgbd_frames,
            "empty_count": empty_frames,
            "resolution_mismatch_count": resolution_mismatch_frames,
        }
        report["samples"] = samples[-20:]
        report["per_id_summary"] = {
            str(tag_id): {
                "pnp_z_mm": distribution(values["pnp"]),
                "rgbd_depth_mm": distribution(values["rgbd"]),
                "pnp_minus_rgbd_mm": distribution(values["signed"]),
                "absolute_difference_mm": distribution(values["absolute"]),
            }
            for tag_id, values in sorted(values_by_id.items())
        }
        report["summary"] = {
            "comparison_observed": bool(samples),
            "comparison_sample_count": len(samples),
            "pnp_failure_count": pnp_failures,
            "no_valid_depth_count": no_depth_samples,
            "threshold_selected": False,
            "note": "RGBD is validation-only; this probe does not reject PnP results",
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if samples else 1
    except Exception as exc:
        report["summary"] = {
            "comparison_observed": False,
            "error": f"{type(exc).__name__}: {exc}",
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 1
    finally:
        try:
            camera.stop()
            print("Orbbec RGBD stream stopped.", flush=True)
        except Exception as exc:
            print(f"Failed to stop camera cleanly: {exc}", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
