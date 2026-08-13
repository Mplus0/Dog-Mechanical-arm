#!/usr/bin/env python3
"""Probe RGBD alignment and tag-center depth without arm access."""

import json
from typing import Any, Dict, List

import numpy as np

from apriltag_block_grasp.core.apriltag_detector import (
    OpenCvAprilTag25h9Detector,
)
from apriltag_block_grasp.core.camera_rgbd_orbbec import OrbbecRgbdCamera


def median_depth_near(depth_mm: np.ndarray, u: float, v: float, radius_px: int) -> Dict[str, Any]:
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
        "center": {"u": float(u), "v": float(v)},
        "window": {"radius_px": int(radius_px), "x0": x0, "y0": y0, "x1": x1, "y1": y1},
        "sample_count": sample_count,
        "valid_count": valid_count,
        "valid_ratio": (float(valid_count) / sample_count) if sample_count else 0.0,
        "median_depth_mm": float(np.median(valid)) if valid_count else None,
    }


def main() -> int:
    camera = OrbbecRgbdCamera()
    detector = OpenCvAprilTag25h9Detector(allowed_ids=(0, 1))
    report: Dict[str, Any] = {
        "tool": "apriltag_block_grasp.probe_rgbd_alignment",
        "read_only": True,
        "pnp_position_source": False,
        "arm_connected": False,
        "motion_commands_enabled": False,
        "alignment": {},
        "frames": {},
        "tag_center_depths": [],
        "summary": {"ready_for_depth_consistency_check": False},
    }
    try:
        camera.start()
        report["alignment"] = {
            "requested_mode": "SW_MODE",
            "request_succeeded": camera.alignment_requested,
            "request_error": camera.alignment_error,
        }

        valid_frames = 0
        empty_frames = 0
        decode_failures = 0
        same_resolution_frames = 0
        depth_scales: List[float] = []
        latest_color_size = None
        latest_depth_size = None
        tag_center_depths: List[Dict[str, Any]] = []

        for _ in range(60):
            frame = camera.read(timeout_ms=500)
            if frame is None:
                empty_frames += 1
                continue
            valid_frames += 1
            color_height, color_width = frame.color.bgr.shape[:2]
            depth_height, depth_width = frame.depth_mm.shape
            latest_color_size = {"width": int(color_width), "height": int(color_height)}
            latest_depth_size = {"width": int(depth_width), "height": int(depth_height)}
            depth_scales.append(float(frame.depth_scale_mm))
            same_resolution = color_width == depth_width and color_height == depth_height
            if same_resolution:
                same_resolution_frames += 1
                detection_result = detector.detect(frame.color.bgr)
                for detection in detection_result.detections:
                    sample = median_depth_near(
                        frame.depth_mm,
                        detection.center[0],
                        detection.center[1],
                        radius_px=2,
                    )
                    sample["tag_id"] = int(detection.tag_id)
                    tag_center_depths.append(sample)
            else:
                decode_failures += 1

        report["frames"] = {
            "requested_count": 60,
            "valid_count": valid_frames,
            "empty_count": empty_frames,
            "resolution_mismatch_count": decode_failures,
            "same_resolution_count": same_resolution_frames,
            "color_size": latest_color_size,
            "depth_size": latest_depth_size,
            "depth_scale_mm_values": sorted(set(depth_scales)),
        }
        report["tag_center_depths"] = tag_center_depths[-20:]

        has_aligned_frames = (
            camera.alignment_requested
            and valid_frames > 0
            and same_resolution_frames == valid_frames
        )
        has_tag_depth = any(
            item["median_depth_mm"] is not None and item["valid_ratio"] > 0.0
            for item in tag_center_depths
        )
        report["summary"] = {
            "ready_for_depth_consistency_check": bool(has_aligned_frames and has_tag_depth),
            "aligned_frame_shape_check_passed": bool(has_aligned_frames),
            "tag_center_valid_depth_observed": bool(has_tag_depth),
            "note": "shape equality is a necessary probe result; semantic D2C alignment still requires field comparison",
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report["summary"]["ready_for_depth_consistency_check"] else 1
    except Exception as exc:
        report["summary"] = {
            "ready_for_depth_consistency_check": False,
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
