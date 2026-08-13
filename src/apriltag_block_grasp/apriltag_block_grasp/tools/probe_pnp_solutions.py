#!/usr/bin/env python3
"""Inspect multiple square-PnP solutions against aligned depth without arm access."""

import argparse
import json
from collections import defaultdict
from typing import Any, Dict, List, Optional

import cv2
import numpy as np

from apriltag_block_grasp.core.apriltag_detector import OpenCvAprilTag25h9Detector
from apriltag_block_grasp.core.camera_calibration import read_orbbec_color_calibration
from apriltag_block_grasp.core.camera_rgbd_orbbec import OrbbecRgbdCamera
from apriltag_block_grasp.tools.probe_pnp_depth_consistency import sample_center_depth


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


def reprojection_error(
    object_points: np.ndarray,
    image_points: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
) -> float:
    projected, _ = cv2.projectPoints(
        object_points,
        np.asarray(rvec, dtype=np.float64).reshape(3, 1),
        np.asarray(tvec, dtype=np.float64).reshape(3, 1),
        camera_matrix,
        distortion,
    )
    residual = projected.reshape(4, 2) - image_points
    return float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))


def make_solution(
    *,
    name: str,
    object_points: np.ndarray,
    image_points: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
) -> Optional[Dict[str, Any]]:
    rotation_vector = np.asarray(rvec, dtype=np.float64).reshape(3)
    translation = np.asarray(tvec, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(np.concatenate((rotation_vector, translation)))):
        return None
    return {
        "name": name,
        "camera_tag_mm": {
            "x": float(translation[0]),
            "y": float(translation[1]),
            "z": float(translation[2]),
        },
        "positive_z": bool(translation[2] > 0.0),
        "reprojection_error_px": reprojection_error(
            object_points,
            image_points,
            camera_matrix,
            distortion,
            rotation_vector,
            translation,
        ),
        "rvec": rotation_vector.tolist(),
    }


def solve_single(
    *,
    name: str,
    flag: int,
    object_points: np.ndarray,
    image_points: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> List[Dict[str, Any]]:
    try:
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            distortion,
            flags=flag,
        )
    except cv2.error:
        return []
    if not success:
        return []
    solution = make_solution(
        name=name,
        object_points=object_points,
        image_points=image_points,
        camera_matrix=camera_matrix,
        distortion=distortion,
        rvec=rvec,
        tvec=tvec,
    )
    return [solution] if solution is not None else []


def solve_ippe_all(
    *,
    object_points: np.ndarray,
    image_points: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    suffix: str,
) -> List[Dict[str, Any]]:
    try:
        result = cv2.solvePnPGeneric(
            object_points,
            image_points,
            camera_matrix,
            distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
    except (cv2.error, AttributeError):
        return []
    if len(result) < 3 or not bool(result[0]):
        return []
    rvecs, tvecs = result[1], result[2]
    solutions = []
    for index, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
        solution = make_solution(
            name=f"IPPE_SQUARE_{suffix}_candidate_{index}",
            object_points=object_points,
            image_points=image_points,
            camera_matrix=camera_matrix,
            distortion=distortion,
            rvec=rvec,
            tvec=tvec,
        )
        if solution is not None:
            solutions.append(solution)
    return solutions


def solve_variants(
    image_points: np.ndarray,
    camera_matrix: np.ndarray,
    calibrated_distortion: np.ndarray,
    tag_size_mm: float,
) -> List[Dict[str, Any]]:
    half = tag_size_mm / 2.0
    ippe_points = np.asarray(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )
    solutions: List[Dict[str, Any]] = []
    distortion_variants = (
        ("calibrated_distortion", calibrated_distortion),
        ("zero_distortion_diagnostic", np.zeros((5, 1), dtype=np.float64)),
    )
    for suffix, distortion in distortion_variants:
        solutions.extend(
            solve_ippe_all(
                object_points=ippe_points,
                image_points=image_points,
                camera_matrix=camera_matrix,
                distortion=distortion,
                suffix=suffix,
            )
        )
        solutions.extend(
            solve_single(
                name=f"ITERATIVE_{suffix}",
                flag=cv2.SOLVEPNP_ITERATIVE,
                object_points=ippe_points,
                image_points=image_points,
                camera_matrix=camera_matrix,
                distortion=distortion,
            )
        )
        if hasattr(cv2, "SOLVEPNP_SQPNP"):
            solutions.extend(
                solve_single(
                    name=f"SQPNP_{suffix}",
                    flag=cv2.SOLVEPNP_SQPNP,
                    object_points=ippe_points,
                    image_points=image_points,
                    camera_matrix=camera_matrix,
                    distortion=distortion,
                )
            )
    return solutions


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Compare square-PnP solution variants and aligned center depth."
    )
    parser.add_argument("--tag-id", type=int, choices=(0, 1), required=True)
    parser.add_argument("--tag-size-mm", type=float, default=38.9)
    parser.add_argument("--frame-count", type=int, default=30)
    parser.add_argument("--frame-timeout-ms", type=int, default=500)
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    tag_id = int(args.tag_id)
    tag_size_mm = float(args.tag_size_mm)
    if not np.isfinite(tag_size_mm) or tag_size_mm <= 0.0:
        raise ValueError("tag_size_mm must be finite and positive")
    frame_count = max(1, int(args.frame_count))
    camera = OrbbecRgbdCamera()
    report: Dict[str, Any] = {
        "tool": "apriltag_block_grasp.probe_pnp_solutions",
        "read_only": True,
        "tag_id": tag_id,
        "tag_size_mm": tag_size_mm,
        "zero_distortion_usage": "diagnostic_only",
        "depth_usage": "diagnostic_only",
        "arm_connected": False,
        "motion_commands_enabled": False,
        "samples": [],
        "summary": {"valid": False},
    }
    try:
        camera.start()
        if not camera.alignment_requested:
            raise RuntimeError(
                "software depth alignment request failed: "
                + str(camera.alignment_error)
            )
        calibration = read_orbbec_color_calibration(camera)
        detector = OpenCvAprilTag25h9Detector(allowed_ids=(tag_id,))
        samples = []
        failure_counts = defaultdict(int)
        for frame_index in range(frame_count):
            frame = camera.read(timeout_ms=int(args.frame_timeout_ms))
            if frame is None:
                failure_counts["empty_rgbd_frame"] += 1
                continue
            height, width = frame.color.bgr.shape[:2]
            if frame.depth_mm.shape != (height, width):
                failure_counts["rgbd_resolution_mismatch"] += 1
                continue
            if not calibration.matches_frame(width, height):
                failure_counts["calibration_resolution_mismatch"] += 1
                continue
            batch = detector.detect(frame.color.bgr)
            detection = next(
                (item for item in batch.detections if int(item.tag_id) == tag_id),
                None,
            )
            if detection is None:
                failure_counts["requested_tag_not_detected"] += 1
                continue
            image_points = np.asarray(detection.corners, dtype=np.float64).reshape(4, 2)
            rolled = np.roll(image_points, -1, axis=0)
            edge_lengths = np.linalg.norm(rolled - image_points, axis=1)
            depth = sample_center_depth(
                frame.depth_mm, detection.center[0], detection.center[1]
            )
            solutions = solve_variants(
                image_points,
                calibration.camera_matrix,
                calibration.distortion_coefficients,
                tag_size_mm,
            )
            if not solutions:
                failure_counts["all_pnp_variants_failed"] += 1
                continue
            samples.append(
                {
                    "frame_index": frame_index,
                    "center": {
                        "u": float(detection.center[0]),
                        "v": float(detection.center[1]),
                    },
                    "corners": image_points.tolist(),
                    "edge_lengths_px": edge_lengths.tolist(),
                    "area_px2": float(detection.area_px2),
                    "aligned_center_depth": depth,
                    "solutions": solutions,
                }
            )

        by_solution = defaultdict(lambda: {"z": [], "error": []})
        edge_values: List[float] = []
        depth_values: List[float] = []
        for sample in samples:
            edge_values.extend(sample["edge_lengths_px"])
            depth_mm = sample["aligned_center_depth"]["median_depth_mm"]
            if depth_mm is not None:
                depth_values.append(float(depth_mm))
            for solution in sample["solutions"]:
                values = by_solution[solution["name"]]
                values["z"].append(solution["camera_tag_mm"]["z"])
                values["error"].append(solution["reprojection_error_px"])
        report["calibration_source"] = calibration.source
        report["calibration_distortion"] = calibration.distortion_coefficients.reshape(-1).tolist()
        report["samples"] = samples[-5:]
        report["geometry_summary"] = {
            "edge_length_px": distribution(edge_values),
            "aligned_center_depth_mm": distribution(depth_values),
        }
        report["solution_summary"] = {
            name: {
                "camera_tag_z_mm": distribution(values["z"]),
                "reprojection_error_px": distribution(values["error"]),
            }
            for name, values in sorted(by_solution.items())
        }
        report["failure_counts"] = dict(sorted(failure_counts.items()))
        report["summary"] = {
            "valid": bool(samples),
            "requested_frame_count": frame_count,
            "valid_sample_count": len(samples),
            "motion_command_sent": False,
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if samples else 1
    except Exception as exc:
        report["summary"] = {
            "valid": False,
            "motion_command_sent": False,
            "error": f"{type(exc).__name__}: {exc}",
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 1
    finally:
        try:
            camera.stop()
        finally:
            print("Orbbec RGBD stream stopped; arm was not connected.", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
