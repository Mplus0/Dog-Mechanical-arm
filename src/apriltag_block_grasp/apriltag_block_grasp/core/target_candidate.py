"""Pure conversion from PnP detections and RoArm state to base-frame candidates."""

import math
from typing import Any, Dict, Iterable, List

import numpy as np

from apriltag_block_grasp.core.rigid_transform import make_transform
from apriltag_block_grasp.core.roarm_state import cartesian_pose_from_state


def _finite(value: Any, label: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


def build_base_object_candidates(
    detections: Iterable[Dict[str, Any]],
    arm_state: Dict[str, Any],
    matrix_eef_camera: np.ndarray,
    matrix_tag_object: np.ndarray,
    allowed_ids=(0, 1),
) -> List[Dict[str, Any]]:
    """Return valid ID 0/1 block-center candidates in the arm base frame."""

    allowed = {int(value) for value in allowed_ids}
    arm_pose = cartesian_pose_from_state(arm_state)
    eef_camera = np.asarray(matrix_eef_camera, dtype=np.float64)
    tag_object = np.asarray(matrix_tag_object, dtype=np.float64)
    for label, matrix in (
        ("T_eef_camera", eef_camera),
        ("T_tag_object", tag_object),
    ):
        if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
            raise ValueError(f"{label} must be a finite 4x4 matrix")

    matrix_base_camera = arm_pose.matrix_base_eef @ eef_camera
    candidates: List[Dict[str, Any]] = []
    seen = set()
    for detection in detections:
        if not isinstance(detection, dict) or not detection.get("valid", False):
            continue
        tag_id = int(detection.get("tag_id", -1))
        if tag_id not in allowed or tag_id in seen:
            continue
        translation = detection.get("camera_tag_mm")
        rotation = detection.get("rotation_matrix")
        if not isinstance(translation, dict):
            continue
        try:
            translation_mm = [
                _finite(translation[axis], f"camera_tag_mm.{axis}")
                for axis in ("x", "y", "z")
            ]
            rotation_matrix = np.asarray(rotation, dtype=np.float64)
            if rotation_matrix.shape != (3, 3) or not np.all(
                np.isfinite(rotation_matrix)
            ):
                raise ValueError("rotation_matrix must be finite 3x3")
            matrix_camera_tag = make_transform(rotation_matrix, translation_mm)
            matrix_base_tag = matrix_base_camera @ matrix_camera_tag
            matrix_base_object = matrix_base_tag @ tag_object
            if not np.all(np.isfinite(matrix_base_object)):
                raise ValueError("T_base_object contains non-finite values")
            reprojection_error_px = (
                None
                if detection.get("reprojection_error_px") is None
                else _finite(
                    detection["reprojection_error_px"], "reprojection_error_px"
                )
            )
            area_px2 = (
                None
                if detection.get("area_px2") is None
                else _finite(detection["area_px2"], "area_px2")
            )
        except (KeyError, TypeError, ValueError):
            continue

        position = matrix_base_object[:3, 3]
        candidates.append(
            {
                "tag_id": tag_id,
                "base_object_mm": {
                    "x": float(position[0]),
                    "y": float(position[1]),
                    "z": float(position[2]),
                },
                "camera_tag_mm": {
                    axis: float(translation[axis]) for axis in ("x", "y", "z")
                },
                "reprojection_error_px": reprojection_error_px,
                "area_px2": area_px2,
            }
        )
        seen.add(tag_id)
    return sorted(candidates, key=lambda item: item["tag_id"])
