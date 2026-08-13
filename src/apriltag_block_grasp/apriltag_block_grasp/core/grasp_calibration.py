"""Load and validate provisional fixed grasp calibration parameters."""

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np


@dataclass(frozen=True)
class GraspCalibration:
    final_grasp_tcp_offset_base_mm: np.ndarray
    grasp_tool_orientation_rpy_rad: np.ndarray
    reference_clamp_g_rad: float
    base_position_correction_mm: Optional[np.ndarray]
    metadata: Dict[str, Any]
    path: str

    def final_grasp_tcp_base_mm(self, base_object_mm) -> np.ndarray:
        object_position = np.asarray(base_object_mm, dtype=np.float64)
        if object_position.shape != (3,):
            raise ValueError(
                "base_object_mm must contain exactly three values, "
                f"got {object_position.shape}"
            )
        if not np.all(np.isfinite(object_position)):
            raise ValueError("base_object_mm contains non-finite values")
        correction = (
            np.zeros(3, dtype=np.float64)
            if self.base_position_correction_mm is None
            else self.base_position_correction_mm
        )
        return object_position + correction + self.final_grasp_tcp_offset_base_mm


def _finite_vector(data: Dict[str, Any], key: str) -> np.ndarray:
    if key not in data:
        raise KeyError(f"grasp calibration file must contain {key}")
    values = np.asarray(data[key], dtype=np.float64)
    if values.shape != (3,):
        raise ValueError(f"{key} must contain exactly three values, got {values.shape}")
    if not np.all(np.isfinite(values)):
        raise ValueError(f"{key} contains non-finite values")
    return values


def load_grasp_calibration(path: str) -> GraspCalibration:
    calibration_path = Path(path)
    with calibration_path.open("r", encoding="utf-8") as stream:
        data = json.load(stream)

    tcp_offset = _finite_vector(data, "final_grasp_tcp_offset_base_mm")
    orientation = _finite_vector(data, "grasp_tool_orientation_rpy_rad")

    gripper = data.get("gripper")
    if not isinstance(gripper, dict) or "reference_clamp_g_rad" not in gripper:
        raise KeyError("grasp calibration must contain gripper.reference_clamp_g_rad")
    clamp_g_rad = float(gripper["reference_clamp_g_rad"])
    if not np.isfinite(clamp_g_rad):
        raise ValueError("gripper.reference_clamp_g_rad must be finite")

    raw_correction = data.get("base_position_correction_mm")
    correction = None
    if raw_correction is not None:
        correction = np.asarray(raw_correction, dtype=np.float64)
        if correction.shape != (3,) or not np.all(np.isfinite(correction)):
            raise ValueError(
                "base_position_correction_mm must be null or three finite values"
            )

    excluded = {
        "final_grasp_tcp_offset_base_mm",
        "grasp_tool_orientation_rpy_rad",
        "base_position_correction_mm",
    }
    metadata = {key: value for key, value in data.items() if key not in excluded}
    return GraspCalibration(
        final_grasp_tcp_offset_base_mm=tcp_offset,
        grasp_tool_orientation_rpy_rad=orientation,
        reference_clamp_g_rad=clamp_g_rad,
        base_position_correction_mm=correction,
        metadata=metadata,
        path=str(calibration_path),
    )
