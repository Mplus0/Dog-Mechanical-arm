"""Load and validate the standalone eye-in-hand calibration."""

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict

import numpy as np

from apriltag_block_grasp.core.rigid_transform import rotation_quality


@dataclass(frozen=True)
class HandEyeCalibration:
    matrix_eef_camera: np.ndarray
    metadata: Dict[str, Any]
    path: str
    orthogonality_error: float
    determinant: float


def load_handeye_calibration(path: str) -> HandEyeCalibration:
    calibration_path = Path(path)
    with calibration_path.open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    if "T_eef_camera" not in data:
        raise KeyError("hand-eye file must contain T_eef_camera")

    matrix = np.asarray(data["T_eef_camera"], dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError(f"T_eef_camera must be 4x4, got {matrix.shape}")
    if not np.all(np.isfinite(matrix)):
        raise ValueError("T_eef_camera contains non-finite values")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError("T_eef_camera has an invalid homogeneous last row")

    orthogonality_error, determinant = rotation_quality(matrix[:3, :3])
    if orthogonality_error > 1e-3 or abs(determinant - 1.0) > 1e-3:
        raise ValueError(
            "T_eef_camera rotation is invalid: "
            f"orthogonality_error={orthogonality_error}, determinant={determinant}"
        )
    metadata = {key: value for key, value in data.items() if key != "T_eef_camera"}
    return HandEyeCalibration(
        matrix_eef_camera=matrix,
        metadata=metadata,
        path=str(calibration_path),
        orthogonality_error=orthogonality_error,
        determinant=determinant,
    )
