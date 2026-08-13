"""Load and validate the fixed transform from the block frame to the Tag frame."""

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict

import numpy as np

from apriltag_block_grasp.core.rigid_transform import (
    make_transform,
    rotation_from_rpy,
    rotation_quality,
)


@dataclass(frozen=True)
class TagToObjectCalibration:
    matrix_tag_object: np.ndarray
    translation_mm: np.ndarray
    rotation_rpy_deg: np.ndarray
    metadata: Dict[str, Any]
    path: str
    orthogonality_error: float
    determinant: float


def _finite_vector(data: Dict[str, Any], key: str) -> np.ndarray:
    if key not in data:
        raise KeyError(f"tag-to-object file must contain {key}")
    values = np.asarray(data[key], dtype=np.float64)
    if values.shape != (3,):
        raise ValueError(f"{key} must contain exactly three values, got {values.shape}")
    if not np.all(np.isfinite(values)):
        raise ValueError(f"{key} contains non-finite values")
    return values


def load_tag_to_object_calibration(path: str) -> TagToObjectCalibration:
    calibration_path = Path(path)
    with calibration_path.open("r", encoding="utf-8") as stream:
        data = json.load(stream)

    translation_mm = _finite_vector(data, "translation_mm")
    rotation_rpy_deg = _finite_vector(data, "rotation_rpy_deg")
    rotation_rpy_rad = np.radians(rotation_rpy_deg)
    rotation = rotation_from_rpy(
        float(rotation_rpy_rad[0]),
        float(rotation_rpy_rad[1]),
        float(rotation_rpy_rad[2]),
    )
    matrix = make_transform(rotation, translation_mm)

    orthogonality_error, determinant = rotation_quality(rotation)
    if orthogonality_error > 1e-9 or not math.isclose(
        determinant, 1.0, rel_tol=0.0, abs_tol=1e-9
    ):
        raise ValueError(
            "T_tag_object rotation is invalid: "
            f"orthogonality_error={orthogonality_error}, determinant={determinant}"
        )

    excluded = {"translation_mm", "rotation_rpy_deg"}
    metadata = {key: value for key, value in data.items() if key not in excluded}
    return TagToObjectCalibration(
        matrix_tag_object=matrix,
        translation_mm=translation_mm,
        rotation_rpy_deg=rotation_rpy_deg,
        metadata=metadata,
        path=str(calibration_path),
        orthogonality_error=orthogonality_error,
        determinant=determinant,
    )
