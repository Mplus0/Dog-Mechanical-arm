"""Strict interpretation of RoArm-M3 T=1051 Cartesian pose fields."""

from dataclasses import dataclass
from typing import Any, Dict

import numpy as np

from apriltag_block_grasp.core.rigid_transform import make_transform, rotation_from_rpy


@dataclass(frozen=True)
class RoArmCartesianPose:
    x_mm: float
    y_mm: float
    z_mm: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    pitch_source_field: str
    matrix_base_eef: np.ndarray


def _required_finite(state: Dict[str, Any], key: str) -> float:
    if key not in state:
        raise KeyError(f"RoArm state is missing required field {key!r}")
    value = float(state[key])
    if not np.isfinite(value):
        raise ValueError(f"RoArm state field {key!r} is non-finite")
    return value


def cartesian_pose_from_state(state: Dict[str, Any]) -> RoArmCartesianPose:
    if not isinstance(state, dict) or state.get("T") != 1051:
        raise ValueError("expected a RoArm T=1051 state dictionary")
    x_mm = _required_finite(state, "x")
    y_mm = _required_finite(state, "y")
    z_mm = _required_finite(state, "z")
    roll_rad = _required_finite(state, "r")
    pitch_source = "tit" if "tit" in state else "t"
    pitch_rad = _required_finite(state, pitch_source)
    yaw_rad = _required_finite(state, "b")
    rotation = rotation_from_rpy(roll_rad, pitch_rad, yaw_rad)
    matrix = make_transform(rotation, [x_mm, y_mm, z_mm])
    return RoArmCartesianPose(
        x_mm=x_mm,
        y_mm=y_mm,
        z_mm=z_mm,
        roll_rad=roll_rad,
        pitch_rad=pitch_rad,
        yaw_rad=yaw_rad,
        pitch_source_field=pitch_source,
        matrix_base_eef=matrix,
    )
