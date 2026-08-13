"""Pure helpers for manually calibrated grasp compensation files."""

import json
import math
from pathlib import Path
from typing import Any, Dict, Iterable, Tuple


def finite_xyz(values: Iterable[Any], name: str) -> Tuple[float, float, float]:
    materialized = tuple(values)
    if len(materialized) != 3:
        raise ValueError(f"{name} must contain exactly three values")
    try:
        parsed = tuple(float(value) for value in materialized)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain numeric values") from exc
    if not all(math.isfinite(value) for value in parsed):
        raise ValueError(f"{name} must contain finite values")
    return parsed  # type: ignore[return-value]


def corrected_offset(
    original_offset: Iterable[Any],
    automatic_xyz: Iterable[Any],
    corrected_xyz: Iterable[Any],
    manual_trim_mm: Iterable[Any] = (0.0, 0.0, 0.0),
) -> Tuple[float, float, float]:
    original = finite_xyz(original_offset, "original_offset")
    automatic = finite_xyz(automatic_xyz, "automatic_xyz")
    corrected = finite_xyz(corrected_xyz, "corrected_xyz")
    trim = finite_xyz(manual_trim_mm, "manual_trim_mm")
    return tuple(
        original[index] + corrected[index] - automatic[index] + trim[index]
        for index in range(3)
    )  # type: ignore[return-value]


def validate_gripper_angle(value: Any, name: str) -> float:
    try:
        angle = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(angle) or not 0.0 <= angle <= 180.0:
        raise ValueError(f"{name} must be within [0, 180] deg")
    return angle


def load_calibration_documents(
    grasp_path: Path, motion_path: Path
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    with grasp_path.open("r", encoding="utf-8") as stream:
        grasp = json.load(stream)
    with motion_path.open("r", encoding="utf-8") as stream:
        motion = json.load(stream)
    finite_xyz(grasp["final_grasp_tcp_offset_base_mm"], "final_grasp_tcp_offset")
    validate_gripper_angle(motion["gripper_open"]["angle_deg"], "open angle")
    validate_gripper_angle(
        motion["pick_sequence"]["close_gripper"]["angle_deg"], "close angle"
    )
    return grasp, motion


def apply_values(
    grasp: Dict[str, Any],
    motion: Dict[str, Any],
    offset_xyz: Iterable[Any],
    open_angle_deg: Any,
    close_angle_deg: Any,
) -> None:
    offset = finite_xyz(offset_xyz, "offset_xyz")
    open_angle = validate_gripper_angle(open_angle_deg, "open angle")
    close_angle = validate_gripper_angle(close_angle_deg, "close angle")
    grasp["final_grasp_tcp_offset_base_mm"] = list(offset)
    motion["gripper_open"]["angle_deg"] = open_angle
    motion["pick_sequence"]["close_gripper"]["angle_deg"] = close_angle
