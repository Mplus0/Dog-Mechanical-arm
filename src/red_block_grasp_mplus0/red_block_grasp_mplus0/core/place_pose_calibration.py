"""Validation and persistence helpers for the fixed competition place pose."""

import math
import shutil
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Mapping, Tuple

import yaml


NODE_NAME = "visual_servo_task_node"
PARAMETERS_KEY = "ros__parameters"
POSE_KEYS = ("place_x_mm", "place_y_mm", "place_z_mm")
DEFAULT_LIMITS = {
    "place_x_mm": (80.0, 700.0),
    "place_y_mm": (-450.0, 450.0),
    "place_z_mm": (-30.0, 380.0),
}


def validate_pose(
    values: Mapping[str, Any],
    limits: Mapping[str, Tuple[float, float]] = DEFAULT_LIMITS,
) -> Dict[str, float]:
    """Return a finite, in-workspace place pose or raise ``ValueError``."""
    pose: Dict[str, float] = {}
    for key in POSE_KEYS:
        try:
            value = float(values[key])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"missing or invalid {key}") from exc
        if not math.isfinite(value):
            raise ValueError(f"{key} must be finite")
        lower, upper = limits[key]
        if value < lower or value > upper:
            raise ValueError(
                f"{key}={value:.3f} is outside [{lower:.3f}, {upper:.3f}] mm"
            )
        pose[key] = value
    return pose


def load_document(path: Path) -> Dict[str, Any]:
    """Load the ROS parameter YAML and verify its fixed place pose."""
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ValueError(f"place pose config does not exist: {path}") from exc
    except yaml.YAMLError as exc:
        raise ValueError(f"invalid YAML in {path}: {exc}") from exc
    if not isinstance(document, dict):
        raise ValueError(f"place pose config must contain a YAML mapping: {path}")
    try:
        parameters = document[NODE_NAME][PARAMETERS_KEY]
    except (KeyError, TypeError) as exc:
        raise ValueError(
            f"expected {NODE_NAME}.{PARAMETERS_KEY} in {path}"
        ) from exc
    if not isinstance(parameters, dict):
        raise ValueError(f"{NODE_NAME}.{PARAMETERS_KEY} must be a mapping")
    validate_pose(parameters)
    return document


def pose_from_document(document: Mapping[str, Any]) -> Dict[str, float]:
    parameters = document[NODE_NAME][PARAMETERS_KEY]
    return validate_pose(parameters)


def apply_pose(document: Dict[str, Any], pose: Mapping[str, Any]) -> Dict[str, float]:
    """Update only the three place coordinates in an already loaded document."""
    validated = validate_pose(pose)
    parameters = document[NODE_NAME][PARAMETERS_KEY]
    parameters.update(validated)
    return validated


def save_document(path: Path, document: Dict[str, Any]) -> Path:
    """Validate, back up, and atomically replace a place-pose YAML file."""
    pose_from_document(document)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = path.with_name(f"{path.name}.bak.{stamp}")
    shutil.copy2(path, backup)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        yaml.safe_dump(document, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    temporary.replace(path)
    return backup
