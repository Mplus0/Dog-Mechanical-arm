"""Rigid-transform helpers shared by localization stages."""

import math
from typing import Iterable

import numpy as np


def rotation_x(angle_rad: float) -> np.ndarray:
    cosine = math.cos(float(angle_rad))
    sine = math.sin(float(angle_rad))
    return np.array(
        [[1.0, 0.0, 0.0], [0.0, cosine, -sine], [0.0, sine, cosine]],
        dtype=np.float64,
    )


def rotation_y(angle_rad: float) -> np.ndarray:
    cosine = math.cos(float(angle_rad))
    sine = math.sin(float(angle_rad))
    return np.array(
        [[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]],
        dtype=np.float64,
    )


def rotation_z(angle_rad: float) -> np.ndarray:
    cosine = math.cos(float(angle_rad))
    sine = math.sin(float(angle_rad))
    return np.array(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def rotation_from_rpy(roll_rad: float, pitch_rad: float, yaw_rad: float) -> np.ndarray:
    """Return Rz(yaw) @ Ry(pitch) @ Rx(roll), matching the RoArm workflow."""

    return (
        rotation_z(yaw_rad)
        @ rotation_y(pitch_rad)
        @ rotation_x(roll_rad)
    )


def make_transform(rotation: np.ndarray, translation: Iterable[float]) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    matrix[:3, 3] = np.asarray(tuple(translation), dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(matrix)):
        raise ValueError("transform contains non-finite values")
    return matrix


def rotation_quality(rotation: np.ndarray):
    matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    orthogonality_error = float(
        np.linalg.norm(matrix.T @ matrix - np.eye(3), ord="fro")
    )
    determinant = float(np.linalg.det(matrix))
    return orthogonality_error, determinant
