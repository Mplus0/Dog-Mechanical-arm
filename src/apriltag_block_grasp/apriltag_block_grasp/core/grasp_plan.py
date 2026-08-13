"""Pure, motion-free planning of the fixed-orientation grasp points."""

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping, Tuple


@dataclass(frozen=True)
class PreGraspPlanConfig:
    final_grasp_tcp_offset_base_mm: Tuple[float, float, float]
    pre_grasp_z_offset_mm: float
    status: str
    source: str

    def __post_init__(self) -> None:
        values = (*self.final_grasp_tcp_offset_base_mm, self.pre_grasp_z_offset_mm)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("grasp-plan parameters must be finite")
        if self.pre_grasp_z_offset_mm <= 0.0:
            raise ValueError("pre_grasp_z_offset_mm must be positive")
        if not self.status.strip() or not self.source.strip():
            raise ValueError("grasp-plan status and source must be non-empty")

    def build(self, base_object_mm: Mapping[str, Any]) -> Dict[str, Any]:
        try:
            object_xyz = tuple(float(base_object_mm[key]) for key in ("x", "y", "z"))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("base_object_mm must contain numeric x/y/z") from exc
        if not all(math.isfinite(value) for value in object_xyz):
            raise ValueError("base_object_mm x/y/z must be finite")

        final_xyz = tuple(
            object_value + offset
            for object_value, offset in zip(
                object_xyz, self.final_grasp_tcp_offset_base_mm
            )
        )
        pre_xyz = (final_xyz[0], final_xyz[1], final_xyz[2] + self.pre_grasp_z_offset_mm)
        return {
            "planning_only": True,
            "cartesian_motion_command_sent": False,
            "plan_status": self.status,
            "plan_source": self.source,
            "base_object_mm": _xyz_dict(object_xyz),
            "final_grasp_tcp_offset_base_mm": _xyz_dict(
                self.final_grasp_tcp_offset_base_mm
            ),
            "final_grasp_tcp_mm": _xyz_dict(final_xyz),
            "pre_grasp_tcp_mm": _xyz_dict(pre_xyz),
            "pre_grasp_z_offset_mm": float(self.pre_grasp_z_offset_mm),
            "pre_grasp_and_final_same_xy": (
                pre_xyz[0] == final_xyz[0] and pre_xyz[1] == final_xyz[1]
            ),
        }


def _xyz_dict(values: Tuple[float, float, float]) -> Dict[str, float]:
    return {key: float(value) for key, value in zip(("x", "y", "z"), values)}


def load_pre_grasp_plan_config(path: str) -> PreGraspPlanConfig:
    with Path(path).open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    offset = data["final_grasp_tcp_offset_base_mm"]
    pre_grasp = data["pre_grasp"]
    if not isinstance(offset, list) or len(offset) != 3:
        raise ValueError("final_grasp_tcp_offset_base_mm must contain three values")
    return PreGraspPlanConfig(
        final_grasp_tcp_offset_base_mm=tuple(float(value) for value in offset),
        pre_grasp_z_offset_mm=float(pre_grasp["z_offset_from_final_mm"]),
        status=str(pre_grasp["status"]),
        source=str(pre_grasp["source"]),
    )
