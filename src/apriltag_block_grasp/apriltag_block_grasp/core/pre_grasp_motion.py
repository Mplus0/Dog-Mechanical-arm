"""Validation and segmentation for the explicitly gated pre-grasp motion."""

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Mapping, Sequence, Tuple

from apriltag_block_grasp.core.roarm_serial_control import RoArmCartesianController


PRE_GRASP_SEGMENT_COMMAND_TYPE = "move_pre_grasp_segment"


def _finite(value: Any, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


@dataclass(frozen=True)
class PreGraspMotionConfig:
    speed: float
    maximum_segment_mm: float
    maximum_segment_count: int
    position_tolerance_mm: float
    arrival_stable_samples: int
    minimum_wait_s: float
    motion_timeout_s: float
    workspace_x_mm: Tuple[float, float]
    workspace_y_mm: Tuple[float, float]
    workspace_z_mm: Tuple[float, float]
    pitch_rad: float
    roll_rad: float

    def __post_init__(self) -> None:
        values = (
            self.speed,
            self.maximum_segment_mm,
            self.position_tolerance_mm,
            self.minimum_wait_s,
            self.motion_timeout_s,
            *self.workspace_x_mm,
            *self.workspace_y_mm,
            *self.workspace_z_mm,
            self.pitch_rad,
            self.roll_rad,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("pre-grasp motion parameters must be finite")
        if min(
            self.speed,
            self.maximum_segment_mm,
            self.position_tolerance_mm,
            self.minimum_wait_s,
            self.motion_timeout_s,
        ) <= 0.0:
            raise ValueError("pre-grasp speed, distances and times must be positive")
        if self.maximum_segment_count < 1 or self.arrival_stable_samples < 1:
            raise ValueError("pre-grasp segment and stable-sample counts must be positive")
        for name, limits in (
            ("workspace_x_mm", self.workspace_x_mm),
            ("workspace_y_mm", self.workspace_y_mm),
            ("workspace_z_mm", self.workspace_z_mm),
        ):
            if limits[0] >= limits[1]:
                raise ValueError(f"{name} minimum must be less than maximum")

    def validate_workspace(self, xyz: Sequence[float]) -> Tuple[float, float, float]:
        if len(xyz) != 3:
            raise ValueError("Cartesian XYZ must contain three values")
        point = tuple(_finite(value, key) for value, key in zip(xyz, "xyz"))
        for value, name, limits in zip(
            point,
            ("x", "y", "z"),
            (self.workspace_x_mm, self.workspace_y_mm, self.workspace_z_mm),
        ):
            if not limits[0] <= value <= limits[1]:
                raise ValueError(
                    f"target {name}={value:.3f} mm is outside configured workspace "
                    f"[{limits[0]:.3f}, {limits[1]:.3f}] mm"
                )
        return point

    def build_segments(
        self, start_xyz: Sequence[float], goal_xyz: Sequence[float]
    ) -> List[Dict[str, float]]:
        start = self.validate_workspace(start_xyz)
        goal = self.validate_workspace(goal_xyz)
        distance = math.dist(start, goal)
        planning_step_mm = self.maximum_segment_mm - self.position_tolerance_mm
        if planning_step_mm <= 0.0:
            raise ValueError(
                "position_tolerance_mm must be less than maximum_segment_mm"
            )
        count = max(1, int(math.ceil(distance / planning_step_mm)))
        if count > self.maximum_segment_count:
            raise ValueError(
                f"pre-grasp path requires {count} segments, exceeding "
                f"maximum_segment_count={self.maximum_segment_count}"
            )
        return [
            {
                "x": start[0] + (goal[0] - start[0]) * index / count,
                "y": start[1] + (goal[1] - start[1]) * index / count,
                "z": start[2] + (goal[2] - start[2]) * index / count,
            }
            for index in range(1, count + 1)
        ]

    def validate_request(
        self, request: Mapping[str, Any], current_state: Mapping[str, Any]
    ) -> Dict[str, Any]:
        if request.get("type") != PRE_GRASP_SEGMENT_COMMAND_TYPE:
            raise ValueError(f"command type must be {PRE_GRASP_SEGMENT_COMMAND_TYPE}")
        command_id = request.get("command_id")
        if isinstance(command_id, bool) or not isinstance(command_id, (int, str)):
            raise ValueError("command_id must be an integer or non-empty string")
        if isinstance(command_id, str) and not command_id.strip():
            raise ValueError("command_id must be an integer or non-empty string")
        unexpected = set(request) - {"command_id", "type", "x", "y", "z"}
        if unexpected:
            raise ValueError(
                "pre-grasp request cannot override configured motion fields: "
                + ",".join(sorted(unexpected))
            )
        current = self.validate_workspace(
            tuple(_finite(current_state[key], f"current_state.{key}") for key in "xyz")
        )
        target = self.validate_workspace(
            tuple(_finite(request.get(key), key) for key in "xyz")
        )
        distance = math.dist(current, target)
        if distance > self.maximum_segment_mm + 1e-6:
            raise ValueError(
                f"requested segment {distance:.3f} mm exceeds "
                f"maximum_segment_mm={self.maximum_segment_mm:.3f}"
            )
        gripper_rad = _finite(current_state.get("g"), "current_state.g")
        serial_command = RoArmCartesianController.build_legacy_cartesian_command(
            x_mm=target[0],
            y_mm=target[1],
            z_mm=target[2],
            pitch_rad=self.pitch_rad,
            roll_rad=self.roll_rad,
            gripper_rad=gripper_rad,
            speed=self.speed,
        )
        return {
            "command_id": command_id,
            "current_xyz_mm": _xyz_dict(current),
            "target_xyz_mm": _xyz_dict(target),
            "segment_distance_mm": distance,
            "serial_command": serial_command,
        }


def _xyz_dict(values: Sequence[float]) -> Dict[str, float]:
    return {key: float(value) for key, value in zip("xyz", values)}


def _limits(values: Any, name: str) -> Tuple[float, float]:
    if not isinstance(values, list) or len(values) != 2:
        raise ValueError(f"{name} must contain [minimum, maximum]")
    return float(values[0]), float(values[1])


def load_pre_grasp_motion_config(
    motion_config_path: str, grasp_calibration_path: str
) -> PreGraspMotionConfig:
    with Path(motion_config_path).open("r", encoding="utf-8") as stream:
        motion_data = json.load(stream)
    with Path(grasp_calibration_path).open("r", encoding="utf-8") as stream:
        grasp_data = json.load(stream)
    config = motion_data["pre_grasp_motion"]
    workspace = config["workspace_mm"]
    orientation = grasp_data["grasp_tool_orientation_rpy_rad"]
    if not isinstance(orientation, list) or len(orientation) != 3:
        raise ValueError("grasp_tool_orientation_rpy_rad must contain roll/pitch/yaw")
    return PreGraspMotionConfig(
        speed=float(config["speed"]),
        maximum_segment_mm=float(config["maximum_segment_mm"]),
        maximum_segment_count=int(config["maximum_segment_count"]),
        position_tolerance_mm=float(config["position_tolerance_mm"]),
        arrival_stable_samples=int(config["arrival_stable_samples"]),
        minimum_wait_s=float(config["minimum_wait_s"]),
        motion_timeout_s=float(config["motion_timeout_s"]),
        workspace_x_mm=_limits(workspace["x"], "workspace.x"),
        workspace_y_mm=_limits(workspace["y"], "workspace.y"),
        workspace_z_mm=_limits(workspace["z"], "workspace.z"),
        roll_rad=float(orientation[0]),
        pitch_rad=float(orientation[1]),
    )
