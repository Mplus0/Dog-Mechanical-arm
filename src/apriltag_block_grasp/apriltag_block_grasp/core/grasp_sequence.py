"""Validated configuration and driver commands for the remaining pick stages."""

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping, Tuple

from apriltag_block_grasp.core.roarm_serial_control import RoArmCartesianController


CARTESIAN_STAGE_COMMAND_TYPE = "move_grasp_cartesian_stage"
CLOSE_GRIPPER_COMMAND_TYPE = "close_gripper"
EXECUTION_STAGES = (
    "pre_grasp",
    "approach",
    "final_grasp",
    "close_gripper",
    "lift",
)
CARTESIAN_STAGES = ("approach", "final_grasp", "lift")


def execution_stage_index(stage: str) -> int:
    try:
        return EXECUTION_STAGES.index(str(stage))
    except ValueError as exc:
        raise ValueError(
            "execution_limit must be one of " + ", ".join(EXECUTION_STAGES)
        ) from exc


def stage_is_enabled(execution_limit: str, stage: str) -> bool:
    return execution_stage_index(execution_limit) >= execution_stage_index(stage)


def _finite(value: Any, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def _command_id(command: Mapping[str, Any]) -> Any:
    value = command.get("command_id")
    if isinstance(value, bool) or not isinstance(value, (int, str)):
        raise ValueError("command_id must be an integer or non-empty string")
    if isinstance(value, str) and not value.strip():
        raise ValueError("command_id must be an integer or non-empty string")
    return value


@dataclass(frozen=True)
class CartesianStageConfig:
    speeds: Dict[str, float]
    maximum_segment_mm: float
    position_tolerance_mm: float
    arrival_stable_samples: int
    minimum_wait_s: float
    motion_timeout_s: float
    workspace_x_mm: Tuple[float, float]
    workspace_y_mm: Tuple[float, float]
    workspace_z_mm: Tuple[float, float]

    def __post_init__(self) -> None:
        if set(self.speeds) != set(CARTESIAN_STAGES):
            raise ValueError("Cartesian speeds must contain approach/final_grasp/lift")
        values = (
            *self.speeds.values(),
            self.maximum_segment_mm,
            self.position_tolerance_mm,
            self.minimum_wait_s,
            self.motion_timeout_s,
            *self.workspace_x_mm,
            *self.workspace_y_mm,
            *self.workspace_z_mm,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("Cartesian-stage parameters must be finite")
        if min(
            *self.speeds.values(),
            self.maximum_segment_mm,
            self.position_tolerance_mm,
            self.minimum_wait_s,
            self.motion_timeout_s,
        ) <= 0.0:
            raise ValueError("Cartesian-stage speeds, distances and times must be positive")
        if self.arrival_stable_samples < 1:
            raise ValueError("arrival_stable_samples must be positive")
        for name, limits in (
            ("workspace_x_mm", self.workspace_x_mm),
            ("workspace_y_mm", self.workspace_y_mm),
            ("workspace_z_mm", self.workspace_z_mm),
        ):
            if limits[0] >= limits[1]:
                raise ValueError(f"{name} minimum must be less than maximum")

    def validate_xyz(self, values: Tuple[Any, Any, Any]) -> Tuple[float, float, float]:
        point = tuple(_finite(value, axis) for value, axis in zip(values, "xyz"))
        for value, axis, limits in zip(
            point,
            "xyz",
            (self.workspace_x_mm, self.workspace_y_mm, self.workspace_z_mm),
        ):
            if not limits[0] <= value <= limits[1]:
                raise ValueError(f"target {axis} is outside configured workspace")
        return point  # type: ignore[return-value]

    def validate_request(
        self,
        request: Mapping[str, Any],
        current_state: Mapping[str, Any],
        execution_limit: str,
    ) -> Dict[str, Any]:
        if request.get("type") != CARTESIAN_STAGE_COMMAND_TYPE:
            raise ValueError(f"command type must be {CARTESIAN_STAGE_COMMAND_TYPE}")
        command_id = _command_id(request)
        unexpected = set(request) - {"command_id", "type", "stage", "x", "y", "z"}
        if unexpected:
            raise ValueError("Cartesian-stage request contains unsupported fields")
        stage = str(request.get("stage"))
        if stage not in CARTESIAN_STAGES:
            raise ValueError("invalid Cartesian grasp stage")
        if not stage_is_enabled(execution_limit, stage):
            raise ValueError(f"stage {stage} exceeds execution_limit={execution_limit}")
        current = self.validate_xyz(tuple(current_state.get(axis) for axis in "xyz"))
        target = self.validate_xyz(tuple(request.get(axis) for axis in "xyz"))
        distance = math.dist(current, target)
        if distance > self.maximum_segment_mm + 1e-6:
            raise ValueError("Cartesian stage exceeds maximum_segment_mm")
        pitch = _finite(current_state.get("tit"), "current_state.tit")
        roll = _finite(current_state.get("r"), "current_state.r")
        gripper = _finite(current_state.get("g"), "current_state.g")
        serial_command = RoArmCartesianController.build_legacy_cartesian_command(
            x_mm=target[0],
            y_mm=target[1],
            z_mm=target[2],
            pitch_rad=pitch,
            roll_rad=roll,
            gripper_rad=gripper,
            speed=self.speeds[stage],
        )
        return {
            "command_id": command_id,
            "stage": stage,
            "current_xyz_mm": _xyz_dict(current),
            "target_xyz_mm": _xyz_dict(target),
            "segment_distance_mm": distance,
            "orientation_source": "fresh_T1051_tit_and_r",
            "serial_command": serial_command,
        }


@dataclass(frozen=True)
class CloseGripperConfig:
    angle_deg: float
    speed_deg_s: float
    acceleration: float
    timed_wait_s: float

    def __post_init__(self) -> None:
        values = (self.angle_deg, self.speed_deg_s, self.acceleration, self.timed_wait_s)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("close-gripper parameters must be finite")
        if self.speed_deg_s <= 0.0 or self.acceleration <= 0.0 or self.timed_wait_s <= 0.0:
            raise ValueError("close-gripper speed, acceleration and wait must be positive")

    def validate_request(self, command: Mapping[str, Any], execution_limit: str) -> Any:
        if command.get("type") != CLOSE_GRIPPER_COMMAND_TYPE:
            raise ValueError(f"command type must be {CLOSE_GRIPPER_COMMAND_TYPE}")
        command_id = _command_id(command)
        if set(command) - {"command_id", "type"}:
            raise ValueError("close-gripper request cannot override configured fields")
        if not stage_is_enabled(execution_limit, "close_gripper"):
            raise ValueError("close_gripper exceeds execution_limit")
        return command_id

    def serial_command(self) -> Dict[str, Any]:
        return {
            "T": 121,
            "joint": 6,
            "angle": float(self.angle_deg),
            "spd": float(self.speed_deg_s),
            "acc": float(self.acceleration),
        }


@dataclass(frozen=True)
class PickSequenceConfig:
    cartesian: CartesianStageConfig
    close_gripper: CloseGripperConfig
    lift_relative_z_mm: float
    grasp_roll_rad: float
    grasp_pitch_rad: float

    def __post_init__(self) -> None:
        if not all(
            math.isfinite(value)
            for value in (
                self.lift_relative_z_mm,
                self.grasp_roll_rad,
                self.grasp_pitch_rad,
            )
        ) or self.lift_relative_z_mm <= 0.0:
            raise ValueError("lift_relative_z_mm must be positive")


def _xyz_dict(values: Tuple[float, float, float]) -> Dict[str, float]:
    return {axis: float(value) for axis, value in zip("xyz", values)}


def _limits(values: Any, name: str) -> Tuple[float, float]:
    if not isinstance(values, list) or len(values) != 2:
        raise ValueError(f"{name} must contain [minimum, maximum]")
    return float(values[0]), float(values[1])


def load_pick_sequence_config(
    motion_config_path: str, grasp_calibration_path: str
) -> PickSequenceConfig:
    with Path(motion_config_path).open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    with Path(grasp_calibration_path).open("r", encoding="utf-8") as stream:
        calibration = json.load(stream)
    config = data["pick_sequence"]
    motion = config["cartesian_motion"]
    workspace = motion["workspace_mm"]
    close = config["close_gripper"]
    orientation = calibration["grasp_tool_orientation_rpy_rad"]
    if not isinstance(orientation, list) or len(orientation) != 3:
        raise ValueError("grasp_tool_orientation_rpy_rad must contain roll/pitch/yaw")
    return PickSequenceConfig(
        cartesian=CartesianStageConfig(
            speeds={stage: float(motion["speed"][stage]) for stage in CARTESIAN_STAGES},
            maximum_segment_mm=float(motion["maximum_segment_mm"]),
            position_tolerance_mm=float(motion["position_tolerance_mm"]),
            arrival_stable_samples=int(motion["arrival_stable_samples"]),
            minimum_wait_s=float(motion["minimum_wait_s"]),
            motion_timeout_s=float(motion["motion_timeout_s"]),
            workspace_x_mm=_limits(workspace["x"], "workspace.x"),
            workspace_y_mm=_limits(workspace["y"], "workspace.y"),
            workspace_z_mm=_limits(workspace["z"], "workspace.z"),
        ),
        close_gripper=CloseGripperConfig(
            angle_deg=float(close["angle_deg"]),
            speed_deg_s=float(close["speed_deg_s"]),
            acceleration=float(close["acceleration"]),
            timed_wait_s=float(close["timed_wait_s"]),
        ),
        lift_relative_z_mm=float(config["lift_relative_z_mm"]),
        grasp_roll_rad=float(orientation[0]),
        grasp_pitch_rad=float(orientation[1]),
    )
