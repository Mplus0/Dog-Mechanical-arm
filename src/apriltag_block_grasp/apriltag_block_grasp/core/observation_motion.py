"""Validation and planning for the fixed observation joint pose."""

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Tuple


OBSERVATION_COMMAND_TYPE = "move_observation_pose"
OBSERVATION_JOINTS = ("b", "s", "e", "t")
JOINT_IDS = {"b": 1, "s": 2, "e": 3, "t": 4}


@dataclass(frozen=True)
class ObservationMotionConfig:
    pose_deg: Dict[str, float]
    move_order: Tuple[str, ...]
    speed_deg_s: float
    acceleration: float
    command_interval_s: float
    timed_wait_s: float

    def __post_init__(self) -> None:
        if len(self.move_order) != len(OBSERVATION_JOINTS) or set(
            self.move_order
        ) != set(OBSERVATION_JOINTS):
            raise ValueError(
                "observation move order must contain b, s, e and t exactly once"
            )
        if set(self.pose_deg) != set(OBSERVATION_JOINTS):
            raise ValueError("observation pose must contain only b, s, e and t")
        values = (
            *self.pose_deg.values(),
            self.speed_deg_s,
            self.acceleration,
            self.command_interval_s,
            self.timed_wait_s,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("observation motion parameters must be finite")
        if self.speed_deg_s <= 0.0 or self.acceleration <= 0.0:
            raise ValueError("observation speed and acceleration must be positive")
        if self.command_interval_s < 0.0:
            raise ValueError("observation command interval must be nonnegative")
        if self.timed_wait_s <= 0.0:
            raise ValueError("observation timed wait must be positive")

    def serial_commands(self) -> Tuple[Dict[str, Any], ...]:
        return tuple(
            {
                "T": 121,
                "joint": JOINT_IDS[name],
                "angle": float(self.pose_deg[name]),
                "spd": float(self.speed_deg_s),
                "acc": float(self.acceleration),
            }
            for name in self.move_order
        )


def validate_observation_request(command: Dict[str, Any]) -> Any:
    if command.get("type") != OBSERVATION_COMMAND_TYPE:
        raise ValueError(f"command type must be {OBSERVATION_COMMAND_TYPE}")
    command_id = command.get("command_id")
    if isinstance(command_id, bool) or not isinstance(command_id, (int, str)):
        raise ValueError("command_id must be an integer or non-empty string")
    if isinstance(command_id, str) and not command_id.strip():
        raise ValueError("command_id must be an integer or non-empty string")
    unexpected = set(command) - {"command_id", "type"}
    if unexpected:
        raise ValueError(
            "observation request cannot override configured motion fields: "
            + ",".join(sorted(unexpected))
        )
    return command_id


def load_observation_motion_config(path: str) -> ObservationMotionConfig:
    with Path(path).open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    pose = data["observation_joint_pose_deg"]
    if pose.get("r") is not None or pose.get("g") is not None:
        raise ValueError("observation pose must preserve R and the gripper")
    completion = data["observation_completion"]
    if completion.get("mode") != "timed":
        raise ValueError("incremental observation motion requires timed completion")
    return ObservationMotionConfig(
        pose_deg={name: float(pose[name]) for name in OBSERVATION_JOINTS},
        move_order=tuple(str(name) for name in data["observation_move_order"]),
        speed_deg_s=float(data["observation_speed_deg_s"]),
        acceleration=float(data["observation_acceleration"]),
        command_interval_s=float(data["observation_command_interval_s"]),
        timed_wait_s=float(completion["timed_wait_s"]),
    )
