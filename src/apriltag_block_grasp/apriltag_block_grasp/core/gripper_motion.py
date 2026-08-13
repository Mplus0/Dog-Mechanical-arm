"""Validation and planning for the incrementally enabled gripper-open action."""

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict


OPEN_GRIPPER_COMMAND_TYPE = "open_gripper"


@dataclass(frozen=True)
class GripperOpenConfig:
    angle_deg: float
    speed_deg_s: float
    acceleration: float
    timed_wait_s: float

    def __post_init__(self) -> None:
        values = (
            self.angle_deg,
            self.speed_deg_s,
            self.acceleration,
            self.timed_wait_s,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("gripper-open parameters must be finite")
        if self.speed_deg_s <= 0.0 or self.acceleration <= 0.0:
            raise ValueError("gripper-open speed and acceleration must be positive")
        if self.timed_wait_s <= 0.0:
            raise ValueError("gripper-open timed wait must be positive")

    def serial_command(self) -> Dict[str, Any]:
        return {
            "T": 121,
            "joint": 6,
            "angle": float(self.angle_deg),
            "spd": float(self.speed_deg_s),
            "acc": float(self.acceleration),
        }


def validate_open_gripper_request(command: Dict[str, Any]) -> Any:
    if command.get("type") != OPEN_GRIPPER_COMMAND_TYPE:
        raise ValueError(f"command type must be {OPEN_GRIPPER_COMMAND_TYPE}")
    command_id = command.get("command_id")
    if isinstance(command_id, bool) or not isinstance(command_id, (int, str)):
        raise ValueError("command_id must be an integer or non-empty string")
    if isinstance(command_id, str) and not command_id.strip():
        raise ValueError("command_id must be an integer or non-empty string")
    unexpected = set(command) - {"command_id", "type"}
    if unexpected:
        raise ValueError(
            "open-gripper request cannot override configured motion fields: "
            + ",".join(sorted(unexpected))
        )
    return command_id


def load_gripper_open_config(path: str) -> GripperOpenConfig:
    with Path(path).open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    config = data["gripper_open"]
    return GripperOpenConfig(
        angle_deg=float(config["angle_deg"]),
        speed_deg_s=float(config["speed_deg_s"]),
        acceleration=float(config["acceleration"]),
        timed_wait_s=float(config["timed_wait_s"]),
    )
