"""Validation and encoding for the narrowly scoped B-joint ROS command."""

import math
from dataclasses import dataclass
from typing import Any, Dict

from apriltag_block_grasp.core.roarm_serial_control import RoArmJointController


@dataclass(frozen=True)
class BJointCommandLimits:
    minimum_deg: float
    maximum_deg: float
    maximum_delta_deg: float
    maximum_speed_deg_s: float
    maximum_acceleration: float

    def __post_init__(self) -> None:
        values = (
            self.minimum_deg,
            self.maximum_deg,
            self.maximum_delta_deg,
            self.maximum_speed_deg_s,
            self.maximum_acceleration,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("B-joint limits must be finite")
        if self.minimum_deg >= self.maximum_deg:
            raise ValueError("minimum_deg must be less than maximum_deg")
        if min(
            self.maximum_delta_deg,
            self.maximum_speed_deg_s,
            self.maximum_acceleration,
        ) <= 0.0:
            raise ValueError("B-joint delta, speed and acceleration limits must be positive")


def _finite_float(value: Any, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def validate_b_joint_request(
    request: Dict[str, Any],
    current_b_deg: float,
    limits: BJointCommandLimits,
) -> Dict[str, Any]:
    """Validate one absolute B target and return the exact T=121 command."""
    if request.get("type") != "move_b_joint":
        raise ValueError("command type must be move_b_joint")
    try:
        joint = int(request.get("joint"))
    except (TypeError, ValueError) as exc:
        raise ValueError("joint must be integer 1") from exc
    if joint != 1:
        raise ValueError("only B joint 1 is allowed")

    current = _finite_float(current_b_deg, "current_b_deg")
    target = _finite_float(request.get("angle"), "angle")
    speed = _finite_float(request.get("speed"), "speed")
    acceleration = _finite_float(request.get("acceleration"), "acceleration")
    if not limits.minimum_deg <= target <= limits.maximum_deg:
        raise ValueError(
            f"target B={target:.3f} deg is outside enabled range "
            f"[{limits.minimum_deg:.3f}, {limits.maximum_deg:.3f}] deg"
        )
    delta = target - current
    if abs(delta) > limits.maximum_delta_deg:
        raise ValueError(
            f"requested B delta {delta:.3f} deg exceeds "
            f"maximum_delta_deg={limits.maximum_delta_deg:.3f}"
        )
    if speed <= 0.0 or speed > limits.maximum_speed_deg_s:
        raise ValueError(
            f"speed must be in (0, {limits.maximum_speed_deg_s:.3f}] deg/s"
        )
    if acceleration <= 0.0 or acceleration > limits.maximum_acceleration:
        raise ValueError(
            f"acceleration must be in (0, {limits.maximum_acceleration:.3f}]"
        )

    command = RoArmJointController.build_b_joint_command(
        target, speed, acceleration
    )
    return {
        "current_b_deg": current,
        "target_b_deg": target,
        "requested_delta_deg": delta,
        "speed_deg_s": speed,
        "acceleration": acceleration,
        "serial_command": command,
    }
