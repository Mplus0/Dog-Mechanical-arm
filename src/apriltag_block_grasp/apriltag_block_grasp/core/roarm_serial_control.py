"""Explicitly enabled, narrowly scoped RoArm-M3 serial motion control."""

import json
import math
from typing import Any, Dict

from apriltag_block_grasp.core.roarm_serial_readonly import RoArmSerialStateReader


class RoArmBJointController(RoArmSerialStateReader):
    """RoArm serial connection that can send only an absolute B-joint command."""

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.transmitted_command_count = 0
        self.transmitted_byte_count = 0

    @staticmethod
    def build_b_joint_command(
        target_b_deg: float, speed_deg_s: float, acceleration: float
    ) -> Dict[str, Any]:
        values = (target_b_deg, speed_deg_s, acceleration)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("B-joint command values must be finite")
        if float(speed_deg_s) <= 0.0 or float(acceleration) <= 0.0:
            raise ValueError("speed and acceleration must be positive")
        return {
            "T": 121,
            "joint": 1,
            "angle": float(target_b_deg),
            "spd": float(speed_deg_s),
            "acc": float(acceleration),
        }

    def send_b_joint_command(
        self, target_b_deg: float, speed_deg_s: float, acceleration: float
    ) -> Dict[str, Any]:
        if self.serial_port is None:
            raise RuntimeError("RoArm B-joint controller is not connected")
        command = self.build_b_joint_command(
            target_b_deg, speed_deg_s, acceleration
        )
        encoded = (json.dumps(command, separators=(",", ":")) + "\n").encode("utf-8")
        written = int(self.serial_port.write(encoded))
        self.transmitted_byte_count += max(0, written)
        self.serial_port.flush()
        if written != len(encoded):
            raise IOError(
                f"partial serial write: expected {len(encoded)} bytes, wrote {written}"
            )
        self.transmitted_command_count += 1
        return command


class RoArmCartesianController(RoArmBJointController):
    """RoArm serial connection that can send one T=104 Cartesian command.

    The firmware command exposes Cartesian position, tool pitch (``t``), tool
    roll (``r``), and gripper angle (``g``).  It does not expose an independent
    Cartesian yaw/B field; B is selected by the firmware inverse kinematics.
    """

    @staticmethod
    def build_cartesian_command(
        x_mm: float,
        y_mm: float,
        z_mm: float,
        pitch_rad: float,
        roll_rad: float,
        gripper_rad: float,
        speed: float,
    ) -> Dict[str, Any]:
        values = (
            x_mm,
            y_mm,
            z_mm,
            pitch_rad,
            roll_rad,
            gripper_rad,
            speed,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("Cartesian command values must be finite")
        if float(speed) <= 0.0:
            raise ValueError("Cartesian speed must be positive")
        return {
            "T": 104,
            "x": float(x_mm),
            "y": float(y_mm),
            "z": float(z_mm),
            "t": float(pitch_rad),
            "r": float(roll_rad),
            "g": float(gripper_rad),
            "spd": float(speed),
        }

    def send_cartesian_command(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        pitch_rad: float,
        roll_rad: float,
        gripper_rad: float,
        speed: float,
    ) -> Dict[str, Any]:
        if self.serial_port is None:
            raise RuntimeError("RoArm Cartesian controller is not connected")
        command = self.build_cartesian_command(
            x_mm=x_mm,
            y_mm=y_mm,
            z_mm=z_mm,
            pitch_rad=pitch_rad,
            roll_rad=roll_rad,
            gripper_rad=gripper_rad,
            speed=speed,
        )
        encoded = (json.dumps(command, separators=(",", ":")) + "\n").encode(
            "utf-8"
        )
        written = int(self.serial_port.write(encoded))
        self.transmitted_byte_count += max(0, written)
        self.serial_port.flush()
        if written != len(encoded):
            raise IOError(
                f"partial serial write: expected {len(encoded)} bytes, wrote {written}"
            )
        self.transmitted_command_count += 1
        return command
