"""Explicitly enabled, narrowly scoped RoArm-M3 serial motion control."""

import json
import math
from typing import Any, Dict

from apriltag_block_grasp.core.roarm_serial_readonly import RoArmSerialStateReader


class RoArmJointController(RoArmSerialStateReader):
    """RoArm serial connection that can send absolute T=121 joint commands."""

    JOINT_IDS = {"b": 1, "s": 2, "e": 3, "t": 4, "r": 5, "g": 6}

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.transmitted_command_count = 0
        self.transmitted_byte_count = 0

    @staticmethod
    def build_joint_command(
        joint: Any, target_deg: float, speed_deg_s: float, acceleration: float
    ) -> Dict[str, Any]:
        if isinstance(joint, str):
            joint_name = joint.strip().lower()
            if joint_name not in RoArmJointController.JOINT_IDS:
                raise ValueError(f"unknown RoArm joint name: {joint!r}")
            joint_id = RoArmJointController.JOINT_IDS[joint_name]
        else:
            joint_id = int(joint)
            if joint_id not in RoArmJointController.JOINT_IDS.values():
                raise ValueError(f"unknown RoArm joint id: {joint_id}")
        values = (target_deg, speed_deg_s, acceleration)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("joint command values must be finite")
        if float(speed_deg_s) <= 0.0 or float(acceleration) <= 0.0:
            raise ValueError("speed and acceleration must be positive")
        return {
            "T": 121,
            "joint": joint_id,
            "angle": float(target_deg),
            "spd": float(speed_deg_s),
            "acc": float(acceleration),
        }

    @staticmethod
    def build_b_joint_command(
        target_b_deg: float, speed_deg_s: float, acceleration: float
    ) -> Dict[str, Any]:
        return RoArmJointController.build_joint_command(
            "b", target_b_deg, speed_deg_s, acceleration
        )

    def send_joint_command(
        self,
        joint: Any,
        target_deg: float,
        speed_deg_s: float,
        acceleration: float,
    ) -> Dict[str, Any]:
        if self.serial_port is None:
            raise RuntimeError("RoArm joint controller is not connected")
        command = self.build_joint_command(
            joint, target_deg, speed_deg_s, acceleration
        )
        self._send_encoded_command(command)
        return command

    def send_b_joint_command(
        self, target_b_deg: float, speed_deg_s: float, acceleration: float
    ) -> Dict[str, Any]:
        return self.send_joint_command(
            "b", target_b_deg, speed_deg_s, acceleration
        )

    def _send_encoded_command(self, command: Dict[str, Any]) -> None:
        if self.serial_port is None:
            raise RuntimeError("RoArm joint controller is not connected")
        encoded = (json.dumps(command, separators=(",", ":")) + "\n").encode("utf-8")
        written = int(self.serial_port.write(encoded))
        self.transmitted_byte_count += max(0, written)
        self.serial_port.flush()
        if written != len(encoded):
            raise IOError(
                f"partial serial write: expected {len(encoded)} bytes, wrote {written}"
            )
        self.transmitted_command_count += 1


class RoArmBJointController(RoArmJointController):
    """Backward-compatible controller used by the B-only diagnostic tools."""


class RoArmCartesianController(RoArmJointController):
    """RoArm serial connection that can send one M3 T=1041 pose command.

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
    ) -> Dict[str, Any]:
        values = (
            x_mm,
            y_mm,
            z_mm,
            pitch_rad,
            roll_rad,
            gripper_rad,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("Cartesian command values must be finite")
        return {
            "T": 1041,
            "x": float(x_mm),
            "y": float(y_mm),
            "z": float(z_mm),
            "t": float(pitch_rad),
            "r": float(roll_rad),
            "g": float(gripper_rad),
        }

    def send_cartesian_command(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        pitch_rad: float,
        roll_rad: float,
        gripper_rad: float,
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
        )
        self._send_encoded_command(command)
        return command
