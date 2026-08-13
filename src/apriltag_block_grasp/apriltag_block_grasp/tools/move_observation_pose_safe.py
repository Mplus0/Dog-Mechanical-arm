#!/usr/bin/env python3
"""Move once to the legacy hardware-validated observation joint pose."""

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional

from ament_index_python.packages import get_package_share_directory

from apriltag_block_grasp.core.roarm_serial_control import RoArmJointController


CONFIRMATION = "I_ACCEPT_OBSERVATION_POSE_MOTION"
COMMANDED_JOINT_NAMES = ("b", "s", "e", "t", "r")
OBSERVED_JOINT_NAMES = ("b", "s", "e", "t", "r")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Move RoArm-M3 once to the configured observation joint pose. "
            "Default is a serial-connected dry run."
        )
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--config", default=None)
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--confirmation", default="")
    return parser.parse_args()


def config_path(override: Optional[str]) -> Path:
    if override:
        return Path(override).expanduser().resolve()
    share = Path(get_package_share_directory("apriltag_block_grasp"))
    return share / "config" / "motion_control.json"


def load_config(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    pose = data.get("observation_joint_pose_deg")
    order = data.get("observation_move_order")
    if not isinstance(pose, dict) or not isinstance(order, list):
        raise ValueError("motion config requires observation pose and move order")
    if set(order) != set(COMMANDED_JOINT_NAMES) or len(order) != len(
        COMMANDED_JOINT_NAMES
    ):
        raise ValueError(
            "observation_move_order must contain b, s, e, t and r exactly once"
        )
    for name in COMMANDED_JOINT_NAMES:
        value = float(pose[name])
        if not math.isfinite(value):
            raise ValueError(f"observation pose {name} must be finite")
    if pose.get("g") is not None:
        raise ValueError("observation pose must not command the gripper")
    return data


def joint_degrees(state: Dict[str, Any]) -> Dict[str, float]:
    result: Dict[str, float] = {}
    for name in OBSERVED_JOINT_NAMES:
        value = float(state[name])
        if not math.isfinite(value):
            raise ValueError(f"state.{name} must be finite")
        result[name] = math.degrees(value)
    return result


def read_initial_state(
    controller: RoArmJointController, max_empty_reads: int = 8
) -> Dict[str, Any]:
    empty = 0
    while empty <= max_empty_reads:
        state = controller.read_state(timeout_s=0.5)
        if state is not None:
            joint_degrees(state)
            return state
        empty += 1
    raise RuntimeError("no initial T=1051 arm state received")


def main() -> None:
    args = parse_args()
    report: Dict[str, Any] = {
        "tool": "apriltag_block_grasp.move_observation_pose_safe",
        "scope": "legacy_T121_fixed_observation_pose",
        "dry_run": not args.enable_motion,
        "enable_motion": bool(args.enable_motion),
        "camera_opened": False,
        "cartesian_commanded": False,
        "gripper_commanded": False,
        "fill_light_commanded": False,
        "automatic_retry_enabled": False,
        "automatic_recovery_enabled": False,
        "motion_command_sent": False,
    }
    controller: Optional[RoArmJointController] = None
    exit_code = 1
    try:
        path = config_path(args.config)
        config = load_config(path)
        pose = config["observation_joint_pose_deg"]
        order = config["observation_move_order"]
        speed = float(config["observation_speed_deg_s"])
        acceleration = float(config["observation_acceleration"])
        interval_s = float(config["observation_command_interval_s"])
        completion = config.get("observation_completion", {})
        if completion.get("mode") != "timed":
            raise ValueError("this incremental tool supports only timed completion")
        timed_wait_s = float(completion["timed_wait_s"])
        if not all(
            math.isfinite(value) and value > 0.0
            for value in (speed, acceleration, timed_wait_s)
        ):
            raise ValueError("speed, acceleration and timed wait must be positive")
        if not math.isfinite(interval_s) or interval_s < 0.0:
            raise ValueError("observation command interval must be nonnegative")

        planned = [
            RoArmJointController.build_joint_command(
                name, float(pose[name]), speed, acceleration
            )
            for name in order
        ]
        report.update(
            {
                "config_path": str(path),
                "observation_pose_source": "legacy_hardware_validated_package",
                "planned_commands": planned,
                "move_order": order,
                "command_interval_s": interval_s,
                "completion_mode": "timed",
                "timed_wait_s": timed_wait_s,
                "joint_tolerance_deg": None,
            }
        )
        if args.enable_motion and args.confirmation != CONFIRMATION:
            raise ValueError(
                f"motion requires --confirmation {CONFIRMATION}"
            )

        controller = RoArmJointController(port=args.port, timeout_s=0.2)
        controller.connect(settle_time_s=2.0)
        initial_state = read_initial_state(controller)
        report["initial_joint_deg"] = joint_degrees(initial_state)
        report["initial_gripper_rad"] = float(initial_state["g"])

        if not args.enable_motion:
            report["summary"] = {
                "valid": True,
                "reason": "dry_run_checks_passed",
                "motion_command_sent": False,
                "reached_target": False,
            }
            exit_code = 0
            return

        sent_commands = []
        for index, name in enumerate(order):
            sent_commands.append(
                controller.send_joint_command(
                    name, float(pose[name]), speed, acceleration
                )
            )
            report["motion_command_sent"] = True
            if index + 1 < len(order) and interval_s > 0.0:
                time.sleep(interval_s)
        report["sent_commands"] = sent_commands

        deadline = time.monotonic() + timed_wait_s
        final_state = None
        final_errors = None
        feedback_sample_count = 0
        while time.monotonic() < deadline:
            state = controller.read_state(timeout_s=0.4)
            if state is None:
                continue
            feedback_sample_count += 1
            actual = joint_degrees(state)
            errors = {
                name: actual[name] - float(pose[name])
                for name in COMMANDED_JOINT_NAMES
            }
            final_state = state
            final_errors = errors
        wait_complete = final_state is not None
        report.update(
            {
                "feedback_sample_count": feedback_sample_count,
                "final_joint_deg": (
                    None if final_state is None else joint_degrees(final_state)
                ),
                "final_joint_error_deg": final_errors,
                "final_gripper_rad": (
                    None if final_state is None else float(final_state["g"])
                ),
                "gripper_delta_rad": (
                    None
                    if final_state is None
                    else float(final_state["g"]) - float(initial_state["g"])
                ),
                "transmitted_command_count": controller.transmitted_command_count,
                "transmitted_byte_count": controller.transmitted_byte_count,
                "summary": {
                    "valid": wait_complete,
                    "reason": (
                        "timed_wait_complete"
                        if wait_complete
                        else "no_feedback_during_timed_wait"
                    ),
                    "motion_command_sent": True,
                    "reached_target": None,
                    "accuracy_requires_user_review": True,
                },
            }
        )
        exit_code = 0 if wait_complete else 1
    except Exception as exc:
        report["summary"] = {
            "valid": False,
            "reason": "validation_or_execution_failed",
            "motion_command_sent": bool(report["motion_command_sent"]),
            "reached_target": False,
            "error": f"{type(exc).__name__}: {exc}",
        }
    finally:
        if controller is not None:
            report.setdefault(
                "transmitted_command_count", controller.transmitted_command_count
            )
            report.setdefault(
                "transmitted_byte_count", controller.transmitted_byte_count
            )
            controller.close()
        print(json.dumps(report, ensure_ascii=False, indent=2))
        sys.exit(exit_code)


if __name__ == "__main__":
    main()
