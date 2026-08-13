#!/usr/bin/env python3
"""Dry-run-first, bounded absolute B-joint motion utility."""

import argparse
import json
import math
import time
from typing import Any, Dict, Optional

from apriltag_block_grasp.core.roarm_serial_control import RoArmBJointController


def finite_float(value: Any, name: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def b_degrees_from_state(state: Dict[str, Any]) -> float:
    if not isinstance(state, dict) or state.get("T") != 1051:
        raise ValueError("expected a RoArm T=1051 state")
    if "b" not in state:
        raise KeyError("RoArm state is missing B-joint field 'b'")
    return math.degrees(finite_float(state["b"], "state.b"))


def validate_motion_request(
    current_b_deg: float,
    target_b_deg: float,
    minimum_b_deg: float,
    maximum_b_deg: float,
    maximum_delta_deg: float,
) -> float:
    values = {
        "current_b_deg": current_b_deg,
        "target_b_deg": target_b_deg,
        "minimum_b_deg": minimum_b_deg,
        "maximum_b_deg": maximum_b_deg,
        "maximum_delta_deg": maximum_delta_deg,
    }
    values = {name: finite_float(value, name) for name, value in values.items()}
    if values["minimum_b_deg"] >= values["maximum_b_deg"]:
        raise ValueError("minimum_b_deg must be less than maximum_b_deg")
    if values["maximum_delta_deg"] <= 0.0:
        raise ValueError("maximum_delta_deg must be positive")
    if not values["minimum_b_deg"] <= values["target_b_deg"] <= values["maximum_b_deg"]:
        raise ValueError(
            f"target B={values['target_b_deg']:.3f} deg is outside the enabled "
            f"range [{values['minimum_b_deg']:.3f}, {values['maximum_b_deg']:.3f}] deg"
        )
    delta_deg = values["target_b_deg"] - values["current_b_deg"]
    if abs(delta_deg) > values["maximum_delta_deg"]:
        raise ValueError(
            f"requested B delta {delta_deg:.3f} deg exceeds "
            f"maximum_delta_deg={values['maximum_delta_deg']:.3f}"
        )
    return delta_deg


def compact_state(state: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    if state is None:
        return None
    keys = ("x", "y", "z", "tit", "b", "s", "e", "t", "r", "g", "v")
    return {key: state[key] for key in keys if key in state}


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Move only RoArm-M3 B joint with bounded, dry-run-first safety checks."
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--target-b-deg", type=float, required=True)
    parser.add_argument("--speed-deg-s", type=float, default=10.0)
    parser.add_argument("--acceleration", type=float, default=10.0)
    parser.add_argument("--min-b-deg", type=float, default=-20.0)
    parser.add_argument("--max-b-deg", type=float, default=20.0)
    parser.add_argument("--max-delta-deg", type=float, default=10.0)
    parser.add_argument("--tolerance-deg", type=float, default=1.0)
    parser.add_argument("--required-stable-samples", type=int, default=3)
    parser.add_argument("--motion-timeout-s", type=float, default=8.0)
    parser.add_argument("--initial-state-attempts", type=int, default=5)
    parser.add_argument("--initial-state-timeout-s", type=float, default=1.0)
    parser.add_argument(
        "--enable-motion",
        action="store_true",
        help="Actually send one B-joint command. Without this flag, perform dry-run only.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    controller = RoArmBJointController(port=args.port, timeout_s=0.2)
    report: Dict[str, Any] = {
        "tool": "apriltag_block_grasp.move_b_joint_safe",
        "scope": "B_joint_only",
        "dry_run": not args.enable_motion,
        "enable_motion": bool(args.enable_motion),
        "camera_opened": False,
        "other_joints_commanded": False,
        "gripper_commanded": False,
        "fill_light_commanded": False,
        "planned_command": None,
        "summary": {"valid": False},
    }
    try:
        target_b_deg = finite_float(args.target_b_deg, "target_b_deg")
        speed_deg_s = finite_float(args.speed_deg_s, "speed_deg_s")
        acceleration = finite_float(args.acceleration, "acceleration")
        tolerance_deg = finite_float(args.tolerance_deg, "tolerance_deg")
        motion_timeout_s = finite_float(args.motion_timeout_s, "motion_timeout_s")
        initial_state_timeout_s = finite_float(
            args.initial_state_timeout_s, "initial_state_timeout_s"
        )
        initial_state_attempts = max(1, int(args.initial_state_attempts))
        required_stable_samples = max(1, int(args.required_stable_samples))
        if (
            tolerance_deg <= 0.0
            or motion_timeout_s <= 0.0
            or initial_state_timeout_s <= 0.0
        ):
            raise ValueError(
                "tolerance_deg, motion_timeout_s and initial_state_timeout_s must be positive"
            )

        controller.connect()
        initial_state = None
        initial_empty_reads = 0
        for _ in range(initial_state_attempts):
            initial_state = controller.read_state(timeout_s=initial_state_timeout_s)
            if initial_state is not None:
                break
            initial_empty_reads += 1
        if initial_state is None:
            raise RuntimeError(
                "no initial T=1051 arm state received after "
                f"{initial_state_attempts} attempts"
            )
        current_b_deg = b_degrees_from_state(initial_state)
        delta_deg = validate_motion_request(
            current_b_deg=current_b_deg,
            target_b_deg=target_b_deg,
            minimum_b_deg=args.min_b_deg,
            maximum_b_deg=args.max_b_deg,
            maximum_delta_deg=args.max_delta_deg,
        )
        command = controller.build_b_joint_command(
            target_b_deg, speed_deg_s, acceleration
        )
        report.update(
            {
                "initial_state": compact_state(initial_state),
                "initial_state_attempt_count": initial_empty_reads + 1,
                "initial_empty_read_count": initial_empty_reads,
                "initial_b_deg": current_b_deg,
                "target_b_deg": target_b_deg,
                "requested_delta_deg": delta_deg,
                "enabled_absolute_range_deg": [args.min_b_deg, args.max_b_deg],
                "maximum_delta_deg": args.max_delta_deg,
                "planned_command": command,
            }
        )

        if not args.enable_motion:
            report["summary"] = {
                "valid": True,
                "reason": "dry_run_checks_passed",
                "motion_command_sent": False,
                "reached_target": False,
            }
            print(json.dumps(report, ensure_ascii=False, indent=2))
            return 0

        sent_command = controller.send_b_joint_command(
            target_b_deg, speed_deg_s, acceleration
        )
        deadline = time.monotonic() + motion_timeout_s
        stable_samples = 0
        feedback_sample_count = 0
        final_state = None
        final_b_deg = None
        while time.monotonic() < deadline:
            state = controller.read_state(timeout_s=0.5)
            if state is None:
                continue
            feedback_sample_count += 1
            final_state = state
            final_b_deg = b_degrees_from_state(state)
            if abs(final_b_deg - target_b_deg) <= tolerance_deg:
                stable_samples += 1
                if stable_samples >= required_stable_samples:
                    break
            else:
                stable_samples = 0

        reached_target = stable_samples >= required_stable_samples
        report.update(
            {
                "sent_command": sent_command,
                "transmitted_command_count": controller.transmitted_command_count,
                "transmitted_byte_count": controller.transmitted_byte_count,
                "feedback_sample_count": feedback_sample_count,
                "final_state": compact_state(final_state),
                "final_b_deg": final_b_deg,
                "final_error_deg": (
                    None if final_b_deg is None else final_b_deg - target_b_deg
                ),
                "stable_samples_at_target": stable_samples,
            }
        )
        report["summary"] = {
            "valid": reached_target,
            "reason": "target_reached" if reached_target else "motion_timeout",
            "motion_command_sent": True,
            "reached_target": reached_target,
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if reached_target else 1
    except Exception as exc:
        report["summary"] = {
            "valid": False,
            "reason": "validation_or_execution_failed",
            "motion_command_sent": controller.transmitted_byte_count > 0,
            "reached_target": False,
            "error": f"{type(exc).__name__}: {exc}",
        }
        report["transmitted_command_count"] = controller.transmitted_command_count
        report["transmitted_byte_count"] = controller.transmitted_byte_count
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 1
    finally:
        controller.close()


if __name__ == "__main__":
    raise SystemExit(main())
