#!/usr/bin/env python3
"""Trace one bounded B-joint command without retrying or commanding recovery."""

import argparse
import json
import math
import time
from typing import Any, Dict, List, Optional

from apriltag_block_grasp.core.roarm_serial_control import RoArmBJointController
from apriltag_block_grasp.tools.move_b_joint_safe import (
    b_degrees_from_state,
    compact_state,
    finite_float,
    validate_motion_request,
)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            "Send at most one bounded RoArm-M3 B-joint command and trace its "
            "feedback in the same serial connection."
        )
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--target-b-deg", type=float, required=True)
    parser.add_argument("--speed-deg-s", type=float, default=10.0)
    parser.add_argument("--acceleration", type=float, default=10.0)
    parser.add_argument("--min-b-deg", type=float, default=-20.0)
    parser.add_argument("--max-b-deg", type=float, default=20.0)
    parser.add_argument("--max-delta-deg", type=float, default=10.0)
    parser.add_argument("--trace-duration-s", type=float, default=15.0)
    parser.add_argument("--trace-interval-s", type=float, default=0.25)
    parser.add_argument("--initial-state-attempts", type=int, default=5)
    parser.add_argument("--initial-state-timeout-s", type=float, default=1.0)
    parser.add_argument(
        "--enable-motion",
        action="store_true",
        help="Send exactly one B-joint command. Without this flag, only validate.",
    )
    return parser.parse_args()


def trace_sample(
    elapsed_s: float, state: Dict[str, Any], b_deg: float
) -> Dict[str, Any]:
    sample: Dict[str, Any] = {
        "elapsed_s": round(float(elapsed_s), 4),
        "b_deg": float(b_deg),
    }
    if "tB" in state:
        sample["tB"] = state["tB"]
    return sample


def joint_degrees(state: Optional[Dict[str, Any]]) -> Optional[Dict[str, float]]:
    if state is None:
        return None
    result = {}
    for key in ("b", "s", "e", "t", "r", "g"):
        if key in state:
            value = finite_float(state[key], f"state.{key}")
            result[key] = math.degrees(value)
    return result


def joint_delta_degrees(
    initial: Optional[Dict[str, float]], final: Optional[Dict[str, float]]
) -> Optional[Dict[str, float]]:
    if initial is None or final is None:
        return None
    return {
        key: final[key] - initial[key]
        for key in initial.keys() & final.keys()
    }


def main() -> int:
    args = parse_arguments()
    controller = RoArmBJointController(port=args.port, timeout_s=0.2)
    report: Dict[str, Any] = {
        "tool": "apriltag_block_grasp.trace_b_joint_motion",
        "scope": "B_joint_only_single_command_trace",
        "dry_run": not args.enable_motion,
        "enable_motion": bool(args.enable_motion),
        "camera_opened": False,
        "other_joints_commanded": False,
        "gripper_commanded": False,
        "fill_light_commanded": False,
        "automatic_retry_enabled": False,
        "automatic_recovery_enabled": False,
        "planned_command": None,
        "trace": [],
        "summary": {"valid": False},
    }
    try:
        target_b_deg = finite_float(args.target_b_deg, "target_b_deg")
        speed_deg_s = finite_float(args.speed_deg_s, "speed_deg_s")
        acceleration = finite_float(args.acceleration, "acceleration")
        trace_duration_s = finite_float(args.trace_duration_s, "trace_duration_s")
        trace_interval_s = finite_float(args.trace_interval_s, "trace_interval_s")
        initial_state_timeout_s = finite_float(
            args.initial_state_timeout_s, "initial_state_timeout_s"
        )
        if trace_duration_s <= 0.0 or trace_interval_s <= 0.0:
            raise ValueError("trace_duration_s and trace_interval_s must be positive")
        if initial_state_timeout_s <= 0.0:
            raise ValueError("initial_state_timeout_s must be positive")

        controller.connect()
        initial_state = None
        initial_empty_reads = 0
        initial_state_attempts = max(1, int(args.initial_state_attempts))
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

        initial_b_deg = b_degrees_from_state(initial_state)
        requested_delta_deg = validate_motion_request(
            current_b_deg=initial_b_deg,
            target_b_deg=target_b_deg,
            minimum_b_deg=args.min_b_deg,
            maximum_b_deg=args.max_b_deg,
            maximum_delta_deg=args.max_delta_deg,
        )
        planned_command = controller.build_b_joint_command(
            target_b_deg, speed_deg_s, acceleration
        )
        initial_joint_deg = joint_degrees(initial_state)
        report.update(
            {
                "planned_command": planned_command,
                "initial_state": compact_state(initial_state),
                "initial_joint_deg": initial_joint_deg,
                "initial_state_attempt_count": initial_empty_reads + 1,
                "initial_empty_read_count": initial_empty_reads,
                "initial_b_deg": initial_b_deg,
                "target_b_deg": target_b_deg,
                "requested_delta_deg": requested_delta_deg,
                "enabled_absolute_range_deg": [args.min_b_deg, args.max_b_deg],
                "maximum_delta_deg": args.max_delta_deg,
                "trace_duration_s": trace_duration_s,
                "trace_interval_s": trace_interval_s,
            }
        )

        if not args.enable_motion:
            report["summary"] = {
                "valid": True,
                "reason": "dry_run_checks_passed",
                "motion_command_sent": False,
                "trace_recorded": False,
            }
            print(json.dumps(report, ensure_ascii=False, indent=2))
            return 0

        sent_command = controller.send_b_joint_command(
            target_b_deg, speed_deg_s, acceleration
        )
        start = time.monotonic()
        deadline = start + trace_duration_s
        next_trace_time = 0.0
        feedback_sample_count = 0
        feedback_empty_read_count = 0
        b_values: List[float] = []
        closest_b_deg = None
        closest_error_deg = None
        closest_elapsed_s = None
        final_state = None

        while time.monotonic() < deadline:
            state = controller.read_state(timeout_s=min(0.25, trace_interval_s))
            now = time.monotonic()
            elapsed_s = now - start
            if state is None:
                feedback_empty_read_count += 1
                continue
            feedback_sample_count += 1
            final_state = state
            b_deg = b_degrees_from_state(state)
            b_values.append(b_deg)
            error_deg = b_deg - target_b_deg
            if closest_error_deg is None or abs(error_deg) < abs(closest_error_deg):
                closest_b_deg = b_deg
                closest_error_deg = error_deg
                closest_elapsed_s = elapsed_s
            if elapsed_s >= next_trace_time:
                report["trace"].append(trace_sample(elapsed_s, state, b_deg))
                next_trace_time = elapsed_s + trace_interval_s

        if final_state is not None:
            final_b_deg = b_degrees_from_state(final_state)
            if not report["trace"] or (
                trace_duration_s - report["trace"][-1]["elapsed_s"]
                > trace_interval_s * 0.25
            ):
                report["trace"].append(
                    trace_sample(time.monotonic() - start, final_state, final_b_deg)
                )
        else:
            final_b_deg = None

        final_joint_deg = joint_degrees(final_state)
        report.update(
            {
                "sent_command": sent_command,
                "transmitted_command_count": controller.transmitted_command_count,
                "transmitted_byte_count": controller.transmitted_byte_count,
                "feedback_sample_count": feedback_sample_count,
                "feedback_empty_read_count": feedback_empty_read_count,
                "final_state": compact_state(final_state),
                "final_joint_deg": final_joint_deg,
                "joint_delta_deg": joint_delta_degrees(
                    initial_joint_deg, final_joint_deg
                ),
                "final_b_deg": final_b_deg,
                "final_error_deg": (
                    None if final_b_deg is None else final_b_deg - target_b_deg
                ),
                "minimum_observed_b_deg": min(b_values) if b_values else None,
                "maximum_observed_b_deg": max(b_values) if b_values else None,
                "closest_b_deg": closest_b_deg,
                "closest_error_deg": closest_error_deg,
                "closest_elapsed_s": closest_elapsed_s,
            }
        )
        valid = feedback_sample_count > 0
        report["summary"] = {
            "valid": valid,
            "reason": "trace_complete" if valid else "no_feedback_during_trace",
            "motion_command_sent": True,
            "trace_recorded": valid,
            "target_reached": (
                closest_error_deg is not None and abs(closest_error_deg) <= 1.0
            ),
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if valid else 1
    except Exception as exc:
        report["summary"] = {
            "valid": False,
            "reason": "validation_or_execution_failed",
            "motion_command_sent": controller.transmitted_byte_count > 0,
            "trace_recorded": False,
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
