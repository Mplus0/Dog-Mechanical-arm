#!/usr/bin/env python3
"""Dry-run-first RoArm T=104 relative XYZ motion utility."""

import argparse
import json
import math
import time
from typing import Any, Dict, Optional

from ament_index_python.packages import get_package_share_directory

from apriltag_block_grasp.core.grasp_calibration import load_grasp_calibration
from apriltag_block_grasp.core.roarm_serial_control import RoArmCartesianController


def finite_float(value: Any, name: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def state_number(state: Dict[str, Any], key: str) -> float:
    if not isinstance(state, dict) or state.get("T") != 1051:
        raise ValueError("expected a RoArm T=1051 state")
    if key not in state:
        raise KeyError(f"RoArm state is missing field {key!r}")
    return finite_float(state[key], f"state.{key}")


def compact_state(state: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    if state is None:
        return None
    keys = ("x", "y", "z", "tit", "b", "s", "e", "t", "r", "g", "v")
    return {key: state[key] for key in keys if key in state}


def parse_arguments():
    default_calibration = (
        get_package_share_directory("apriltag_block_grasp")
        + "/config/grasp_calibration.json"
    )
    parser = argparse.ArgumentParser(
        description=(
            "Send at most one low-speed RoArm T=104 command. Target XYZ is a "
            "small offset from the current TCP. By default, pitch/roll/gripper "
            "are copied from the current T=1051 feedback. Cartesian yaw/B "
            "cannot be commanded by T=104."
        )
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--calibration-path", default=default_calibration)
    parser.add_argument(
        "--orientation-source",
        choices=("current", "calibration"),
        default="current",
        help=(
            "Use current T=1051 tit/r feedback (default), or the provisional "
            "grasp calibration, as the T=104 pitch/roll target."
        ),
    )
    parser.add_argument("--dx-mm", type=float, default=0.0)
    parser.add_argument("--dy-mm", type=float, default=0.0)
    parser.add_argument("--dz-mm", type=float, default=0.0)
    parser.add_argument("--speed", type=float, default=0.05)
    parser.add_argument("--max-axis-delta-mm", type=float, default=5.0)
    parser.add_argument("--max-distance-mm", type=float, default=5.0)
    parser.add_argument("--max-pitch-change-deg", type=float, default=15.0)
    parser.add_argument("--max-roll-change-deg", type=float, default=5.0)
    parser.add_argument("--position-tolerance-mm", type=float, default=2.0)
    parser.add_argument("--orientation-tolerance-deg", type=float, default=2.0)
    parser.add_argument("--required-stable-samples", type=int, default=3)
    parser.add_argument("--motion-timeout-s", type=float, default=12.0)
    parser.add_argument("--initial-state-attempts", type=int, default=5)
    parser.add_argument("--initial-state-timeout-s", type=float, default=1.0)
    parser.add_argument(
        "--wait-for-ready",
        action="store_true",
        help=(
            "After opening serial, wait for Enter before reading the initial "
            "state. Useful because opening the ESP32 serial port may reset/home "
            "the arm. This wait is mandatory when --enable-motion is used."
        ),
    )
    parser.add_argument(
        "--enable-motion",
        action="store_true",
        help=(
            "Actually send one T=104 command. Without this flag, only validate "
            "and print the planned command."
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    controller = RoArmCartesianController(port=args.port, timeout_s=0.2)
    report: Dict[str, Any] = {
        "tool": "apriltag_block_grasp.move_cartesian_fixed_orientation_safe",
        "scope": "single_T104_relative_XYZ",
        "dry_run": not args.enable_motion,
        "enable_motion": bool(args.enable_motion),
        "camera_opened": False,
        "serial_open_can_reset_controller": True,
        "cartesian_yaw_command_supported": False,
        "yaw_B_behavior": "selected_by_firmware_inverse_kinematics",
        "gripper_field_required_by_T104": True,
        "gripper_change_requested": False,
        "fill_light_commanded": False,
        "automatic_retry_enabled": False,
        "automatic_recovery_enabled": False,
        "planned_command": None,
        "summary": {"valid": False},
    }
    try:
        if args.enable_motion:
            raise RuntimeError(
                "direct T=104 motion is disabled: two hardware tests did not "
                "preserve the requested XYZ/pitch, and the current official "
                "RoArm-M3 SDK uses T=1041 for pose control"
            )
        deltas = [
            finite_float(args.dx_mm, "dx_mm"),
            finite_float(args.dy_mm, "dy_mm"),
            finite_float(args.dz_mm, "dz_mm"),
        ]
        speed = finite_float(args.speed, "speed")
        max_axis_delta = finite_float(
            args.max_axis_delta_mm, "max_axis_delta_mm"
        )
        max_distance = finite_float(args.max_distance_mm, "max_distance_mm")
        max_pitch_change_deg = finite_float(
            args.max_pitch_change_deg, "max_pitch_change_deg"
        )
        max_roll_change_deg = finite_float(
            args.max_roll_change_deg, "max_roll_change_deg"
        )
        position_tolerance = finite_float(
            args.position_tolerance_mm, "position_tolerance_mm"
        )
        orientation_tolerance_rad = math.radians(
            finite_float(args.orientation_tolerance_deg, "orientation_tolerance_deg")
        )
        motion_timeout = finite_float(args.motion_timeout_s, "motion_timeout_s")
        initial_timeout = finite_float(
            args.initial_state_timeout_s, "initial_state_timeout_s"
        )
        if any(
            value <= 0.0
            for value in (
                speed,
                max_axis_delta,
                max_distance,
                max_pitch_change_deg,
                max_roll_change_deg,
                position_tolerance,
                orientation_tolerance_rad,
                motion_timeout,
                initial_timeout,
            )
        ):
            raise ValueError("speed, limits, tolerances and timeouts must be positive")
        if any(abs(delta) > max_axis_delta for delta in deltas):
            raise ValueError(
                f"each XYZ delta must be within +/-{max_axis_delta:.3f} mm"
            )
        distance = math.sqrt(sum(delta * delta for delta in deltas))
        if distance > max_distance:
            raise ValueError(
                f"requested XYZ distance {distance:.3f} mm exceeds "
                f"max_distance_mm={max_distance:.3f}"
            )

        controller.connect()
        if args.enable_motion or args.wait_for_ready:
            execution_notice = (
                "This test will send exactly one T=104 command after Enter.\n"
                if args.enable_motion
                else "Dry-run only: no command will be sent after Enter.\n"
            )
            print(
                "\nThe serial port is open and may have reset the RoArm controller.\n"
                "Remove the block, then adjust the arm near the intended grasp pose "
                "using a control path that does not open this serial port.\n"
                + execution_notice
                + "Press Ctrl-C to cancel, or press Enter when the workspace is safe.\n",
                flush=True,
            )
            input()
            controller.reset_input_buffer()

        initial_state = None
        initial_empty_reads = 0
        initial_attempts = max(1, int(args.initial_state_attempts))
        for _ in range(initial_attempts):
            initial_state = controller.read_state(timeout_s=initial_timeout)
            if initial_state is not None:
                break
            initial_empty_reads += 1
        if initial_state is None:
            raise RuntimeError(
                "no initial T=1051 arm state received after "
                f"{initial_attempts} attempts"
            )

        current_xyz = [state_number(initial_state, key) for key in ("x", "y", "z")]
        current_pitch = state_number(initial_state, "tit")
        current_roll = state_number(initial_state, "r")
        current_gripper = state_number(initial_state, "g")
        current_yaw = state_number(initial_state, "b")
        calibration_report = None
        if args.orientation_source == "current":
            target_pitch = current_pitch
            target_roll = current_roll
            configured_yaw = None
        else:
            calibration = load_grasp_calibration(args.calibration_path)
            target_roll = float(calibration.grasp_tool_orientation_rpy_rad[0])
            target_pitch = float(calibration.grasp_tool_orientation_rpy_rad[1])
            configured_yaw = float(calibration.grasp_tool_orientation_rpy_rad[2])
            calibration_report = {
                "path": calibration.path,
                "orientation_rpy_rad": [
                    target_roll,
                    target_pitch,
                    configured_yaw,
                ],
                "orientation_order": "Rz(yaw) @ Ry(pitch) @ Rx(roll)",
                "applied_by_T104": {
                    "roll_r": True,
                    "pitch_t": True,
                    "yaw_B": False,
                },
            }
        target_xyz = [current_xyz[i] + deltas[i] for i in range(3)]
        pitch_change_deg = math.degrees(target_pitch - current_pitch)
        roll_change_deg = math.degrees(target_roll - current_roll)
        command = controller.build_cartesian_command(
            x_mm=target_xyz[0],
            y_mm=target_xyz[1],
            z_mm=target_xyz[2],
            pitch_rad=target_pitch,
            roll_rad=target_roll,
            gripper_rad=current_gripper,
            speed=speed,
        )
        report.update(
            {
                "initial_state": compact_state(initial_state),
                "initial_state_attempt_count": initial_empty_reads + 1,
                "initial_empty_read_count": initial_empty_reads,
                "requested_delta_xyz_mm": deltas,
                "requested_distance_mm": distance,
                "orientation_source": args.orientation_source,
                "target_xyz_mm": target_xyz,
                "target_pitch_rad": target_pitch,
                "target_roll_rad": target_roll,
                "uncommanded_configured_yaw_rad": configured_yaw,
                "initial_uncommanded_yaw_B_rad": current_yaw,
                "pitch_change_deg": pitch_change_deg,
                "roll_change_deg": roll_change_deg,
                "held_gripper_rad": current_gripper,
                "planned_command": command,
            }
        )
        if calibration_report is not None:
            report["calibration"] = calibration_report
        if abs(pitch_change_deg) > max_pitch_change_deg:
            raise ValueError(
                f"target pitch change {pitch_change_deg:.3f} deg exceeds "
                f"max_pitch_change_deg={max_pitch_change_deg:.3f}"
            )
        if abs(roll_change_deg) > max_roll_change_deg:
            raise ValueError(
                f"target roll change {roll_change_deg:.3f} deg exceeds "
                f"max_roll_change_deg={max_roll_change_deg:.3f}"
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

        sent_command = controller.send_cartesian_command(
            x_mm=target_xyz[0],
            y_mm=target_xyz[1],
            z_mm=target_xyz[2],
            pitch_rad=target_pitch,
            roll_rad=target_roll,
            gripper_rad=current_gripper,
            speed=speed,
        )
        deadline = time.monotonic() + motion_timeout
        stable_samples = 0
        feedback_sample_count = 0
        final_state = None
        final_errors = None
        required_stable = max(1, int(args.required_stable_samples))
        while time.monotonic() < deadline:
            state = controller.read_state(timeout_s=0.5)
            if state is None:
                continue
            feedback_sample_count += 1
            final_state = state
            xyz = [state_number(state, key) for key in ("x", "y", "z")]
            position_error = math.sqrt(
                sum((xyz[i] - target_xyz[i]) ** 2 for i in range(3))
            )
            pitch_error = state_number(state, "tit") - target_pitch
            roll_error = state_number(state, "r") - target_roll
            final_errors = {
                "position_norm_mm": position_error,
                "pitch_deg": math.degrees(pitch_error),
                "roll_deg": math.degrees(roll_error),
            }
            if (
                position_error <= position_tolerance
                and abs(pitch_error) <= orientation_tolerance_rad
                and abs(roll_error) <= orientation_tolerance_rad
            ):
                stable_samples += 1
                if stable_samples >= required_stable:
                    break
            else:
                stable_samples = 0

        reached_target = stable_samples >= required_stable
        report.update(
            {
                "sent_command": sent_command,
                "transmitted_command_count": controller.transmitted_command_count,
                "transmitted_byte_count": controller.transmitted_byte_count,
                "feedback_sample_count": feedback_sample_count,
                "final_state": compact_state(final_state),
                "final_errors": final_errors,
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
    except (EOFError, KeyboardInterrupt):
        report["summary"] = {
            "valid": False,
            "reason": "cancelled_before_motion",
            "motion_command_sent": controller.transmitted_byte_count > 0,
            "reached_target": False,
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 130
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
