#!/usr/bin/env python3
"""Read finite RoArm T=1051 state samples without transmitting commands."""

import argparse
import json
from typing import Any, Dict, List, Optional

import numpy as np

from apriltag_block_grasp.core.rigid_transform import (
    make_transform,
    rotation_from_rpy,
    rotation_quality,
)
from apriltag_block_grasp.core.roarm_serial_readonly import RoArmSerialStateReader


def finite_number(state: Dict[str, Any], key: str) -> Optional[float]:
    if key not in state:
        return None
    try:
        value = float(state[key])
    except (TypeError, ValueError):
        return None
    return value if np.isfinite(value) else None


def distribution(values: List[float]) -> Dict[str, Any]:
    if not values:
        return {"count": 0}
    data = np.asarray(values, dtype=np.float64)
    return {
        "count": int(data.size),
        "min": float(np.min(data)),
        "median": float(np.median(data)),
        "max": float(np.max(data)),
        "peak_to_peak": float(np.ptp(data)),
    }


def pose_from_state(state: Dict[str, Any]) -> Dict[str, Any]:
    position = {key: finite_number(state, key) for key in ("x", "y", "z")}
    roll = finite_number(state, "r")
    pitch_key = "tit" if finite_number(state, "tit") is not None else "t"
    pitch = finite_number(state, pitch_key)
    yaw = finite_number(state, "b")
    required = [*position.values(), roll, pitch, yaw]
    if any(value is None for value in required):
        return {
            "valid": False,
            "reason": "required_cartesian_pose_fields_missing_or_non_finite",
            "required": ["x", "y", "z", "r", "tit_or_t", "b"],
            "available_fields": sorted(str(key) for key in state.keys()),
        }

    rotation = rotation_from_rpy(roll, pitch, yaw)
    transform = make_transform(rotation, [position["x"], position["y"], position["z"]])
    orthogonality_error, determinant = rotation_quality(rotation)
    return {
        "valid": True,
        "position_unit": "mm",
        "angle_unit": "rad",
        "rotation_order": "Rz(b/yaw) @ Ry(tit_or_t/pitch) @ Rx(r/roll)",
        "pitch_source_field": pitch_key,
        "pose_mm_rad": {
            "x": position["x"],
            "y": position["y"],
            "z": position["z"],
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
        },
        "T_base_eef_candidate": transform.tolist(),
        "rotation_quality": {
            "orthogonality_error_frobenius": orthogonality_error,
            "determinant": determinant,
        },
    }


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Read RoArm-M3 T=1051 serial state without sending commands."
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--sample-count", type=int, default=20)
    parser.add_argument("--sample-timeout-s", type=float, default=1.0)
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    requested_count = max(1, int(args.sample_count))
    reader = RoArmSerialStateReader(
        port=args.port,
        baudrate=args.baudrate,
        timeout_s=min(max(float(args.sample_timeout_s), 0.02), 0.5),
    )
    report: Dict[str, Any] = {
        "tool": "apriltag_block_grasp.probe_arm_serial_state",
        "read_only": True,
        "port": args.port,
        "baudrate": args.baudrate,
        "accepted_frame_type": 1051,
        "serial_bytes_transmitted": 0,
        "camera_opened": False,
        "handeye_loaded": False,
        "motion_commands_enabled": False,
        "samples": [],
        "summary": {"valid": False},
    }
    try:
        reader.connect()
        samples = []
        empty_reads = 0
        for index in range(requested_count):
            state = reader.read_state(timeout_s=args.sample_timeout_s)
            if state is None:
                empty_reads += 1
                continue
            samples.append(
                {
                    "index": index,
                    "raw_state": state,
                    "pose_interpretation": pose_from_state(state),
                }
            )

        valid_poses = [
            sample["pose_interpretation"]
            for sample in samples
            if sample["pose_interpretation"].get("valid", False)
        ]
        pose_fields = ("x", "y", "z", "roll", "pitch", "yaw")
        report["samples"] = samples
        report["observed_state_fields"] = sorted(
            {str(key) for sample in samples for key in sample["raw_state"].keys()}
        )
        report["pose_stability"] = {
            field: distribution(
                [pose["pose_mm_rad"][field] for pose in valid_poses]
            )
            for field in pose_fields
        }
        report["summary"] = {
            "valid": len(samples) == requested_count and len(valid_poses) == requested_count,
            "requested_count": requested_count,
            "state_frame_count": len(samples),
            "valid_pose_count": len(valid_poses),
            "empty_read_count": empty_reads,
            "motion_command_sent": False,
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report["summary"]["valid"] else 1
    except Exception as exc:
        report["summary"] = {
            "valid": False,
            "motion_command_sent": False,
            "error": f"{type(exc).__name__}: {exc}",
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 1
    finally:
        reader.close()
        print("RoArm serial reader closed; no command bytes were sent.", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
