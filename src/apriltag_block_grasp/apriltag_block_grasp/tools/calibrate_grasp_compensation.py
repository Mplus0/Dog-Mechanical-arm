#!/usr/bin/env python3
"""Interactively record XYZ and gripper calibration without sending commands."""

import argparse
import json
import math
import shutil
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from apriltag_block_grasp.core.manual_compensation import (
    apply_values,
    corrected_offset,
    finite_xyz,
    load_calibration_documents,
    validate_gripper_angle,
)


def source_config_candidate() -> Path:
    return Path.cwd() / "src" / "apriltag_block_grasp" / "config"


class StateReader(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("apriltag_grasp_compensation_calibrator")
        self._lock = threading.Lock()
        self._state: Optional[Dict[str, float]] = None
        self._received_monotonic: Optional[float] = None
        self.create_subscription(String, topic, self._on_state, 10)

    def _on_state(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            state = payload.get("state")
            if not bool(payload.get("state_valid", False)) or not isinstance(state, dict):
                return
            parsed = {
                key: float(state[key]) for key in ("x", "y", "z", "g")
            }
            if not all(math.isfinite(value) for value in parsed.values()):
                return
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        with self._lock:
            self._state = parsed
            self._received_monotonic = time.monotonic()

    def current(self, maximum_age_s: float = 1.0) -> Optional[Dict[str, float]]:
        with self._lock:
            if self._state is None or self._received_monotonic is None:
                return None
            if time.monotonic() - self._received_monotonic > maximum_age_s:
                return None
            return dict(self._state)


class InteractiveCalibration:
    def __init__(self, node: StateReader, config_dir: Path) -> None:
        self.node = node
        self.config_dir = config_dir
        self.grasp_path = config_dir / "grasp_calibration.json"
        self.motion_path = config_dir / "motion_control.json"
        self.grasp, self.motion = load_calibration_documents(
            self.grasp_path, self.motion_path
        )
        self.original_offset = finite_xyz(
            self.grasp["final_grasp_tcp_offset_base_mm"], "original_offset"
        )
        self.candidate_offset = self.original_offset
        self.open_angle = validate_gripper_angle(
            self.motion["gripper_open"]["angle_deg"], "open angle"
        )
        self.close_angle = validate_gripper_angle(
            self.motion["pick_sequence"]["close_gripper"]["angle_deg"],
            "close angle",
        )
        self.automatic_xyz: Optional[Tuple[float, float, float]] = None
        self.corrected_xyz: Optional[Tuple[float, float, float]] = None
        self.trim_xyz = (0.0, 0.0, 0.0)

    def current_state(self) -> Dict[str, float]:
        state = self.node.current()
        if state is None:
            raise RuntimeError("no fresh /roarm_m3/state feedback")
        return state

    @staticmethod
    def xyz(state: Dict[str, float]) -> Tuple[float, float, float]:
        return state["x"], state["y"], state["z"]

    @staticmethod
    def state_text(state: Dict[str, float]) -> str:
        return (
            f"XYZ=[{state['x']:.3f}, {state['y']:.3f}, {state['z']:.3f}] mm, "
            f"g={state['g']:.6f} rad ({math.degrees(state['g']):.3f} deg)"
        )

    def recompute(self) -> None:
        if self.automatic_xyz is None or self.corrected_xyz is None:
            return
        self.candidate_offset = corrected_offset(
            self.original_offset,
            self.automatic_xyz,
            self.corrected_xyz,
            self.trim_xyz,
        )

    def show(self) -> None:
        state = self.node.current()
        print("\nCalibration preview")
        print(f"  config_dir: {self.config_dir}")
        print(f"  live_state: {self.state_text(state) if state else 'unavailable'}")
        print(f"  automatic_xyz: {self.automatic_xyz}")
        print(f"  corrected_xyz: {self.corrected_xyz}")
        print(f"  manual_trim_mm: {self.trim_xyz}")
        print(f"  original_offset_mm: {self.original_offset}")
        print(f"  candidate_offset_mm: {self.candidate_offset}")
        print(f"  open_angle_deg: {self.open_angle:.3f}")
        print(f"  close_angle_deg: {self.close_angle:.3f}\n")

    def capture(self, destination: str) -> None:
        point = self.xyz(self.current_state())
        if destination == "auto":
            self.automatic_xyz = point
        else:
            self.corrected_xyz = point
        self.recompute()
        print(f"Recorded {destination}: {point}")

    def save(self) -> None:
        apply_values(
            self.grasp,
            self.motion,
            self.candidate_offset,
            self.open_angle,
            self.close_angle,
        )
        self.grasp["last_manual_compensation"] = {
            "saved_at_local": datetime.now().astimezone().isoformat(),
            "automatic_xyz_mm": self.automatic_xyz,
            "corrected_xyz_mm": self.corrected_xyz,
            "manual_trim_mm": self.trim_xyz,
            "result_offset_mm": self.candidate_offset,
            "tool_sent_motion_commands": False,
        }
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backups = []
        for path in (self.grasp_path, self.motion_path):
            backup = path.with_name(f"{path.name}.bak.{stamp}")
            shutil.copy2(path, backup)
            backups.append(backup)
        self.grasp_path.write_text(
            json.dumps(self.grasp, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        self.motion_path.write_text(
            json.dumps(self.motion, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print("Saved configuration.")
        for backup in backups:
            print(f"  backup: {backup}")

    def run(self) -> None:
        print("AprilTag grasp manual compensation")
        print("READ ONLY: this tool never publishes a motion or gripper command.")
        print("Type 'help' for commands.")
        self.show()
        while True:
            try:
                parts = input("calibrate> ").strip().split()
            except (EOFError, KeyboardInterrupt):
                print("\nExited without an implicit save.")
                return
            if not parts:
                continue
            command = parts[0].lower()
            try:
                if command in ("quit", "exit"):
                    print("Exited without an implicit save.")
                    return
                if command == "help":
                    print(
                        "status | live [seconds] | auto | corrected | "
                        "trim X Y Z | offset X Y Z | open DEG | close DEG | "
                        "use-g open|close | show | save | quit"
                    )
                elif command == "status":
                    print(self.state_text(self.current_state()))
                elif command == "live":
                    duration = 10.0 if len(parts) == 1 else float(parts[1])
                    deadline = time.monotonic() + max(0.1, duration)
                    while time.monotonic() < deadline:
                        state = self.node.current()
                        print(
                            "\r" + (self.state_text(state) if state else "state unavailable") + "   ",
                            end="",
                            flush=True,
                        )
                        time.sleep(0.25)
                    print()
                elif command in ("auto", "corrected"):
                    self.capture(command)
                elif command == "trim" and len(parts) == 4:
                    self.trim_xyz = finite_xyz(parts[1:], "trim")
                    self.recompute()
                    self.show()
                elif command == "offset" and len(parts) == 4:
                    self.candidate_offset = finite_xyz(parts[1:], "offset")
                    self.show()
                elif command in ("open", "close") and len(parts) == 2:
                    angle = validate_gripper_angle(parts[1], command)
                    if command == "open":
                        self.open_angle = angle
                    else:
                        self.close_angle = angle
                    self.show()
                elif command == "use-g" and len(parts) == 2 and parts[1] in ("open", "close"):
                    angle = validate_gripper_angle(
                        math.degrees(self.current_state()["g"]), "current g"
                    )
                    if parts[1] == "open":
                        self.open_angle = angle
                    else:
                        self.close_angle = angle
                    self.show()
                elif command == "show":
                    self.show()
                elif command == "save":
                    self.show()
                    if input("Type SAVE to write both JSON files: ").strip() == "SAVE":
                        self.save()
                    else:
                        print("Save cancelled.")
                else:
                    print("Unknown or malformed command; type 'help'.")
            except Exception as exc:
                print(f"ERROR: {type(exc).__name__}: {exc}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--state-topic", default="/roarm_m3/state")
    parser.add_argument("--config-dir", default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.config_dir:
        config_dir = Path(args.config_dir).expanduser().resolve()
    elif source_config_candidate().is_dir():
        config_dir = source_config_candidate().resolve()
    else:
        config_dir = Path(
            get_package_share_directory("apriltag_block_grasp")
        ) / "config"
    rclpy.init()
    node = StateReader(args.state_topic)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        InteractiveCalibration(node, config_dir).run()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
