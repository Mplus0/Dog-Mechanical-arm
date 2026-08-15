#!/usr/bin/env python3
"""Interactively record a fixed placement XYZ without sending arm commands."""

import argparse
import json
import math
import threading
import time
from pathlib import Path
from typing import Dict, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from red_block_grasp_mplus0.core.place_pose_calibration import (
    POSE_KEYS,
    apply_pose,
    load_document,
    pose_from_document,
    save_document,
    validate_pose,
)


def source_config_candidate() -> Path:
    return (
        Path.cwd()
        / "src"
        / "red_block_grasp_mplus0"
        / "config"
        / "place_pose.yaml"
    )


class StateReader(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("fixed_place_pose_calibrator")
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
            parsed = {key: float(state[key]) for key in ("x", "y", "z")}
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


class InteractivePlaceCalibration:
    def __init__(self, node: StateReader, config_path: Path) -> None:
        self.node = node
        self.config_path = config_path
        self.document = load_document(config_path)
        self.original = pose_from_document(self.document)
        self.candidate = dict(self.original)

    @staticmethod
    def format_pose(pose: Dict[str, float]) -> str:
        return (
            f"XYZ=[{pose['place_x_mm']:.3f}, {pose['place_y_mm']:.3f}, "
            f"{pose['place_z_mm']:.3f}] mm"
        )

    @staticmethod
    def state_as_pose(state: Dict[str, float]) -> Dict[str, float]:
        return {
            "place_x_mm": state["x"],
            "place_y_mm": state["y"],
            "place_z_mm": state["z"],
        }

    def current_pose(self) -> Dict[str, float]:
        state = self.node.current()
        if state is None:
            raise RuntimeError("no fresh /roarm_m3/state feedback")
        return validate_pose(self.state_as_pose(state))

    def show(self) -> None:
        live = self.node.current()
        live_text = "unavailable"
        if live is not None:
            try:
                live_text = self.format_pose(validate_pose(self.state_as_pose(live)))
            except (KeyError, ValueError):
                live_text = "available but outside configured workspace"
        print("\nFixed placement calibration preview")
        print(f"  config:    {self.config_path}")
        print(f"  live:      {live_text}")
        print(f"  original:  {self.format_pose(self.original)}")
        print(f"  candidate: {self.format_pose(self.candidate)}\n")

    def capture(self) -> None:
        self.candidate = self.current_pose()
        print(f"Captured current arm feedback: {self.format_pose(self.candidate)}")

    def set_pose(self, x: str, y: str, z: str) -> None:
        self.candidate = validate_pose(
            {"place_x_mm": x, "place_y_mm": y, "place_z_mm": z}
        )
        self.show()

    def nudge(self, axis: str, delta_text: str) -> None:
        key = f"place_{axis.lower()}_mm"
        if key not in POSE_KEYS:
            raise ValueError("axis must be x, y, or z")
        delta = float(delta_text)
        if not math.isfinite(delta):
            raise ValueError("nudge delta must be finite")
        candidate = dict(self.candidate)
        candidate[key] += delta
        self.candidate = validate_pose(candidate)
        self.show()

    def save(self) -> None:
        apply_pose(self.document, self.candidate)
        backup = save_document(self.config_path, self.document)
        self.original = dict(self.candidate)
        print(f"Saved:  {self.config_path}")
        print(f"Backup: {backup}")

    def run(self) -> None:
        print("Fixed placement coordinate calibration")
        print("READ ONLY: this tool never publishes motion or gripper commands.")
        print("Move the arm with the approved manual/web control, then use 'capture'.")
        print("Type 'help' for commands.")
        self.show()
        while True:
            try:
                parts = input("place-calibrate> ").strip().split()
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
                        "status | live [seconds] | capture | set X Y Z | "
                        "nudge x|y|z DELTA_MM | show | save | quit"
                    )
                elif command == "status":
                    print(self.format_pose(self.current_pose()))
                elif command == "live":
                    duration = 10.0 if len(parts) == 1 else float(parts[1])
                    deadline = time.monotonic() + max(0.1, duration)
                    while time.monotonic() < deadline:
                        state = self.node.current()
                        text = "state unavailable"
                        if state is not None:
                            text = self.format_pose(self.state_as_pose(state))
                        print(f"\r{text}   ", end="", flush=True)
                        time.sleep(0.25)
                    print()
                elif command == "capture" and len(parts) == 1:
                    self.capture()
                elif command == "set" and len(parts) == 4:
                    self.set_pose(parts[1], parts[2], parts[3])
                elif command == "nudge" and len(parts) == 3:
                    self.nudge(parts[1], parts[2])
                elif command == "show" and len(parts) == 1:
                    self.show()
                elif command == "save" and len(parts) == 1:
                    self.show()
                    answer = input("Type SAVE to write place_pose.yaml: ").strip()
                    if answer == "SAVE":
                        self.save()
                    else:
                        print("Save cancelled.")
                else:
                    print("Unknown or malformed command; type 'help'.")
            except Exception as exc:
                print(f"ERROR: {type(exc).__name__}: {exc}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read arm feedback and save the fixed place XYZ."
    )
    parser.add_argument("--state-topic", default="/roarm_m3/state")
    parser.add_argument("--config", default=None)
    return parser.parse_args()


def resolve_config_path(argument: Optional[str]) -> Path:
    if argument:
        return Path(argument).expanduser().resolve()
    source_path = source_config_candidate()
    if source_path.is_file():
        return source_path.resolve()
    return (
        Path(get_package_share_directory("red_block_grasp_mplus0"))
        / "config"
        / "place_pose.yaml"
    )


def main() -> None:
    args = parse_args()
    config_path = resolve_config_path(args.config)
    rclpy.init()
    node = StateReader(args.state_topic)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        InteractivePlaceCalibration(node, config_path).run()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
