#!/usr/bin/env python3
# English comments:
# Lightweight execution logger for field runs.
# It listens to the main runtime topics and writes compact JSONL snapshots.

import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

#这是一个轻量级的执行日志记录器，适用于实地运行。它监听主要的运行时主题，并写入紧凑的JSONL快照。
class ExecutionLoggerNode(Node):
    def __init__(self):
        super().__init__("execution_logger_node")

        self.declare_parameter("record_dir", "/home/sunrise/dog/ros2_red_block_ws/run_records")
        self.declare_parameter("flush_interval_s", 1.0)

        self.record_dir = Path(str(self.get_parameter("record_dir").value))
        self.flush_interval_s = float(self.get_parameter("flush_interval_s").value)
        self.record_dir.mkdir(parents=True, exist_ok=True)

        filename = time.strftime("run_%Y%m%d_%H%M%S.jsonl", time.localtime())
        self.record_path = self.record_dir / filename
        self.file = self.record_path.open("a", encoding="utf-8")

        self.latest_arm_state = None
        self.latest_target = None
        self.latest_visual_servo_state = None
        self.latest_command = None
        self.last_flush_time = time.time()

        self.create_subscription(String, "/roarm_m3/state", self.on_arm_state, 50)
        self.create_subscription(String, "/red_block/target_base", self.on_target, 50)
        self.create_subscription(String, "/red_block/visual_servo_state", self.on_visual_servo_state, 50)
        self.create_subscription(String, "/roarm_m3/cmd", self.on_command, 50)

        self.get_logger().info(f"Execution logger writing: {self.record_path}")

    @staticmethod
    def decode_json(text):
        try:
            return json.loads(text)
        except Exception:
            return {"raw": text}

    def write_snapshot(self, event):
        row = {
            "stamp": time.time(),
            "event": event,
            "arm_state": self.latest_arm_state,
            "target": self.latest_target,
            "visual_servo_state": self.latest_visual_servo_state,
            "command": self.latest_command,
        }

        self.file.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")
        now = time.time()
        if self.flush_interval_s <= 0.0 or now - self.last_flush_time >= self.flush_interval_s:
            self.file.flush()
            self.last_flush_time = now

    def on_arm_state(self, msg):
        self.latest_arm_state = self.decode_json(msg.data)
        self.write_snapshot("arm_state")

    def on_target(self, msg):
        self.latest_target = self.decode_json(msg.data)
        self.write_snapshot("target")

    def on_visual_servo_state(self, msg):
        self.latest_visual_servo_state = self.decode_json(msg.data)
        self.write_snapshot("visual_servo_state")

    def on_command(self, msg):
        self.latest_command = self.decode_json(msg.data)
        self.write_snapshot("command")

    def destroy_node(self):
        try:
            self.file.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ExecutionLoggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
