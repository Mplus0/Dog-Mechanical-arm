#!/usr/bin/env python3
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CompetitionArmTaskNode(Node):
    def __init__(self):
        super().__init__("competition_arm_task_node")

        self.declare_parameter("task_cmd_topic", "/dog_arm/task_cmd")
        self.declare_parameter("task_result_topic", "/dog_arm/task_result")
        self.declare_parameter("base_adjust_req_topic", "/dog_arm/base_adjust_req")
        self.declare_parameter("visual_servo_cmd_topic", "/red_block/visual_servo_cmd")
        self.declare_parameter("visual_servo_state_topic", "/red_block/visual_servo_state")
        self.declare_parameter("pick_timeout_s", 180.0)
        self.declare_parameter("place_timeout_s", 60.0)
        self.declare_parameter("base_adjust_step_m", 0.05)

        self.task_cmd_topic = str(self.get_parameter("task_cmd_topic").value)
        self.task_result_topic = str(self.get_parameter("task_result_topic").value)
        self.base_adjust_req_topic = str(self.get_parameter("base_adjust_req_topic").value)
        self.visual_servo_cmd_topic = str(self.get_parameter("visual_servo_cmd_topic").value)
        self.visual_servo_state_topic = str(self.get_parameter("visual_servo_state_topic").value)
        self.pick_timeout_s = float(self.get_parameter("pick_timeout_s").value)
        self.place_timeout_s = float(self.get_parameter("place_timeout_s").value)
        self.base_adjust_step_m = float(self.get_parameter("base_adjust_step_m").value)

        self.pub_result = self.create_publisher(String, self.task_result_topic, 10)
        self.pub_base_adjust = self.create_publisher(String, self.base_adjust_req_topic, 10)
        self.pub_visual_cmd = self.create_publisher(String, self.visual_servo_cmd_topic, 10)

        self.sub_task_cmd = self.create_subscription(String, self.task_cmd_topic, self.on_task_cmd, 10)
        self.sub_visual_state = self.create_subscription(
            String,
            self.visual_servo_state_topic,
            self.on_visual_state,
            10,
        )

        self.active_task_id = None
        self.active_cmd = None
        self.active_start_time = 0.0
        self.result_sent = False
        self.base_adjust_sent = False

        self.timer = self.create_timer(0.5, self.on_timer)

        self.get_logger().info(
            "Competition arm task node started. "
            f"cmd={self.task_cmd_topic}, result={self.task_result_topic}, "
            f"base_adjust={self.base_adjust_req_topic}"
        )

    def publish_json(self, publisher, data, label):
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        publisher.publish(msg)
        self.get_logger().info(f"PUB {label}: {msg.data}")

    def publish_result(self, task_id, result, error=None, clear_active=True):
        data = {
            "task_id": task_id,
            "result": result,
        }
        if error:
            data["error"] = str(error)
        self.publish_json(self.pub_result, data, self.task_result_topic)
        if clear_active:
            self.result_sent = True
            self.active_task_id = None
            self.active_cmd = None
            self.active_start_time = 0.0
            self.base_adjust_sent = False

    def publish_visual_cmd(self, task_id, cmd):
        self.publish_json(
            self.pub_visual_cmd,
            {
                "task_id": task_id,
                "cmd": cmd,
            },
            self.visual_servo_cmd_topic,
        )

    def on_task_cmd(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as exc:
            self.publish_result(
                None,
                "task_failed",
                f"invalid_json:{exc}",
                clear_active=self.active_cmd is None,
            )
            return

        task_id = data.get("task_id", None)
        cmd = str(data.get("cmd", "")).strip()

        if cmd not in ("pick", "place_to_zone"):
            self.publish_result(
                task_id,
                "task_failed",
                f"unknown_cmd:{cmd}",
                clear_active=self.active_cmd is None,
            )
            return

        if self.active_cmd is not None:
            fail_result = "pick_failed" if cmd == "pick" else "place_failed"
            self.publish_result(task_id, fail_result, "arm_task_busy", clear_active=False)
            return

        self.active_task_id = task_id
        self.active_cmd = cmd
        self.active_start_time = time.time()
        self.result_sent = False
        self.base_adjust_sent = False
        self.publish_visual_cmd(task_id, cmd)

    def on_visual_state(self, msg):
        if self.active_cmd is None or self.result_sent:
            return

        try:
            state = json.loads(msg.data)
        except Exception:
            return

        state_task_id = state.get("task_id", None)
        if state_task_id != self.active_task_id:
            return

        base_adjust = state.get("base_adjust", None)
        error = state.get("error", None)
        visual_state = str(state.get("state", ""))
        task_result = state.get("task_result", None) or state.get("result", None)

        if error == "need_base_adjust" and isinstance(base_adjust, dict):
            self.relay_base_adjust(base_adjust)
            self.publish_result(self.active_task_id, "pick_failed", "need_base_adjust")
            return

        if task_result == "pick_success":
            self.publish_result(self.active_task_id, "pick_success")
            return

        if task_result == "place_success":
            self.publish_result(self.active_task_id, "place_success")
            return

        if visual_state == "FAIL":
            if self.active_cmd == "pick":
                self.publish_result(self.active_task_id, "pick_failed", error or "pick_motion_failed")
            else:
                self.publish_result(self.active_task_id, "place_failed", error or "place_motion_failed")

    def relay_base_adjust(self, base_adjust):
        if self.base_adjust_sent:
            return

        direction = str(base_adjust.get("direction", "")).strip()
        if direction not in ("left", "right"):
            self.get_logger().warn(f"Ignoring invalid base adjust direction: {direction}")
            return

        data = {
            "task_id": self.active_task_id,
            "direction": direction,
            "step_m": float(base_adjust.get("step_m", self.base_adjust_step_m)),
            "reason": str(base_adjust.get("reason", "need_base_adjust")),
        }
        self.publish_json(self.pub_base_adjust, data, self.base_adjust_req_topic)
        self.base_adjust_sent = True

    def on_timer(self):
        if self.active_cmd is None or self.result_sent:
            return

        elapsed = time.time() - self.active_start_time
        timeout_s = self.pick_timeout_s if self.active_cmd == "pick" else self.place_timeout_s
        if elapsed <= timeout_s:
            return

        if self.active_cmd == "pick":
            self.publish_result(self.active_task_id, "pick_failed", "task_timeout")
        else:
            self.publish_result(self.active_task_id, "place_failed", "task_timeout")


def main(args=None):
    rclpy.init(args=args)
    node = CompetitionArmTaskNode()

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
