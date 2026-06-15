#!/usr/bin/env python3
# 中文说明：
# RoArm-M3 ROS2 驱动节点。
#
# 设计原则：
# 1. 本节点是唯一占用 /dev/ttyUSB0 的节点。
# 2. 其他节点不能直接连接机械臂串口。
# 3. 本节点发布 /roarm_m3/state。
# 4. 本节点订阅 /roarm_m3/cmd。
#
# /roarm_m3/state:
#   std_msgs/String，JSON 格式，包含当前机械臂状态。
#
# /roarm_m3/cmd:
#   std_msgs/String，JSON 格式。
#
# 支持命令示例：
#
# 1. 设置 LED：
# {
#   "type": "set_led",
#   "led": 120
# }
#
# 2. 移动单个关节：
# {
#   "type": "move_joint",
#   "joint": 1,
#   "angle": 0.0,
#   "speed": 35.0,
#   "acc": 35.0
# }
#
# 3. 移动末端位姿：
# {
#   "type": "move_pose",
#   "x": 280.0,
#   "y": -140.0,
#   "z": 120.0,
#   "t": 1.23,
#   "r": -1.57,
#   "g": 3.14,
#   "speed": 0.15
# }
#
# 4. 回初始姿态：
# {
#   "type": "set_initial_pose",
#   "targets_deg": {
#     "b": 0.0,
#     "s": 0.0,
#     "e": 70.0,
#     "t": 90.0,
#     "r": -90.0,
#     "g": null
#   },
#   "speed": 35.0,
#   "acc": 35.0
# }

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from red_block_grasp_ros2.roarm_m3 import RoArmM3


class RoArmDriverNode(Node):
    def __init__(self):
        super().__init__("roarm_driver_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("state_period", 0.2)
        self.declare_parameter("auto_connect", True)
        self.declare_parameter("enable_fill_light", False)
        self.declare_parameter("fill_light_led", 120)

        self.port = self.get_parameter("port").value
        self.state_period = float(self.get_parameter("state_period").value)
        self.auto_connect = bool(self.get_parameter("auto_connect").value)
        self.enable_fill_light = self.parse_bool(self.get_parameter("enable_fill_light").value)
        self.fill_light_led = max(0, min(255, int(self.get_parameter("fill_light_led").value)))

        self.pub_state = self.create_publisher(String, "/roarm_m3/state", 10)
        self.sub_cmd = self.create_subscription(String, "/roarm_m3/cmd", self.on_cmd, 10)

        self.arm = RoArmM3(port=self.port)
        self.connected = False
        self.last_state = None

        if self.auto_connect:
            self.connect_arm()

        self.timer = self.create_timer(self.state_period, self.publish_state)

        self.get_logger().info("RoArm driver node started.")
        self.get_logger().info(f"port = {self.port}")
        self.get_logger().info(f"state_period = {self.state_period}")
        self.get_logger().info(
            f"enable_fill_light = {self.enable_fill_light}, fill_light_led = {self.fill_light_led}"
        )

    @staticmethod
    def parse_bool(value):
        if isinstance(value, bool):
            return value
        if value is None:
            return False
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def connect_arm(self):
        try:
            self.arm.connect()
            self.connected = True
            self.get_logger().info("RoArm-M3 connected.")
            self.apply_fill_light_default()
        except Exception as e:
            self.connected = False
            self.get_logger().error(f"Failed to connect RoArm-M3: {e}")

    def apply_fill_light_default(self):
        led = self.fill_light_led if self.enable_fill_light else 0
        raw = {
            "T": 114,
            "led": led,
        }
        self.get_logger().info(
            f"Applying fill light default after connect: {'on' if self.enable_fill_light else 'off'} (led={led})"
        )
        self.arm.send_cmd(raw)

    def publish_state(self):
        msg_dict = {
            "stamp": time.time(),
            "connected": self.connected,
            "state_valid": False,
            "state": None,
        }

        if not self.connected:
            msg = String()
            msg.data = json.dumps(msg_dict, ensure_ascii=False)
            self.pub_state.publish(msg)
            return

        try:
            state = self.arm.read_latest_state(duration_s=0.05)
        except Exception as e:
            self.get_logger().warn(f"Failed to read arm state: {e}")
            state = None

        if state is not None:
            self.last_state = state
            msg_dict["state_valid"] = True
            msg_dict["state"] = self.normalize_state(state)
        else:
            msg_dict["state_valid"] = False
            msg_dict["state"] = self.normalize_state(self.last_state) if self.last_state is not None else None

        msg = String()
        msg.data = json.dumps(msg_dict, ensure_ascii=False)
        self.pub_state.publish(msg)

    @staticmethod
    def normalize_state(state):
        if state is None:
            return None

        out = {}
        for key, value in state.items():
            try:
                out[key] = float(value)
            except Exception:
                out[key] = value

        return out

    def on_cmd(self, msg):
        try:
            cmd = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Invalid JSON command: {e}")
            return

        cmd_type = cmd.get("type", "")

        try:
            if cmd_type == "connect":
                self.connect_arm()

            elif cmd_type == "set_led":
                self.handle_set_led(cmd)

            elif cmd_type == "move_joint":
                self.handle_move_joint(cmd)

            elif cmd_type == "move_pose":
                self.handle_move_pose(cmd)

            elif cmd_type == "set_initial_pose":
                self.handle_set_initial_pose(cmd)

            elif cmd_type == "raw":
                self.handle_raw(cmd)

            else:
                self.get_logger().warn(f"Unknown command type: {cmd_type}")

        except Exception as e:
            self.get_logger().error(f"Failed to execute command {cmd_type}: {e}")

    def send_cmd(self, cmd):
        if not self.connected:
            self.get_logger().warn("Arm is not connected. Try connecting first.")
            self.connect_arm()

        self.get_logger().info("SEND: " + json.dumps(cmd, separators=(",", ":")))
        self.arm.send_cmd(cmd)

    def handle_set_led(self, cmd):
        led = int(cmd.get("led", 120))
        led = max(0, min(255, led))

        raw = {
            "T": 114,
            "led": led,
        }

        self.send_cmd(raw)

    def handle_move_joint(self, cmd):
        joint = int(cmd["joint"])
        angle = float(cmd["angle"])
        speed = float(cmd.get("speed", cmd.get("spd", 35.0)))
        acc = float(cmd.get("acc", 35.0))

        raw = {
            "T": 121,
            "joint": joint,
            "angle": angle,
            "spd": speed,
            "acc": acc,
        }

        self.send_cmd(raw)

    def handle_move_pose(self, cmd):
        raw = {
            "T": 104,
            "x": float(cmd["x"]),
            "y": float(cmd["y"]),
            "z": float(cmd["z"]),
            "t": float(cmd["t"]),
            "r": float(cmd["r"]),
            "g": float(cmd["g"]),
            "spd": float(cmd.get("speed", cmd.get("spd", 0.15))),
        }

        self.send_cmd(raw)

    @staticmethod
    def joint_name_to_id():
        return {
            "b": 1,
            "s": 2,
            "e": 3,
            "t": 4,
            "r": 5,
            "g": 6,
        }

    def handle_set_initial_pose(self, cmd):
        targets_deg = cmd.get("targets_deg", {})
        speed = float(cmd.get("speed", cmd.get("spd", 35.0)))
        acc = float(cmd.get("acc", 35.0))
        move_order = cmd.get("move_order", ["b", "t", "r", "s", "e"])

        if not isinstance(targets_deg, dict):
            raise ValueError("targets_deg must be a dict.")

        joint_map = self.joint_name_to_id()

        for name in move_order:
            if name not in targets_deg:
                continue

            target = targets_deg[name]
            if target is None:
                continue

            if name not in joint_map:
                self.get_logger().warn(f"Unknown joint name: {name}")
                continue

            raw = {
                "T": 121,
                "joint": joint_map[name],
                "angle": float(target),
                "spd": speed,
                "acc": acc,
            }

            self.send_cmd(raw)
            time.sleep(0.1)

    def handle_raw(self, cmd):
        raw = cmd.get("cmd", None)

        if not isinstance(raw, dict):
            raise ValueError("raw command must contain dict field 'cmd'.")

        self.send_cmd(raw)

    def destroy_node(self):
        try:
            self.arm.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoArmDriverNode()

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
