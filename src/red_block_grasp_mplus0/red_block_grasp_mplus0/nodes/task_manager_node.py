#!/usr/bin/env python3
# 中文说明：
# 红色物料任务管理节点。
#
# 本节点不直接连接相机，也不直接连接机械臂串口。
#
# 订阅：
#   /roarm_m3/state
#   /red_block/target_base
#
# 发布：
#   /roarm_m3/cmd
#   /red_block/task_state
#
# 当前任务：
#   1. 发送回初始姿态命令
#   2. 等待 red#0 的 base 坐标稳定
#   3. 生成红块正上方目标点
#   4. 分段发布 move_pose 命令
#   5. 移动完成后停止
#
# 当前阶段不执行夹爪闭合。

import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TaskManagerNode(Node):
    def __init__(self):
        super().__init__("task_manager_node")

        # ===== 基础参数 =====
        self.declare_parameter("auto_start", True)
        self.declare_parameter("move_once_only", True)

        # ===== 初始姿态，单位：度 =====
        self.declare_parameter("init_b_deg", 0.0)
        self.declare_parameter("init_s_deg", 0.0)
        self.declare_parameter("init_e_deg", 70.0)
        self.declare_parameter("init_t_deg", 90.0)
        self.declare_parameter("init_r_deg", -90.0)
        self.declare_parameter("init_speed_deg_s", 35.0)
        self.declare_parameter("init_acc", 35.0)

        # ===== 搜索策略：找不到红块时只改变 b 角 =====
        self.declare_parameter("enable_b_scan", True)
        self.declare_parameter("b_scan_offsets", "0,-8,8,-16,16,-24,24")
        self.declare_parameter("scan_timeout_s", 8.0)
        self.declare_parameter("after_scan_pose_wait_s", 2.5)

        # ===== 目标稳定判断 =====
        self.declare_parameter("stable_count_required", 3)
        self.declare_parameter("stable_position_tol_mm", 25.0)
        self.declare_parameter("target_timeout_s", 1.0)

        # ===== 移动参数 =====
        self.declare_parameter("safe_above_offset_mm", 120.0)
        self.declare_parameter("grasp_offset_x_mm", 0.0)
        self.declare_parameter("grasp_offset_y_mm", 0.0)
        self.declare_parameter("grasp_offset_z_mm", 0.0)
        self.declare_parameter("min_safe_z_mm", 30.0)
        self.declare_parameter("max_step_mm", 60.0)
        self.declare_parameter("move_speed", 0.15)
        self.declare_parameter("direct_move_to_target", True)
        self.declare_parameter("step_wait_s", 3.0)

        # ===== 工作空间保护 =====
        self.declare_parameter("base_x_min", 80.0)
        self.declare_parameter("base_x_max", 700.0)
        self.declare_parameter("base_y_min", -450.0)
        self.declare_parameter("base_y_max", 450.0)
        self.declare_parameter("base_z_min", -30.0)
        self.declare_parameter("base_z_max", 380.0)

        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.move_once_only = bool(self.get_parameter("move_once_only").value)

        self.init_pose = {
            "b": float(self.get_parameter("init_b_deg").value),
            "s": float(self.get_parameter("init_s_deg").value),
            "e": float(self.get_parameter("init_e_deg").value),
            "t": float(self.get_parameter("init_t_deg").value),
            "r": float(self.get_parameter("init_r_deg").value),
            "g": None,
        }
        self.init_speed_deg_s = float(self.get_parameter("init_speed_deg_s").value)
        self.init_acc = float(self.get_parameter("init_acc").value)

        self.enable_b_scan = bool(self.get_parameter("enable_b_scan").value)
        self.b_scan_offsets = self.parse_offsets(self.get_parameter("b_scan_offsets").value)
        self.scan_timeout_s = float(self.get_parameter("scan_timeout_s").value)
        self.after_scan_pose_wait_s = float(self.get_parameter("after_scan_pose_wait_s").value)

        self.stable_count_required = int(self.get_parameter("stable_count_required").value)
        self.stable_position_tol_mm = float(self.get_parameter("stable_position_tol_mm").value)
        self.target_timeout_s = float(self.get_parameter("target_timeout_s").value)

        self.safe_above_offset_mm = float(self.get_parameter("safe_above_offset_mm").value)
        self.grasp_offset_x_mm = float(self.get_parameter("grasp_offset_x_mm").value)
        self.grasp_offset_y_mm = float(self.get_parameter("grasp_offset_y_mm").value)
        self.grasp_offset_z_mm = float(self.get_parameter("grasp_offset_z_mm").value)
        self.min_safe_z_mm = float(self.get_parameter("min_safe_z_mm").value)
        self.max_step_mm = float(self.get_parameter("max_step_mm").value)
        self.move_speed = float(self.get_parameter("move_speed").value)
        self.direct_move_to_target = bool(self.get_parameter("direct_move_to_target").value)
        self.step_wait_s = float(self.get_parameter("step_wait_s").value)

        self.base_x_min = float(self.get_parameter("base_x_min").value)
        self.base_x_max = float(self.get_parameter("base_x_max").value)
        self.base_y_min = float(self.get_parameter("base_y_min").value)
        self.base_y_max = float(self.get_parameter("base_y_max").value)
        self.base_z_min = float(self.get_parameter("base_z_min").value)
        self.base_z_max = float(self.get_parameter("base_z_max").value)

        self.pub_cmd = self.create_publisher(String, "/roarm_m3/cmd", 10)
        self.pub_task_state = self.create_publisher(String, "/red_block/task_state", 10)

        self.sub_arm_state = self.create_subscription(
            String,
            "/roarm_m3/state",
            self.on_arm_state,
            10,
        )
        self.sub_target = self.create_subscription(
            String,
            "/red_block/target_base",
            self.on_target,
            10,
        )

        self.latest_arm_state = None
        self.latest_arm_state_time = 0.0

        self.latest_target = None
        self.latest_target_time = 0.0

        self.prev_target_base = None
        self.stable_count = 0

        self.state = "INIT"
        self.started = False
        self.moved_once = False
        self.busy = False

        self.scan_index = 0
        self.last_scan_switch_time = time.time()
        self.last_valid_target_time = 0.0

        self.timer = self.create_timer(0.2, self.on_timer)

        self.get_logger().info("Task manager node started.")
        self.get_logger().info("This node does not use camera or serial directly.")

    @staticmethod
    def parse_offsets(text):
        result = []
        for item in str(text).split(","):
            item = item.strip()
            if not item:
                continue
            result.append(float(item))
        return result if result else [0.0]

    def publish_cmd(self, cmd):
        msg = String()
        msg.data = json.dumps(cmd, ensure_ascii=False)
        self.pub_cmd.publish(msg)
        self.get_logger().info("PUB /roarm_m3/cmd: " + msg.data)

    def publish_task_state(self, extra=None):
        data = {
            "stamp": time.time(),
            "state": self.state,
            "started": self.started,
            "busy": self.busy,
            "moved_once": self.moved_once,
            "stable_count": self.stable_count,
            "scan_index": self.scan_index,
            "latest_arm_state_age_s": None,
            "latest_target_age_s": None,
        }

        now = time.time()
        if self.latest_arm_state is not None:
            data["latest_arm_state_age_s"] = now - self.latest_arm_state_time
        if self.latest_target is not None:
            data["latest_target_age_s"] = now - self.latest_target_time

        if extra:
            data.update(extra)

        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        self.pub_task_state.publish(msg)

    def on_arm_state(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        if not data.get("connected", False):
            return

        if not data.get("state_valid", False):
            return

        state = data.get("state", None)
        if not isinstance(state, dict):
            return

        self.latest_arm_state = state
        self.latest_arm_state_time = time.time()

    def on_target(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        if not data.get("valid", False):
            return

        base = data.get("base_mm", None)
        if not isinstance(base, dict):
            return

        self.latest_target = data
        self.latest_target_time = time.time()
        self.last_valid_target_time = time.time()

        self.update_stability(base)

    def update_stability(self, base):
        current = {
            "x": float(base["x"]),
            "y": float(base["y"]),
            "z": float(base["z"]),
        }

        if self.prev_target_base is None:
            self.stable_count = 1
        else:
            dist = self.dist3(current, self.prev_target_base)
            if dist <= self.stable_position_tol_mm:
                self.stable_count += 1
            else:
                self.stable_count = 1

        self.prev_target_base = current

    @staticmethod
    def dist3(a, b):
        return math.sqrt(
            (a["x"] - b["x"]) ** 2
            + (a["y"] - b["y"]) ** 2
            + (a["z"] - b["z"]) ** 2
        )

    def send_initial_pose(self, b_offset_deg=0.0):
        target = dict(self.init_pose)
        target["b"] = float(self.init_pose["b"]) + float(b_offset_deg)

        cmd = {
            "type": "set_initial_pose",
            "targets_deg": target,
            "move_order": ["b", "t", "r", "s", "e"],
            "speed": self.init_speed_deg_s,
            "acc": self.init_acc,
        }

        self.publish_cmd(cmd)

    def get_pose_from_arm_state(self):
        if self.latest_arm_state is None:
            return None

        s = self.latest_arm_state

        x = float(s.get("x", 0.0))
        y = float(s.get("y", 0.0))
        z = float(s.get("z", 0.0))

        t = self.get_pose_angle_rad(s, ["tit", "pose_t", "tool_t", "t"], default=1.20)
        r = self.get_pose_angle_rad(s, ["rol", "pose_r", "tool_r", "r"], default=-1.57)
        g = self.get_pose_angle_rad(s, ["g", "pose_g", "tool_g"], default=3.14)

        return {
            "x": x,
            "y": y,
            "z": z,
            "t": t,
            "r": r,
            "g": g,
        }

    @staticmethod
    def get_pose_angle_rad(state, keys, default):
        for key in keys:
            if key not in state:
                continue
            try:
                value = float(state[key])
            except Exception:
                continue

            # 如果数值明显大于 2π，认为是角度，转成弧度。
            if abs(value) > 6.5:
                return math.radians(value)
            return value

        return float(default)

    def check_target_range(self, x, y, z):
        if not (self.base_x_min <= x <= self.base_x_max):
            return False, f"x out of range: {x:.1f}"
        if not (self.base_y_min <= y <= self.base_y_max):
            return False, f"y out of range: {y:.1f}"
        if not (self.base_z_min <= z <= self.base_z_max):
            return False, f"z out of range: {z:.1f}"
        return True, "ok"

    def build_move_steps(self, target_base):
        pose = self.get_pose_from_arm_state()
        if pose is None:
            raise RuntimeError("No valid arm state.")

        current_x = float(pose["x"])
        current_y = float(pose["y"])
        current_z = float(pose["z"])

        target_x = float(target_base["x"]) + self.grasp_offset_x_mm
        target_y = float(target_base["y"]) + self.grasp_offset_y_mm
        target_z = max(
            float(target_base["z"]) + self.safe_above_offset_mm + self.grasp_offset_z_mm,
            self.min_safe_z_mm,
        )

        ok, reason = self.check_target_range(target_x, target_y, target_z)
        if not ok:
            raise RuntimeError(reason)

        if self.direct_move_to_target:
            steps = [("direct", target_x, target_y, target_z)]
            return pose, {
                "x": target_x,
                "y": target_y,
                "z": target_z,
            }, steps

        stage_z = max(current_z, target_z, self.min_safe_z_mm + 40.0)

        steps = []

        if abs(stage_z - current_z) > 3.0:
            steps.append(("lift", current_x, current_y, stage_z))

        xy_dist = math.hypot(target_x - current_x, target_y - current_y)
        n_xy = max(1, int(math.ceil(xy_dist / max(self.max_step_mm, 1.0))))

        for i in range(1, n_xy + 1):
            ratio = i / n_xy
            x_i = current_x + (target_x - current_x) * ratio
            y_i = current_y + (target_y - current_y) * ratio
            steps.append((f"xy-{i}/{n_xy}", x_i, y_i, stage_z))

        if abs(target_z - stage_z) > 3.0:
            steps.append(("final-z", target_x, target_y, target_z))

        return pose, {
            "x": target_x,
            "y": target_y,
            "z": target_z,
        }, steps

    def publish_move_pose(self, x, y, z, pose, label):
        cmd = {
            "type": "move_pose",
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "t": float(pose["t"]),
            "r": float(pose["r"]),
            "g": float(pose["g"]),
            "speed": self.move_speed,
            "label": label,
        }
        self.publish_cmd(cmd)

    def execute_move_above(self):
        if self.latest_target is None:
            raise RuntimeError("No latest target.")

        target_base = self.latest_target["base_mm"]

        pose, target_eef, steps = self.build_move_steps(target_base)

        self.get_logger().info("====================================")
        self.get_logger().info("Task manager move above target:")
        self.get_logger().info(
            f"target_base: x={target_base['x']:.1f}, y={target_base['y']:.1f}, z={target_base['z']:.1f}"
        )
        self.get_logger().info(
            f"target_eef : x={target_eef['x']:.1f}, y={target_eef['y']:.1f}, z={target_eef['z']:.1f}"
        )
        self.get_logger().info("Planned steps:")

        last_x = float(pose["x"])
        last_y = float(pose["y"])
        last_z = float(pose["z"])

        for label, x_i, y_i, z_i in steps:
            step_delta = math.sqrt(
                (x_i - last_x) ** 2
                + (y_i - last_y) ** 2
                + (z_i - last_z) ** 2
            )
            self.get_logger().info(
                f"  {label}: x={x_i:.1f}, y={y_i:.1f}, z={z_i:.1f}, step_delta={step_delta:.1f} mm"
            )
            last_x, last_y, last_z = x_i, y_i, z_i

        for label, x_i, y_i, z_i in steps:
            self.publish_move_pose(x_i, y_i, z_i, pose, label)
            time.sleep(self.step_wait_s)

        return target_eef

    def target_is_fresh(self):
        if self.latest_target is None:
            return False
        return time.time() - self.latest_target_time <= self.target_timeout_s

    def on_timer(self):
        self.publish_task_state()

        if not self.auto_start:
            return

        if self.busy:
            return

        now = time.time()

        if self.state == "INIT":
            self.get_logger().info("State INIT: send initial pose.")
            self.send_initial_pose(b_offset_deg=0.0)
            self.scan_index = 0
            self.last_scan_switch_time = now
            self.state = "WAIT_TARGET"
            self.started = True
            return

        if self.state == "WAIT_TARGET":
            if self.moved_once and self.move_once_only:
                self.state = "DONE"
                return

            if self.target_is_fresh() and self.stable_count >= self.stable_count_required:
                self.state = "MOVE_ABOVE"
                return

            if self.enable_b_scan and now - self.last_scan_switch_time > self.scan_timeout_s:
                self.scan_index += 1

                if self.scan_index >= len(self.b_scan_offsets):
                    self.get_logger().warn("All b scan views tried. Task failed.")
                    self.state = "FAIL"
                    return

                offset = self.b_scan_offsets[self.scan_index]
                self.get_logger().info(f"No stable target. Scan next b offset: {offset:.1f} deg")
                self.send_initial_pose(b_offset_deg=offset)

                self.prev_target_base = None
                self.stable_count = 0
                self.last_scan_switch_time = now + self.after_scan_pose_wait_s
                return

        if self.state == "MOVE_ABOVE":
            self.busy = True
            try:
                target_eef = self.execute_move_above()
                self.moved_once = True
                self.state = "DONE" if self.move_once_only else "WAIT_TARGET"
                self.publish_task_state(
                    {
                        "move_result": {
                            "ok": True,
                            "target_eef": target_eef,
                        }
                    }
                )
                self.get_logger().info("Move above target finished.")
            except Exception as e:
                self.get_logger().error(f"Move above target failed: {e}")
                self.state = "FAIL"
                self.publish_task_state(
                    {
                        "move_result": {
                            "ok": False,
                            "error": str(e),
                        }
                    }
                )
            finally:
                self.busy = False
            return

        if self.state == "DONE":
            return

        if self.state == "FAIL":
            return


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()

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
