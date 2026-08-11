#!/usr/bin/env python3
import json
import math
import os
import sys
import threading
import time
from collections import deque

import rclpy
import yaml
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String

from roarm_msgs.srv import MoveJointCmd, MoveLineCmd


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def mean(values):
    if not values:
        return 0.0
    return float(sum(values)) / float(len(values))


def stddev(values):
    if len(values) <= 1:
        return 0.0
    avg = mean(values)
    var = sum((float(v) - avg) ** 2 for v in values) / float(len(values))
    return math.sqrt(var)


class VisualMoveItGraspNode(Node):
    def __init__(self, config_path):
        super().__init__("visual_moveit_grasp")
        self.config_path = config_path
        self.cfg = load_yaml(config_path)
        self.callback_group = ReentrantCallbackGroup()
        self.timer_period_s = self.float_cfg("timer_period_s", 0.1)
        self.move_line_timeout_s = self.float_cfg("move_line_timeout_s", 120.0)
        self.move_joint_timeout_s = self.float_cfg("move_joint_timeout_s", 120.0)
        self.service_wait_timeout_s = self.float_cfg("service_wait_timeout_s", 5.0)

        self.target_buffer = deque(maxlen=max(3, int(self.cfg["stable_target_min_frames"])))
        self.latest_target = None
        self.current_snapshot = None
        self.current_grasp_targets = None
        self.state = "IDLE"
        self.state_enter_time = time.time()
        self.state_action_done = False
        self.base_adjust_sent_at = 0.0
        self.done_logged = False
        self.motion_states = {
            "OPEN_GRIPPER",
            "MOVE_TO_PRE_GRASP",
            "MOVE_LINE_TO_GRASP",
            "CLOSE_GRIPPER",
            "LIFT",
        }

        self.sub_target = self.create_subscription(
            String,
            "/red_block/target_base",
            self.on_target_msg,
            10,
            callback_group=self.callback_group,
        )
        self.pub_gripper = self.create_publisher(Float32, "/gripper_cmd", 10)
        self.pub_base_adjust = self.create_publisher(String, self.cfg["base_adjust_topic"], 10)
        self.pub_joint = None
        if str(self.cfg.get("joint_command_mode", "official_joint_topic")).strip() == "official_joint_topic":
            self.pub_joint = self.create_publisher(
                JointState,
                str(self.cfg.get("official_joint_topic", "/joint_states")).strip() or "/joint_states",
                10,
            )
        self.move_line_client = self.create_client(
            MoveLineCmd,
            "/move_line_cmd",
            callback_group=self.callback_group,
        )
        self.move_joint_client = self.create_client(
            MoveJointCmd,
            "/move_joint_cmd",
            callback_group=self.callback_group,
        )
        self.timer = self.create_timer(
            self.timer_period_s,
            self.on_timer,
            callback_group=self.callback_group,
        )

        self.get_logger().info(f"加载抓取配置: {self.config_path}")
        self.get_logger().info("桥接启动: 订阅 /red_block/target_base, 调用 /move_line_cmd, 发布 /gripper_cmd")

    def bool_cfg(self, key):
        value = self.cfg.get(key, False)
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def float_cfg(self, key, default=0.0):
        try:
            return float(self.cfg.get(key, default))
        except Exception:
            return float(default)

    def list_cfg(self, key):
        value = self.cfg.get(key, [])
        if isinstance(value, (list, tuple)):
            return [str(item) for item in value]
        return []

    def set_state(self, new_state, reason=""):
        if self.state != new_state:
            text = f"状态切换: {self.state} -> {new_state}"
            if reason:
                text += f" | {reason}"
            self.get_logger().info(text)

        if new_state == "RECOVER":
            self.clear_snapshot()
        elif new_state == "DONE":
            self.clear_snapshot(reset_done_logged=False)
            self.done_logged = False

        self.state = new_state
        self.state_enter_time = time.time()
        self.state_action_done = False

    def on_target_msg(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"目标 JSON 解析失败: {exc}")
            return

        self.latest_target = data
        if self.state not in ("IDLE", "WAIT_TARGET"):
            return

        target = self.extract_target(data)
        if target is None:
            return

        self.target_buffer.append(target)

    def extract_target(self, data):
        if not isinstance(data, dict):
            return None
        if not bool(data.get("valid", False)):
            return None
        if bool(data.get("is_hold", False)):
            return None

        base = data.get("base_mm")
        pixel = data.get("pixel")
        if not isinstance(base, dict) or not isinstance(pixel, dict):
            return None

        now = time.time()
        stamp = float(data.get("stamp", now))
        age_s = max(0.0, now - stamp)
        if age_s > float(self.cfg["stage_target_max_age_s"]):
            self.get_logger().warn(f"忽略过期目标: age={age_s:.2f}s")
            return None

        try:
            return {
                "x_mm": float(base["x"]),
                "y_mm": float(base["y"]),
                "z_mm": float(base["z"]),
                "pixel_u": int(pixel["x"]),
                "pixel_v": int(pixel["y"]),
                "confidence": float(data.get("confidence", data.get("score", 0.0))),
                "stamp": stamp,
                "source": str(data.get("source", "")),
                "is_hold": bool(data.get("is_hold", False)),
            }
        except Exception as exc:
            self.get_logger().warn(f"目标字段不完整，忽略该帧: {exc}")
            return None

    def workspace_contains(self, x_mm, y_mm, z_mm):
        return (
            float(self.cfg["workspace_x_min_mm"]) <= x_mm <= float(self.cfg["workspace_x_max_mm"])
            and float(self.cfg["workspace_y_min_mm"]) <= y_mm <= float(self.cfg["workspace_y_max_mm"])
            and float(self.cfg["workspace_z_min_mm"]) <= z_mm <= float(self.cfg["workspace_z_max_mm"])
        )

    def suggest_base_adjust(self, snapshot):
        x_mm = snapshot["x_mm"]
        y_mm = snapshot["y_mm"]
        z_mm = snapshot["z_mm"]
        request = {
            "reason": "target_out_of_workspace",
            "target_x_mm": x_mm,
            "target_y_mm": y_mm,
            "target_z_mm": z_mm,
            "workspace_x_min": float(self.cfg["workspace_x_min_mm"]),
            "workspace_x_max": float(self.cfg["workspace_x_max_mm"]),
            "workspace_y_min": float(self.cfg["workspace_y_min_mm"]),
            "workspace_y_max": float(self.cfg["workspace_y_max_mm"]),
            "suggested_action": "stop_and_reobserve",
            "move_distance_mm": 0.0,
            "turn_deg": 0.0,
        }

        if x_mm > float(self.cfg["workspace_x_max_mm"]):
            request["suggested_action"] = "move_forward"
            request["move_distance_mm"] = float(x_mm - float(self.cfg["workspace_x_max_mm"]))
        elif x_mm < float(self.cfg["workspace_x_min_mm"]):
            request["suggested_action"] = "move_backward"
            request["move_distance_mm"] = float(float(self.cfg["workspace_x_min_mm"]) - x_mm)
        elif y_mm > float(self.cfg["workspace_y_max_mm"]):
            request["suggested_action"] = "turn_left"
            request["turn_deg"] = 10.0
        elif y_mm < float(self.cfg["workspace_y_min_mm"]):
            request["suggested_action"] = "turn_right"
            request["turn_deg"] = 10.0
        elif z_mm > float(self.cfg["workspace_z_max_mm"]) or z_mm < float(self.cfg["workspace_z_min_mm"]):
            request["suggested_action"] = "stop_and_reobserve"

        return request

    def publish_base_adjust_request(self, snapshot):
        if not self.bool_cfg("enable_base_adjust_request"):
            return
        if time.time() - self.base_adjust_sent_at < 0.5:
            return

        request = self.suggest_base_adjust(snapshot)
        msg = String()
        msg.data = json.dumps(request, ensure_ascii=False)
        self.pub_base_adjust.publish(msg)
        self.base_adjust_sent_at = time.time()
        self.get_logger().warn(f"目标超出工作空间，发布底盘调整请求: {msg.data}")

    def try_build_snapshot(self):
        required = int(self.cfg["stable_target_min_frames"])
        if len(self.target_buffer) < required:
            return None

        targets = list(self.target_buffer)[-required:]
        xs = [item["x_mm"] for item in targets]
        ys = [item["y_mm"] for item in targets]
        zs = [item["z_mm"] for item in targets]

        max_std = max(stddev(xs), stddev(ys), stddev(zs))
        if max_std > float(self.cfg["stable_target_max_position_std_mm"]):
            return None

        latest = targets[-1]
        snapshot = {
            "x_mm": mean(xs),
            "y_mm": mean(ys),
            "z_mm": mean(zs),
            "pixel_u": int(round(mean([item["pixel_u"] for item in targets]))),
            "pixel_v": int(round(mean([item["pixel_v"] for item in targets]))),
            "confidence": latest["confidence"],
            "stamp": latest["stamp"],
            "source": latest["source"],
            "is_hold": latest["is_hold"],
            "frozen_at": time.time(),
        }

        if not self.workspace_contains(snapshot["x_mm"], snapshot["y_mm"], snapshot["z_mm"]):
            if (
                float(self.cfg["workspace_x_min_mm"]) <= snapshot["x_mm"] <= float(self.cfg["workspace_x_max_mm"])
                and float(self.cfg["workspace_y_min_mm"]) <= snapshot["y_mm"] <= float(self.cfg["workspace_y_max_mm"])
            ):
                self.get_logger().warn("目标 z 超出工作空间，安全拒绝本次抓取。")
            else:
                self.publish_base_adjust_request(snapshot)
            self.target_buffer.clear()
            return None

        self.get_logger().info(
            "target_snapshot created: "
            f"x={snapshot['x_mm']:.1f}mm y={snapshot['y_mm']:.1f}mm z={snapshot['z_mm']:.1f}mm "
            f"pixel=({snapshot['pixel_u']},{snapshot['pixel_v']}) conf={snapshot['confidence']:.3f}"
        )
        return snapshot

    def snapshot_expired(self, log_warning=True):
        if self.current_snapshot is None:
            return True
        age_frozen = time.time() - float(self.current_snapshot["frozen_at"])
        age_target = time.time() - float(self.current_snapshot["stamp"])
        limit = float(self.cfg["stage_target_max_age_s"])
        expired = age_frozen > limit or age_target > limit
        if expired and log_warning:
            self.get_logger().warn(
                f"snapshot 过期: frozen_age={age_frozen:.2f}s target_age={age_target:.2f}s limit={limit:.2f}s"
            )
        return expired

    def allow_expired_snapshot_in_current_state(self):
        if self.state not in self.motion_states:
            return False
        if not self.bool_cfg("allow_snapshot_expire_during_motion"):
            return False
        self.get_logger().warn("snapshot expired by age but allowed during current motion stage")
        return True

    def clear_snapshot(self, reset_done_logged=True):
        self.current_snapshot = None
        self.current_grasp_targets = None
        self.target_buffer.clear()
        if reset_done_logged:
            self.done_logged = False

    def wait_for_future(self, future, timeout_sec):
        done_event = threading.Event()
        future.add_done_callback(lambda _: done_event.set())
        return done_event.wait(timeout_sec)

    @staticmethod
    def deg_to_rad(value_deg):
        return float(value_deg) * math.pi / 180.0

    def build_observe_joint_message(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        joint_names = self.list_cfg("official_joint_names")
        if not joint_names:
            raise ValueError("official_joint_names is empty")
        if len(joint_names) != 6:
            raise ValueError(f"official_joint_names must contain 6 joints, got {len(joint_names)}")
        msg.name = joint_names
        unit_mode = str(self.cfg.get("official_joint_units", "rad")).strip().lower()
        joint_deg_positions = [
            self.float_cfg("observe_b_deg", 0.0),
            self.float_cfg("observe_s_deg", 0.0),
            self.float_cfg("observe_e_deg", 70.0),
            self.float_cfg("observe_t_deg", 90.0),
            self.float_cfg("observe_r_deg", -90.0),
        ]
        if unit_mode == "deg":
            converted = [float(v) for v in joint_deg_positions]
        else:
            converted = [self.deg_to_rad(v) for v in joint_deg_positions]
        converted.append(self.float_cfg("observe_gripper_rad", 1.5))
        msg.position = converted
        if len(msg.name) != len(msg.position):
            raise ValueError(
                f"observe JointState name/position length mismatch: {len(msg.name)} vs {len(msg.position)}"
            )
        return msg

    def move_to_observe_joint_pose(self):
        if str(self.cfg.get("joint_command_mode", "official_joint_topic")).strip() != "official_joint_topic":
            self.get_logger().error("未确认可靠的官方关节控制接口，不能自动回观察关节姿态。")
            return False
        if self.pub_joint is None:
            self.get_logger().error("官方关节控制 topic 未初始化，不能自动回观察关节姿态。")
            return False

        try:
            msg = self.build_observe_joint_message()
        except Exception as exc:
            self.get_logger().error(f"构造观察关节姿态消息失败: {exc}")
            return False

        topic_name = str(self.cfg.get("official_joint_topic", "/joint_states")).strip() or "/joint_states"
        unit_mode = str(self.cfg.get("official_joint_units", "rad")).strip().lower()
        self.get_logger().info(
            "发布观察关节姿态到官方 topic: "
            f"topic={topic_name}, unit={unit_mode}, names={list(msg.name)}, position={list(msg.position)}"
        )
        if self.bool_cfg("dry_run"):
            self.get_logger().info("MOVE_TO_OBSERVE_JOINT_POSE: dry_run=true，仅打印，不发布关节姿态")
            return True

        try:
            publish_duration_s = max(0.1, self.float_cfg("observe_joint_publish_duration_s", 1.0))
            publish_rate_hz = max(1.0, self.float_cfg("observe_joint_publish_rate_hz", 10.0))
            publish_count = max(1, int(round(publish_duration_s * publish_rate_hz)))
            sleep_s = 1.0 / publish_rate_hz
            for _ in range(publish_count):
                msg.header.stamp = self.get_clock().now().to_msg()
                if len(msg.name) != len(msg.position):
                    self.get_logger().error("观察关节姿态消息长度不一致，拒绝发布")
                    return False
                self.pub_joint.publish(msg)
                time.sleep(sleep_s)
            return True
        except Exception as exc:
            self.get_logger().error(f"发布观察关节姿态失败: {exc}")
            return False

    def to_moveit_xyz(self, x_mm, y_mm, z_mm):
        scale = float(self.cfg["moveit_position_scale"])
        x_m = float(self.cfg["moveit_x_sign"]) * float(x_mm) * scale + float(self.cfg["moveit_x_offset_m"])
        y_m = float(self.cfg["moveit_y_sign"]) * float(y_mm) * scale + float(self.cfg["moveit_y_offset_m"])
        z_m = float(self.cfg["moveit_z_sign"]) * float(z_mm) * scale + float(self.cfg["moveit_z_offset_m"])
        return x_m, y_m, z_m

    def build_grasp_targets_from_snapshot(self):
        snapshot = self.current_snapshot
        if snapshot is None:
            return None

        raw_x = float(snapshot["x_mm"])
        raw_y = float(snapshot["y_mm"])
        raw_z = float(snapshot["z_mm"])

        if self.bool_cfg("enable_grasp_coordinate_compensation"):
            bias_x = self.float_cfg("grasp_bias_x_mm", 0.0)
            bias_y = self.float_cfg("grasp_bias_y_mm", 0.0)
            bias_z = self.float_cfg("grasp_bias_z_mm", 0.0)
        else:
            bias_x = 0.0
            bias_y = 0.0
            bias_z = 0.0

        pre_x = raw_x + bias_x if self.bool_cfg("apply_xy_bias_to_pre_grasp") else raw_x
        pre_y = raw_y + bias_y if self.bool_cfg("apply_xy_bias_to_pre_grasp") else raw_y
        pre_z = raw_z + self.float_cfg("pre_grasp_z_offset_mm", 0.0)
        if self.bool_cfg("apply_z_bias_to_pre_grasp"):
            pre_z += bias_z

        grasp_x = raw_x + bias_x
        grasp_y = raw_y + bias_y
        grasp_z = raw_z + self.float_cfg("grasp_offset_z_mm", 0.0) + bias_z

        lift_x = grasp_x
        lift_y = grasp_y
        lift_z = grasp_z + self.float_cfg("lift_up_mm", 0.0)

        if self.bool_cfg("enable_fixed_grasp_rpy"):
            grasp_rpy = {
                "roll": self.float_cfg("fixed_grasp_roll_rad", 0.0),
                "pitch": self.float_cfg("fixed_grasp_pitch_rad", 0.0),
                "yaw": self.float_cfg("fixed_grasp_yaw_rad", 0.0),
            }
        else:
            grasp_rpy = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}

        self.get_logger().info(f"VISUAL_TARGET raw_mm=({raw_x:.1f},{raw_y:.1f},{raw_z:.1f})")
        self.get_logger().info(
            "GRASP_TARGET compensated_mm="
            f"({grasp_x:.1f},{grasp_y:.1f},{grasp_z:.1f}), "
            f"bias=({bias_x:.1f},{bias_y:.1f},{bias_z:.1f}), "
            f"rpy=({grasp_rpy['roll']:.4f},{grasp_rpy['pitch']:.4f},{grasp_rpy['yaw']:.4f})"
        )

        return {
            "raw": {"x_mm": raw_x, "y_mm": raw_y, "z_mm": raw_z},
            "pre_grasp": {"x_mm": pre_x, "y_mm": pre_y, "z_mm": pre_z},
            "grasp": {"x_mm": grasp_x, "y_mm": grasp_y, "z_mm": grasp_z},
            "lift": {"x_mm": lift_x, "y_mm": lift_y, "z_mm": lift_z},
            "bias": {"x_mm": bias_x, "y_mm": bias_y, "z_mm": bias_z},
            "grasp_rpy": grasp_rpy,
        }

    def ensure_grasp_targets(self):
        if self.current_snapshot is None:
            return None
        if self.current_grasp_targets is None:
            self.current_grasp_targets = self.build_grasp_targets_from_snapshot()
        return self.current_grasp_targets

    def settle_after_motion(self, stage_name, duration_s):
        settle_s = max(0.0, float(duration_s))
        if settle_s <= 0.0:
            return
        self.get_logger().info(f"{stage_name}: settle {settle_s:.2f}s")
        time.sleep(settle_s)

    def move_line_mm(self, stage_name, x_mm, y_mm, z_mm):
        if not self.workspace_contains(x_mm, y_mm, z_mm):
            self.get_logger().error(
                f"{stage_name} 超出工作空间，拒绝调用 move_line: "
                f"x={x_mm:.1f} y={y_mm:.1f} z={z_mm:.1f} mm"
            )
            return False

        x_m, y_m, z_m = self.to_moveit_xyz(x_mm, y_mm, z_mm)
        self.get_logger().info(
            f"{stage_name}: mm=({x_mm:.1f},{y_mm:.1f},{z_mm:.1f}) -> "
            f"moveit({self.cfg['moveit_frame_id']}) m=({x_m:.4f},{y_m:.4f},{z_m:.4f})"
        )

        if self.bool_cfg("dry_run"):
            self.get_logger().info(f"{stage_name}: dry_run=true，仅打印，不调用 /move_line_cmd")
            return True

        if not self.move_line_client.wait_for_service(timeout_sec=self.service_wait_timeout_s):
            self.get_logger().error("/move_line_cmd 服务不可用")
            return False

        req = MoveLineCmd.Request()
        req.x = float(x_m)
        req.y = float(y_m)
        req.z = float(z_m)
        future = self.move_line_client.call_async(req)
        if not self.wait_for_future(future, self.move_line_timeout_s):
            self.get_logger().error(f"{stage_name}: /move_line_cmd 调用超时")
            return False

        result = future.result()
        if result is None:
            self.get_logger().error(f"{stage_name}: /move_line_cmd 返回空结果")
            return False

        self.get_logger().info(
            f"{stage_name}: /move_line_cmd success={result.success} message={result.message}"
        )
        return bool(result.success)

    def move_joint_mm(self, stage_name, x_mm, y_mm, z_mm, roll, pitch, yaw):
        if not self.workspace_contains(x_mm, y_mm, z_mm):
            self.get_logger().error(
                f"{stage_name} 超出工作空间，拒绝调用 move_joint: "
                f"x={x_mm:.1f} y={y_mm:.1f} z={z_mm:.1f} mm"
            )
            return False

        x_m, y_m, z_m = self.to_moveit_xyz(x_mm, y_mm, z_mm)
        self.get_logger().info(
            f"{stage_name}: mm=({x_mm:.1f},{y_mm:.1f},{z_mm:.1f}) -> "
            f"moveit({self.cfg['moveit_frame_id']}) m=({x_m:.4f},{y_m:.4f},{z_m:.4f}), "
            f"rpy=({roll:.4f},{pitch:.4f},{yaw:.4f})"
        )

        if self.bool_cfg("dry_run"):
            self.get_logger().info(f"{stage_name}: dry_run=true，仅打印，不调用 /move_joint_cmd")
            return True

        if not self.move_joint_client.wait_for_service(timeout_sec=self.service_wait_timeout_s):
            self.get_logger().error("/move_joint_cmd 服务不可用")
            return False

        req = MoveJointCmd.Request()
        req.x = float(x_m)
        req.y = float(y_m)
        req.z = float(z_m)
        req.roll = float(roll)
        req.pitch = float(pitch)
        req.yaw = float(yaw)
        future = self.move_joint_client.call_async(req)
        if not self.wait_for_future(future, self.move_joint_timeout_s):
            self.get_logger().error(f"{stage_name}: /move_joint_cmd 调用超时")
            return False

        result = future.result()
        if result is None:
            self.get_logger().error(f"{stage_name}: /move_joint_cmd 返回空结果")
            return False

        self.get_logger().info(
            f"{stage_name}: /move_joint_cmd success={result.success} message={result.message}"
        )
        return bool(result.success)

    def publish_gripper(self, value, stage_name):
        self.get_logger().info(f"{stage_name}: gripper_cmd={float(value):.3f}")
        if not self.bool_cfg("enable_gripper"):
            self.get_logger().info(f"{stage_name}: enable_gripper=false，跳过夹爪发布")
            return True
        if self.bool_cfg("dry_run"):
            self.get_logger().info(f"{stage_name}: dry_run=true，仅打印，不发布 /gripper_cmd")
            return True

        msg = Float32()
        msg.data = float(value)
        self.pub_gripper.publish(msg)
        return True

    def on_timer(self):
        now = time.time()

        if self.state == "IDLE":
            if self.bool_cfg("auto_start"):
                if self.bool_cfg("move_to_observe_joint_pose_on_start"):
                    self.set_state("MOVE_TO_OBSERVE_JOINT_POSE", "auto_start=true")
                else:
                    self.set_state("WAIT_TARGET", "auto_start=true")
            return

        if self.state == "DONE":
            if not self.done_logged:
                self.get_logger().info("抓取流程完成。当前第一版仅执行单个红块抓取。")
                self.done_logged = True
            return

        if self.state == "RECOVER":
            if not self.state_action_done:
                self.get_logger().error("进入 RECOVER：本轮抓取终止，等待重新观测目标。")
                self.clear_snapshot(reset_done_logged=False)
                self.state_action_done = True
            return

        if self.state == "MOVE_TO_OBSERVE_JOINT_POSE":
            if not self.state_action_done:
                self.state_action_done = True
                ok = self.move_to_observe_joint_pose()
                if not ok:
                    self.set_state("RECOVER", "观察关节姿态发布失败")
                    return
            if now - self.state_enter_time >= self.float_cfg("observe_wait_s", 3.0):
                self.set_state("WAIT_TARGET", "观察关节姿态等待完成")
            return

        if self.state == "WAIT_TARGET":
            if self.current_snapshot is not None:
                if self.snapshot_expired(log_warning=False):
                    self.clear_snapshot(reset_done_logged=False)
                else:
                    self.set_state("OPEN_GRIPPER", "稳定目标已冻结")
                    return

            snapshot = self.try_build_snapshot()
            if snapshot is not None:
                self.current_snapshot = snapshot
                self.current_grasp_targets = self.build_grasp_targets_from_snapshot()
                self.set_state("OPEN_GRIPPER", "target_snapshot created")
            return

        if self.current_snapshot is None:
            self.set_state("RECOVER", "抓取阶段缺少 snapshot")
            return

        grasp_targets = self.ensure_grasp_targets()
        if grasp_targets is None:
            self.set_state("RECOVER", "grasp targets unavailable")
            return

        if self.state in self.motion_states and self.snapshot_expired():
            if not self.allow_expired_snapshot_in_current_state():
                self.set_state("RECOVER", "snapshot 已过期")
                return

        if self.state == "OPEN_GRIPPER":
            if not self.state_action_done:
                self.publish_gripper(self.cfg["gripper_open_value"], "OPEN_GRIPPER")
                self.state_action_done = True
            if now - self.state_enter_time >= 1.0:
                self.set_state("MOVE_TO_PRE_GRASP")
            return

        if self.state == "MOVE_TO_PRE_GRASP":
            if self.state_action_done:
                return
            self.state_action_done = True
            pre_grasp = grasp_targets["pre_grasp"]
            grasp_rpy = grasp_targets["grasp_rpy"]
            self.get_logger().info(
                "MOVE_TO_PRE_GRASP target: "
                f"mm=({pre_grasp['x_mm']:.1f},{pre_grasp['y_mm']:.1f},{pre_grasp['z_mm']:.1f}) "
                f"rpy=({grasp_rpy['roll']:.4f},{grasp_rpy['pitch']:.4f},{grasp_rpy['yaw']:.4f})"
            )
            if self.bool_cfg("use_move_joint_for_pre_grasp"):
                ok = self.move_joint_mm(
                    "MOVE_TO_PRE_GRASP",
                    pre_grasp["x_mm"],
                    pre_grasp["y_mm"],
                    pre_grasp["z_mm"],
                    grasp_rpy["roll"],
                    grasp_rpy["pitch"],
                    grasp_rpy["yaw"],
                )
            else:
                ok = self.move_line_mm(
                    "MOVE_TO_PRE_GRASP",
                    pre_grasp["x_mm"],
                    pre_grasp["y_mm"],
                    pre_grasp["z_mm"],
                )
            if ok:
                self.settle_after_motion("MOVE_TO_PRE_GRASP", self.float_cfg("pre_grasp_settle_s", 0.5))
            self.set_state("MOVE_LINE_TO_GRASP" if ok else "RECOVER", "预抓取位成功" if ok else "预抓取位失败")
            return

        if self.state == "MOVE_LINE_TO_GRASP":
            if self.state_action_done:
                return
            self.state_action_done = True
            grasp = grasp_targets["grasp"]
            grasp_rpy = grasp_targets["grasp_rpy"]
            self.get_logger().info(
                "MOVE_LINE_TO_GRASP target: "
                f"mm=({grasp['x_mm']:.1f},{grasp['y_mm']:.1f},{grasp['z_mm']:.1f}) "
                f"rpy=({grasp_rpy['roll']:.4f},{grasp_rpy['pitch']:.4f},{grasp_rpy['yaw']:.4f})"
            )
            if self.bool_cfg("use_move_joint_for_grasp"):
                ok = self.move_joint_mm(
                    "MOVE_LINE_TO_GRASP",
                    grasp["x_mm"],
                    grasp["y_mm"],
                    grasp["z_mm"],
                    grasp_rpy["roll"],
                    grasp_rpy["pitch"],
                    grasp_rpy["yaw"],
                )
            else:
                ok = self.move_line_mm(
                    "MOVE_LINE_TO_GRASP",
                    grasp["x_mm"],
                    grasp["y_mm"],
                    grasp["z_mm"],
                )
            if ok:
                self.settle_after_motion("MOVE_LINE_TO_GRASP", self.float_cfg("grasp_settle_s", 1.0))
            self.set_state("CLOSE_GRIPPER" if ok else "RECOVER", "抓取位成功" if ok else "抓取位失败")
            return

        if self.state == "CLOSE_GRIPPER":
            if not self.state_action_done:
                self.publish_gripper(self.cfg["gripper_close_value"], "CLOSE_GRIPPER")
                self.state_action_done = True
            if now - self.state_enter_time >= 1.0:
                self.set_state("LIFT")
            return

        if self.state == "LIFT":
            if self.state_action_done:
                return
            self.state_action_done = True
            lift = grasp_targets["lift"]
            grasp_rpy = grasp_targets["grasp_rpy"]
            self.get_logger().info(
                "LIFT target: "
                f"mm=({lift['x_mm']:.1f},{lift['y_mm']:.1f},{lift['z_mm']:.1f}) "
                f"rpy=({grasp_rpy['roll']:.4f},{grasp_rpy['pitch']:.4f},{grasp_rpy['yaw']:.4f})"
            )
            if self.bool_cfg("use_move_joint_for_lift"):
                ok = self.move_joint_mm(
                    "LIFT",
                    lift["x_mm"],
                    lift["y_mm"],
                    lift["z_mm"],
                    grasp_rpy["roll"],
                    grasp_rpy["pitch"],
                    grasp_rpy["yaw"],
                )
            else:
                ok = self.move_line_mm("LIFT", lift["x_mm"], lift["y_mm"], lift["z_mm"])
            if ok:
                self.settle_after_motion("LIFT", self.float_cfg("lift_settle_s", 0.5))
            self.set_state("DONE" if ok else "RECOVER", "抬升成功" if ok else "抬升失败")
            return


def resolve_config_path():
    if len(sys.argv) >= 2 and sys.argv[1].strip():
        return os.path.abspath(sys.argv[1].strip())
    env_path = os.environ.get("DOG_VISUAL_GRASP_CONFIG", "").strip()
    if env_path:
        return os.path.abspath(env_path)
    base_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(base_dir, "config", "grasp_config.yaml")


def main(args=None):
    config_path = resolve_config_path()
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"配置文件不存在: {config_path}")

    rclpy.init(args=args)
    node = VisualMoveItGraspNode(config_path)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
