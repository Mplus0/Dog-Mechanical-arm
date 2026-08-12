#!/usr/bin/env python3
import json
import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class OpenLoopGraspTaskNode(Node):
    STATES = (
        "IDLE",
        "WAIT_PRE_TARGET",
        "MOVE_PRE_GRASP_FAST",
        "WAIT_GRASP_TARGET",
        "MOVE_GRASP_DIRECT",
        "CLOSE_GRIPPER",
        "MOVE_LIFT",
        "MOVE_RETREAT",
        "DONE",
        "RECOVER",
    )

    def __init__(self):
        super().__init__("open_loop_grasp_task_node")

        self.declare_parameter("auto_start", True)
        self.declare_parameter("loop_hz", 5.0)
        self.declare_parameter("target_timeout_s", 0.8)
        self.declare_parameter("stable_frame_count", 3)
        self.declare_parameter("stable_position_threshold_mm", 20.0)
        self.declare_parameter("stable_depth_threshold_mm", 30.0)

        self.declare_parameter("pre_grasp_offset_x_mm", 0.0)
        self.declare_parameter("pre_grasp_offset_y_mm", 0.0)
        self.declare_parameter("pre_grasp_z_offset_mm", 120.0)
        self.declare_parameter("grasp_offset_x_mm", 0.0)
        self.declare_parameter("grasp_offset_y_mm", 0.0)
        self.declare_parameter("grasp_offset_z_mm", 0.0)
        self.declare_parameter("lift_up_mm", 80.0)

        self.declare_parameter("fast_move_speed", 0.18)
        self.declare_parameter("grasp_move_speed", 0.08)
        self.declare_parameter("lift_move_speed", 0.10)
        self.declare_parameter("retreat_move_speed", 0.15)
        self.declare_parameter("fast_max_step_mm", 120.0)
        self.declare_parameter("max_pre_grasp_segments", 3)

        self.declare_parameter("motion_timeout_s", 8.0)
        self.declare_parameter("motion_wait_s", 1.5)
        self.declare_parameter("motion_min_wait_s", 0.8)
        self.declare_parameter("position_reached_tolerance_mm", 25.0)
        self.declare_parameter("settle_time_s", 0.3)
        self.declare_parameter("pose_t_rad", "none")
        self.declare_parameter("pose_r_rad", "none")
        self.declare_parameter("pose_g_rad", "none")

        self.declare_parameter("max_retarget_drift_mm", 60.0)
        self.declare_parameter("final_correction_enabled", True)
        self.declare_parameter("max_final_corrections", 2)
        self.declare_parameter("final_correction_step_mm", 20.0)

        self.declare_parameter("gripper_close_deg", 55.0)
        self.declare_parameter("gripper_open_deg", 110.0)
        self.declare_parameter("gripper_speed_deg_s", 25.0)
        self.declare_parameter("gripper_acc", 25.0)
        self.declare_parameter("gripper_wait_s", 1.0)

        self.declare_parameter("safe_x_mm", 260.0)
        self.declare_parameter("safe_y_mm", 0.0)
        self.declare_parameter("safe_z_mm", 180.0)
        self.declare_parameter("workspace_x_min", 80.0)
        self.declare_parameter("workspace_x_max", 700.0)
        self.declare_parameter("workspace_y_min", -450.0)
        self.declare_parameter("workspace_y_max", 450.0)
        self.declare_parameter("workspace_z_min", 20.0)
        self.declare_parameter("workspace_z_max", 380.0)

        self.auto_start = self.parse_bool(self.get_parameter("auto_start").value)
        self.loop_hz = float(self.get_parameter("loop_hz").value)
        self.target_timeout_s = float(self.get_parameter("target_timeout_s").value)
        self.stable_frame_count = int(self.get_parameter("stable_frame_count").value)
        self.stable_position_threshold_mm = float(self.get_parameter("stable_position_threshold_mm").value)
        self.stable_depth_threshold_mm = float(self.get_parameter("stable_depth_threshold_mm").value)

        self.pre_grasp_offset_x_mm = float(self.get_parameter("pre_grasp_offset_x_mm").value)
        self.pre_grasp_offset_y_mm = float(self.get_parameter("pre_grasp_offset_y_mm").value)
        self.pre_grasp_z_offset_mm = float(self.get_parameter("pre_grasp_z_offset_mm").value)
        self.grasp_offset_x_mm = float(self.get_parameter("grasp_offset_x_mm").value)
        self.grasp_offset_y_mm = float(self.get_parameter("grasp_offset_y_mm").value)
        self.grasp_offset_z_mm = float(self.get_parameter("grasp_offset_z_mm").value)
        self.lift_up_mm = float(self.get_parameter("lift_up_mm").value)

        self.fast_move_speed = float(self.get_parameter("fast_move_speed").value)
        self.grasp_move_speed = float(self.get_parameter("grasp_move_speed").value)
        self.lift_move_speed = float(self.get_parameter("lift_move_speed").value)
        self.retreat_move_speed = float(self.get_parameter("retreat_move_speed").value)
        self.fast_max_step_mm = float(self.get_parameter("fast_max_step_mm").value)
        self.max_pre_grasp_segments = int(self.get_parameter("max_pre_grasp_segments").value)

        self.motion_timeout_s = float(self.get_parameter("motion_timeout_s").value)
        self.motion_wait_s = float(self.get_parameter("motion_wait_s").value)
        self.motion_min_wait_s = float(self.get_parameter("motion_min_wait_s").value)
        self.position_reached_tolerance_mm = float(self.get_parameter("position_reached_tolerance_mm").value)
        self.settle_time_s = float(self.get_parameter("settle_time_s").value)
        self.pose_t_rad = self.optional_float(self.get_parameter("pose_t_rad").value)
        self.pose_r_rad = self.optional_float(self.get_parameter("pose_r_rad").value)
        self.pose_g_rad = self.optional_float(self.get_parameter("pose_g_rad").value)

        self.max_retarget_drift_mm = float(self.get_parameter("max_retarget_drift_mm").value)
        self.final_correction_enabled = self.parse_bool(self.get_parameter("final_correction_enabled").value)
        self.max_final_corrections = int(self.get_parameter("max_final_corrections").value)
        self.final_correction_step_mm = float(self.get_parameter("final_correction_step_mm").value)

        self.gripper_close_deg = float(self.get_parameter("gripper_close_deg").value)
        self.gripper_open_deg = float(self.get_parameter("gripper_open_deg").value)
        self.gripper_speed_deg_s = float(self.get_parameter("gripper_speed_deg_s").value)
        self.gripper_acc = float(self.get_parameter("gripper_acc").value)
        self.gripper_wait_s = float(self.get_parameter("gripper_wait_s").value)

        self.safe_pose = {
            "x": float(self.get_parameter("safe_x_mm").value),
            "y": float(self.get_parameter("safe_y_mm").value),
            "z": float(self.get_parameter("safe_z_mm").value),
        }
        self.workspace = {
            "x_min": float(self.get_parameter("workspace_x_min").value),
            "x_max": float(self.get_parameter("workspace_x_max").value),
            "y_min": float(self.get_parameter("workspace_y_min").value),
            "y_max": float(self.get_parameter("workspace_y_max").value),
            "z_min": float(self.get_parameter("workspace_z_min").value),
            "z_max": float(self.get_parameter("workspace_z_max").value),
        }

        self.pub_cmd = self.create_publisher(String, "/roarm_m3/cmd", 10)
        self.pub_state = self.create_publisher(String, "/red_block/open_loop_grasp_state", 10)
        self.sub_arm_state = self.create_subscription(String, "/roarm_m3/state", self.on_arm_state, 10)
        self.sub_target = self.create_subscription(String, "/red_block/target_base", self.on_target, 10)

        self.latest_arm_state = None
        self.latest_arm_state_time = 0.0
        self.latest_target = None
        self.latest_target_time = 0.0
        self.target_window = deque(maxlen=max(1, self.stable_frame_count))

        self.state = "IDLE"
        self.state_enter_time = time.time()
        self.last_command_time = 0.0
        self.state_command_sent = False
        self.pre_target = None
        self.grasp_target = None
        self.current_goal = None
        self.pre_grasp_segments = []
        self.pre_grasp_segment_index = 0
        self.final_corrections_done = 0
        self.done = False
        self.error_reason = ""
        self.recover_step = 0

        self.timer = self.create_timer(1.0 / max(self.loop_hz, 0.1), self.on_timer)
        self.get_logger().info("Open-loop grasp task node started.")

    @staticmethod
    def parse_bool(value):
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    @staticmethod
    def optional_float(value):
        if value is None:
            return None
        text = str(value).strip().lower()
        if text in ("", "none", "null"):
            return None
        return float(value)

    @staticmethod
    def dist3(a, b):
        return math.sqrt(
            (float(a["x"]) - float(b["x"])) ** 2
            + (float(a["y"]) - float(b["y"])) ** 2
            + (float(a["z"]) - float(b["z"])) ** 2
        )

    def enter_state(self, state, reason=""):
        if state not in self.STATES:
            raise ValueError(f"unknown state: {state}")
        self.state = state
        self.state_enter_time = time.time()
        self.state_command_sent = False
        if reason:
            self.get_logger().info(f"STATE -> {state}: {reason}")
        else:
            self.get_logger().info(f"STATE -> {state}")

    def publish_cmd(self, cmd):
        msg = String()
        msg.data = json.dumps(cmd, ensure_ascii=False)
        self.pub_cmd.publish(msg)
        self.get_logger().info("PUB /roarm_m3/cmd: " + msg.data)
        self.last_command_time = time.time()
        self.state_command_sent = True

    def publish_state(self, extra=None):
        now = time.time()
        data = {
            "stamp": now,
            "state": self.state,
            "done": self.done,
            "error_reason": self.error_reason,
            "state_command_sent": self.state_command_sent,
            "arm_state_age_s": None,
            "target_age_s": None,
            "pre_target": self.pre_target,
            "grasp_target": self.grasp_target,
            "current_goal": self.current_goal,
            "final_corrections_done": self.final_corrections_done,
        }
        if self.latest_arm_state is not None:
            data["arm_state_age_s"] = now - self.latest_arm_state_time
        if self.latest_target is not None:
            data["target_age_s"] = now - self.latest_target_time
        if extra:
            data.update(extra)
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        self.pub_state.publish(msg)

    def on_arm_state(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if not data.get("connected", False) or not data.get("state_valid", False):
            return
        state = data.get("state")
        if isinstance(state, dict):
            self.latest_arm_state = state
            self.latest_arm_state_time = time.time()

    def on_target(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        if self.state not in ("WAIT_PRE_TARGET", "WAIT_GRASP_TARGET", "MOVE_GRASP_DIRECT"):
            return
        if not data.get("valid", False) or not isinstance(data.get("base_mm"), dict):
            self.target_window.clear()
            self.latest_target = None
            return

        self.latest_target = data
        self.latest_target_time = time.time()
        if data.get("stable", False):
            self.target_window.clear()
            self.target_window.append(data)
        else:
            self.target_window.append(data)

    def arm_state_is_fresh(self):
        return self.latest_arm_state is not None and time.time() - self.latest_arm_state_time <= 1.0

    def target_is_fresh(self):
        return self.latest_target is not None and time.time() - self.latest_target_time <= self.target_timeout_s

    def get_pose_from_arm_state(self):
        if self.latest_arm_state is None:
            return None
        s = self.latest_arm_state
        t = self.get_pose_angle_rad(s, ["tit", "pose_t", "tool_t", "t"], default=1.20)
        r = self.get_pose_angle_rad(s, ["rol", "pose_r", "tool_r", "r"], default=-1.57)
        g = self.get_pose_angle_rad(s, ["g", "pose_g", "tool_g"], default=3.14)
        return {
            "x": float(s.get("x", 0.0)),
            "y": float(s.get("y", 0.0)),
            "z": float(s.get("z", 0.0)),
            "t": self.pose_t_rad if self.pose_t_rad is not None else t,
            "r": self.pose_r_rad if self.pose_r_rad is not None else r,
            "g": self.pose_g_rad if self.pose_g_rad is not None else g,
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
            if abs(value) > 6.5:
                return math.radians(value)
            return value
        return float(default)

    def stable_target_from_window(self):
        if not self.target_is_fresh():
            self.target_window.clear()
            return None, "target_timeout"
        if self.latest_target and self.latest_target.get("stable", False):
            return self.latest_target, "publisher_stable"
        if len(self.target_window) < self.stable_frame_count:
            return None, "not_enough_frames"

        points = []
        depths = []
        for item in self.target_window:
            base = item.get("base_mm", {})
            try:
                points.append([float(base["x"]), float(base["y"]), float(base["z"])])
                depths.append(float(item.get("depth_mm", base.get("z", 0.0))))
            except Exception:
                return None, "bad_target_window"

        arr = np.array(points, dtype=np.float64)
        depth_arr = np.array(depths, dtype=np.float64)
        median = np.median(arr, axis=0)
        max_pos = float(np.max(np.linalg.norm(arr - median, axis=1)))
        max_depth = float(np.max(np.abs(depth_arr - np.median(depth_arr))))
        if max_pos > self.stable_position_threshold_mm or max_depth > self.stable_depth_threshold_mm:
            return None, "window_unstable"

        stable = dict(self.target_window[-1])
        stable["base_mm"] = {"x": float(median[0]), "y": float(median[1]), "z": float(median[2])}
        stable["stable"] = True
        stable["stable_frames"] = len(self.target_window)
        return stable, "local_window_stable"

    def build_pre_grasp(self, target):
        base = target["base_mm"]
        return {
            "x": float(base["x"]) + self.pre_grasp_offset_x_mm,
            "y": float(base["y"]) + self.pre_grasp_offset_y_mm,
            "z": float(base["z"]) + self.pre_grasp_z_offset_mm,
        }

    def build_grasp(self, target):
        base = target["base_mm"]
        return {
            "x": float(base["x"]) + self.grasp_offset_x_mm,
            "y": float(base["y"]) + self.grasp_offset_y_mm,
            "z": float(base["z"]) + self.grasp_offset_z_mm,
        }

    def clamp_workspace(self, pose):
        return {
            "x": max(self.workspace["x_min"], min(self.workspace["x_max"], float(pose["x"]))),
            "y": max(self.workspace["y_min"], min(self.workspace["y_max"], float(pose["y"]))),
            "z": max(self.workspace["z_min"], min(self.workspace["z_max"], float(pose["z"]))),
        }

    def pose_cmd(self, target, speed, label):
        pose = self.get_pose_from_arm_state()
        if pose is None:
            return None
        target = self.clamp_workspace(target)
        self.current_goal = dict(target)
        return {
            "type": "move_pose",
            "x": float(target["x"]),
            "y": float(target["y"]),
            "z": float(target["z"]),
            "t": float(pose["t"]),
            "r": float(pose["r"]),
            "g": float(pose["g"]),
            "speed": float(speed),
            "label": label,
        }

    def build_segments(self, start_pose, goal_pose):
        dist = self.dist3(start_pose, goal_pose)
        if dist <= self.fast_max_step_mm:
            return [goal_pose]
        count = min(self.max_pre_grasp_segments, max(2, int(math.ceil(dist / max(self.fast_max_step_mm, 1.0)))))
        segments = []
        for i in range(1, count + 1):
            ratio = float(i) / float(count)
            segments.append(
                {
                    "x": start_pose["x"] + (goal_pose["x"] - start_pose["x"]) * ratio,
                    "y": start_pose["y"] + (goal_pose["y"] - start_pose["y"]) * ratio,
                    "z": start_pose["z"] + (goal_pose["z"] - start_pose["z"]) * ratio,
                }
            )
        return segments

    def motion_min_wait_done(self):
        return time.time() - self.last_command_time >= self.motion_min_wait_s

    def motion_wait_done(self):
        return self.motion_min_wait_done()

    def motion_reached(self):
        if not isinstance(self.current_goal, dict):
            return False
        if any(key not in self.current_goal for key in ("x", "y", "z")):
            return False
        if not self.arm_state_is_fresh():
            return False

        pose = self.get_pose_from_arm_state()
        if pose is None:
            return False
        distance = self.dist3(pose, self.current_goal)
        return distance <= self.position_reached_tolerance_mm

    def motion_timed_out(self):
        return time.time() - self.last_command_time >= self.motion_timeout_s

    def pose_motion_done(self):
        return self.motion_min_wait_done() and self.motion_reached()

    def send_gripper(self, angle, label):
        self.current_goal = {"joint": 6, "angle": float(angle)}
        self.publish_cmd(
            {
                "type": "move_joint",
                "joint": 6,
                "angle": float(angle),
                "speed": self.gripper_speed_deg_s,
                "acc": self.gripper_acc,
                "label": label,
            }
        )

    def fail(self, reason):
        self.error_reason = reason
        self.recover_step = 0
        self.get_logger().error(reason)
        self.enter_state("RECOVER", reason)

    def on_timer(self):
        self.publish_state()
        now = time.time()
        if not self.auto_start:
            return

        if self.state == "IDLE":
            self.done = False
            self.error_reason = ""
            self.pre_target = None
            self.grasp_target = None
            self.current_goal = None
            self.target_window.clear()
            self.enter_state("WAIT_PRE_TARGET")
            return

        if self.state == "WAIT_PRE_TARGET":
            target, reason = self.stable_target_from_window()
            if target is None:
                self.publish_state({"wait_reason": reason})
                return
            self.pre_target = target
            self.target_window.clear()
            self.pre_grasp_segments = []
            self.pre_grasp_segment_index = 0
            self.enter_state("MOVE_PRE_GRASP_FAST", "pre target locked")
            return

        if self.state == "MOVE_PRE_GRASP_FAST":
            if not self.arm_state_is_fresh():
                return
            if not self.pre_grasp_segments:
                pose = self.get_pose_from_arm_state()
                goal = self.build_pre_grasp(self.pre_target)
                self.pre_grasp_segments = self.build_segments(pose, self.clamp_workspace(goal))
                self.pre_grasp_segment_index = 0
            if self.pre_grasp_segment_index < len(self.pre_grasp_segments) and not self.state_command_sent:
                cmd = self.pose_cmd(
                    self.pre_grasp_segments[self.pre_grasp_segment_index],
                    self.fast_move_speed,
                    f"pre_grasp_{self.pre_grasp_segment_index + 1}",
                )
                if cmd is None:
                    return
                self.publish_cmd(cmd)
                return
            if self.state_command_sent and self.pose_motion_done():
                self.pre_grasp_segment_index += 1
                self.state_command_sent = False
                if self.pre_grasp_segment_index >= len(self.pre_grasp_segments):
                    self.target_window.clear()
                    self.latest_target = None
                    self.enter_state("WAIT_GRASP_TARGET", "arrived pre-grasp")
            elif self.state_command_sent and self.motion_timed_out():
                self.fail("pre_grasp_motion_timeout")
            return

        if self.state == "WAIT_GRASP_TARGET":
            if now - self.state_enter_time < self.settle_time_s:
                return
            target, reason = self.stable_target_from_window()
            if target is None:
                self.publish_state({"wait_reason": reason})
                return
            drift = self.dist3(target["base_mm"], self.pre_target["base_mm"])
            if drift > self.max_retarget_drift_mm:
                self.get_logger().warn(f"Retarget drift {drift:.1f} mm is too large. Restart pre-target.")
                self.target_window.clear()
                self.enter_state("WAIT_PRE_TARGET", "retarget drift")
                return
            self.grasp_target = target
            self.final_corrections_done = 0
            self.enter_state("MOVE_GRASP_DIRECT", "grasp target locked")
            return

        if self.state == "MOVE_GRASP_DIRECT":
            if not self.arm_state_is_fresh():
                return
            if self.final_correction_enabled and self.final_corrections_done < self.max_final_corrections:
                target, _ = self.stable_target_from_window()
                if target is not None:
                    dx = float(target["base_mm"]["x"]) - float(self.grasp_target["base_mm"]["x"])
                    dy = float(target["base_mm"]["y"]) - float(self.grasp_target["base_mm"]["y"])
                    dz = float(target["base_mm"]["z"]) - float(self.grasp_target["base_mm"]["z"])
                    step = math.sqrt(dx * dx + dy * dy + dz * dz)
                    if step > 1e-3:
                        scale = min(1.0, self.final_correction_step_mm / step)
                        corrected = dict(self.grasp_target)
                        corrected["base_mm"] = {
                            "x": float(self.grasp_target["base_mm"]["x"]) + dx * scale,
                            "y": float(self.grasp_target["base_mm"]["y"]) + dy * scale,
                            "z": float(self.grasp_target["base_mm"]["z"]) + dz * scale,
                        }
                        self.grasp_target = corrected
                    self.final_corrections_done += 1
            if not self.state_command_sent:
                cmd = self.pose_cmd(self.build_grasp(self.grasp_target), self.grasp_move_speed, "grasp_direct")
                if cmd is None:
                    return
                self.publish_cmd(cmd)
                return
            if self.pose_motion_done():
                self.enter_state("CLOSE_GRIPPER")
            elif self.motion_timed_out():
                self.fail("grasp_motion_timeout")
            return

        if self.state == "CLOSE_GRIPPER":
            if not self.state_command_sent:
                self.send_gripper(self.gripper_close_deg, "close-gripper")
                return
            if now - self.last_command_time >= self.gripper_wait_s:
                self.enter_state("MOVE_LIFT")
            return

        if self.state == "MOVE_LIFT":
            if not self.arm_state_is_fresh():
                return
            if not self.state_command_sent:
                pose = self.get_pose_from_arm_state()
                target = {"x": pose["x"], "y": pose["y"], "z": pose["z"] + self.lift_up_mm}
                cmd = self.pose_cmd(target, self.lift_move_speed, "lift_after_grasp")
                if cmd is None:
                    return
                self.publish_cmd(cmd)
                return
            if self.pose_motion_done():
                self.enter_state("MOVE_RETREAT")
            elif self.motion_timed_out():
                self.fail("lift_motion_timeout")
            return

        if self.state == "MOVE_RETREAT":
            if not self.arm_state_is_fresh():
                return
            if not self.state_command_sent:
                cmd = self.pose_cmd(self.safe_pose, self.retreat_move_speed, "retreat_safe")
                if cmd is None:
                    return
                self.publish_cmd(cmd)
                return
            if self.pose_motion_done():
                self.done = True
                self.enter_state("DONE")
            elif self.motion_timed_out():
                self.fail("retreat_motion_timeout")
            return

        if self.state == "DONE":
            if now - self.state_enter_time >= 1.0:
                self.enter_state("IDLE")
            return

        if self.state == "RECOVER":
            if not self.arm_state_is_fresh():
                return
            if self.recover_step == 0 and not self.state_command_sent:
                self.send_gripper(self.gripper_open_deg, "recover-open-gripper")
                return
            if self.recover_step == 0 and now - self.last_command_time >= self.gripper_wait_s:
                self.recover_step = 1
                self.state_command_sent = False
                return
            if self.recover_step == 1 and not self.state_command_sent:
                pose = self.get_pose_from_arm_state()
                lift = {"x": pose["x"], "y": pose["y"], "z": max(pose["z"] + self.lift_up_mm, self.safe_pose["z"])}
                cmd = self.pose_cmd(lift, self.lift_move_speed, "recover-lift")
                if cmd is not None:
                    self.publish_cmd(cmd)
                return
            if self.recover_step == 1 and self.pose_motion_done():
                self.recover_step = 2
                self.state_command_sent = False
                return
            if self.recover_step == 1 and self.motion_timed_out():
                self.target_window.clear()
                self.latest_target = None
                self.enter_state("IDLE", "recover lift timed out")
                return
            if self.recover_step == 2 and not self.state_command_sent:
                cmd = self.pose_cmd(self.safe_pose, self.retreat_move_speed, "recover-retreat-safe")
                if cmd is not None:
                    self.publish_cmd(cmd)
                return
            if self.recover_step == 2 and self.pose_motion_done():
                self.target_window.clear()
                self.latest_target = None
                self.enter_state("IDLE")
                return
            if self.recover_step == 2 and self.motion_timed_out():
                self.target_window.clear()
                self.latest_target = None
                self.enter_state("IDLE", "recover retreat timed out")


def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopGraspTaskNode()
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
