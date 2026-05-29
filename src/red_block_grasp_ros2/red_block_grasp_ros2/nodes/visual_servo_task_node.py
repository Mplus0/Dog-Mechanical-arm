#!/usr/bin/env python3
# 中文说明：
# 视觉闭环任务节点。
#
# 本节点不直接连接相机，也不直接连接机械臂串口。
#
# 订阅：
#   /roarm_m3/state
#   /red_block/target_base
#
# 发布：
#   /roarm_m3/cmd
#   /red_block/visual_servo_state
#
# 核心思想：
#   不再“一次识别后直接运动到目标点”。
#   而是：
#       看一帧目标
#       只移动一小步
#       停一下
#       重新识别
#       再移动一小步
#
# 这样适合眼在手上的相机结构，可以减少机械臂运动时目标突然跑出视野的问题。

import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VisualServoTaskNode(Node):
    def __init__(self):
        super().__init__("visual_servo_task_node")

        # ===== 基础控制 =====
        self.declare_parameter("auto_start", True)
        self.declare_parameter("move_once_only", True)
        self.declare_parameter("competition_mode", False)
        self.declare_parameter("visual_servo_cmd_topic", "/red_block/visual_servo_cmd")

        # ===== 初始姿态 =====
        self.declare_parameter("init_b_deg", 0.0)
        self.declare_parameter("init_s_deg", 0.0)
        self.declare_parameter("init_e_deg", 70.0)
        self.declare_parameter("init_t_deg", 90.0)
        self.declare_parameter("init_r_deg", -90.0)
        self.declare_parameter("init_speed_deg_s", 35.0)
        self.declare_parameter("init_acc", 35.0)
        self.declare_parameter("initial_wait_s", 3.0)

        # ===== 找不到目标时的 b 角搜索 =====
        self.declare_parameter("enable_b_scan", True)
        self.declare_parameter("b_scan_offsets", "0,-8,8,-16,16,-24,24")
        self.declare_parameter("scan_timeout_s", 8.0)
        self.declare_parameter("after_scan_pose_wait_s", 2.5)

        # ===== 视觉闭环移动参数 =====
        self.declare_parameter("target_timeout_s", 1.0)
        self.declare_parameter("max_step_mm", 25.0)
        self.declare_parameter("edge_step_mm", 12.0)
        self.declare_parameter("move_speed", 0.10)
        self.declare_parameter("step_wait_s", 2.0)
        self.declare_parameter("adaptive_step_enabled", True)
        self.declare_parameter("adaptive_step_min_mm", 5.0)
        self.declare_parameter("adaptive_step_max_mm", 25.0)
        self.declare_parameter("adaptive_edge_step_min_mm", 5.0)
        self.declare_parameter("adaptive_center_good_ratio", 0.25)
        self.declare_parameter("adaptive_center_bad_ratio", 0.55)

        # ===== E 关节视野保持 =====
        # 如果红块在图像上下方向偏离目标像素点，就优先微调 e 关节，避免目标跑出视野。
        self.declare_parameter("enable_e_pixel_servo", True)
        self.declare_parameter("desired_pixel_v", 240.0)
        self.declare_parameter("pixel_v_deadband", 45.0)
        self.declare_parameter("e_kp_deg_per_px", 0.025)
        self.declare_parameter("e_max_step_deg", 3.0)
        self.declare_parameter("e_min_deg", 30.0)
        self.declare_parameter("e_max_deg", 120.0)
        self.declare_parameter("e_pixel_sign", 1.0)
        self.declare_parameter("e_servo_speed_deg_s", 18.0)
        self.declare_parameter("e_servo_acc", 18.0)

        # ===== 到达判定 =====
        self.declare_parameter("target_xy_tolerance_mm", 25.0)
        self.declare_parameter("target_z_tolerance_mm", 25.0)

        # ===== 目标上方点 =====
        self.declare_parameter("safe_above_offset_mm", 120.0)
        self.declare_parameter("grasp_offset_x_mm", 0.0)
        self.declare_parameter("grasp_offset_y_mm", 0.0)
        self.declare_parameter("grasp_offset_z_mm", 0.0)
        self.declare_parameter("min_safe_z_mm", 30.0)
        self.declare_parameter("servo_min_z_mm", 80.0)
        self.declare_parameter("servo_recover_z_margin_mm", 30.0)
        self.declare_parameter("servo_recover_max_attempts", 3)

        # ===== Center-first visual servo =====
        self.declare_parameter("enable_center_first_servo", True)
        self.declare_parameter("center_pixel_u", 320.0)
        self.declare_parameter("center_pixel_v", 240.0)
        self.declare_parameter("center_pixel_deadband_u", 40.0)
        self.declare_parameter("center_pixel_deadband_v", 40.0)
        self.declare_parameter("center_stable_frames_required", 3)
        self.declare_parameter("center_b_kp_deg_per_px", 0.018)
        self.declare_parameter("center_b_max_step_deg", 2.0)
        self.declare_parameter("center_e_kp_deg_per_px", 0.020)
        self.declare_parameter("center_e_max_step_deg", 2.0)
        self.declare_parameter("center_b_pixel_sign", 1.0)
        self.declare_parameter("center_e_pixel_sign", -1.0)
        self.declare_parameter("center_step_wait_s", 1.0)
        self.declare_parameter("approach_hold_z_mm", 140.0)

        # ===== 图像安全区域 =====
        # 如果红块中心太靠边，只允许更小步运动，防止冲出视野。
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("safe_roi_x_min_ratio", 0.20)
        self.declare_parameter("safe_roi_x_max_ratio", 0.80)
        self.declare_parameter("safe_roi_y_min_ratio", 0.20)
        self.declare_parameter("safe_roi_y_max_ratio", 0.80)

        # ===== 工作空间保护 =====
        self.declare_parameter("base_x_min", 80.0)
        self.declare_parameter("base_x_max", 700.0)
        self.declare_parameter("base_y_min", -450.0)
        self.declare_parameter("base_y_max", 450.0)
        self.declare_parameter("base_z_min", -30.0)
        self.declare_parameter("base_z_max", 380.0)

        # ===== 下降测试参数 =====
        self.declare_parameter("descend_test_mm", 30.0)
        self.declare_parameter("descend_step_mm", 5.0)
        self.declare_parameter("descend_control_mode", "pixel")
        self.declare_parameter("descend_lock_xy", True)
        self.declare_parameter("descend_xy_step_mm", 0.0)
        self.declare_parameter("descend_x_comp_mm_per_mm", 0.0)
        self.declare_parameter("descend_y_comp_mm_per_mm", 0.0)
        self.declare_parameter("descend_desired_pixel_u", 320.0)
        self.declare_parameter("descend_desired_pixel_v", 240.0)
        self.declare_parameter("descend_pixel_deadband", 35.0)
        self.declare_parameter("descend_pixel_kp_mm_per_px", 0.04)
        self.declare_parameter("descend_pixel_max_xy_step_mm", 6.0)
        self.declare_parameter("descend_pixel_x_sign", 1.0)
        self.declare_parameter("descend_pixel_y_sign", 1.0)
        self.declare_parameter("descend_min_z_mm", 30.0)
        self.declare_parameter("descend_min_confidence", 0.60)
        self.declare_parameter("descend_speed", 0.06)
        self.declare_parameter("descend_wait_s", 2.0)
        self.declare_parameter("descend_step_wait_s", 1.0)

        # ===== 抓取、抬升、放置参数 =====
        self.declare_parameter("enable_pick_place_sequence", False)
        self.declare_parameter("gripper_close_deg", 55.0)
        self.declare_parameter("gripper_open_deg", 110.0)
        self.declare_parameter("gripper_speed_deg_s", 25.0)
        self.declare_parameter("gripper_acc", 25.0)
        self.declare_parameter("gripper_wait_s", 1.0)
        self.declare_parameter("lift_up_mm", 80.0)
        self.declare_parameter("lift_speed", 0.08)
        self.declare_parameter("lift_wait_s", 2.0)
        self.declare_parameter("place_x_mm", 260.0)
        self.declare_parameter("place_y_mm", 180.0)
        self.declare_parameter("place_z_mm", 120.0)
        self.declare_parameter("place_speed", 0.10)
        self.declare_parameter("place_wait_s", 2.0)
        self.declare_parameter("hold_after_pick_s", 3.2)
        self.declare_parameter("place_wait_after_open_s", 0.8)
        self.declare_parameter("base_adjust_step_m", 0.05)
        self.declare_parameter("base_adjust_y_margin_mm", 40.0)

        self.auto_start = self.parse_bool(self.get_parameter("auto_start").value)
        self.move_once_only = self.parse_bool(self.get_parameter("move_once_only").value)
        self.competition_mode = self.parse_bool(self.get_parameter("competition_mode").value)
        self.visual_servo_cmd_topic = str(self.get_parameter("visual_servo_cmd_topic").value)

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
        self.initial_wait_s = float(self.get_parameter("initial_wait_s").value)

        self.enable_b_scan = self.parse_bool(self.get_parameter("enable_b_scan").value)
        self.b_scan_offsets = self.parse_offsets(self.get_parameter("b_scan_offsets").value)
        self.scan_timeout_s = float(self.get_parameter("scan_timeout_s").value)
        self.after_scan_pose_wait_s = float(self.get_parameter("after_scan_pose_wait_s").value)

        self.target_timeout_s = float(self.get_parameter("target_timeout_s").value)
        self.max_step_mm = float(self.get_parameter("max_step_mm").value)
        self.edge_step_mm = float(self.get_parameter("edge_step_mm").value)
        self.move_speed = float(self.get_parameter("move_speed").value)
        self.step_wait_s = float(self.get_parameter("step_wait_s").value)
        self.adaptive_step_enabled = self.parse_bool(self.get_parameter("adaptive_step_enabled").value)
        self.adaptive_step_min_mm = float(self.get_parameter("adaptive_step_min_mm").value)
        self.adaptive_step_max_mm = float(self.get_parameter("adaptive_step_max_mm").value)
        self.adaptive_edge_step_min_mm = float(self.get_parameter("adaptive_edge_step_min_mm").value)
        self.adaptive_center_good_ratio = float(self.get_parameter("adaptive_center_good_ratio").value)
        self.adaptive_center_bad_ratio = float(self.get_parameter("adaptive_center_bad_ratio").value)

        self.enable_e_pixel_servo = self.parse_bool(self.get_parameter("enable_e_pixel_servo").value)
        self.desired_pixel_v = float(self.get_parameter("desired_pixel_v").value)
        self.pixel_v_deadband = float(self.get_parameter("pixel_v_deadband").value)
        self.e_kp_deg_per_px = float(self.get_parameter("e_kp_deg_per_px").value)
        self.e_max_step_deg = float(self.get_parameter("e_max_step_deg").value)
        self.e_min_deg = float(self.get_parameter("e_min_deg").value)
        self.e_max_deg = float(self.get_parameter("e_max_deg").value)
        self.e_pixel_sign = float(self.get_parameter("e_pixel_sign").value)
        self.e_servo_speed_deg_s = float(self.get_parameter("e_servo_speed_deg_s").value)
        self.e_servo_acc = float(self.get_parameter("e_servo_acc").value)

        self.target_xy_tolerance_mm = float(self.get_parameter("target_xy_tolerance_mm").value)
        self.target_z_tolerance_mm = float(self.get_parameter("target_z_tolerance_mm").value)

        self.safe_above_offset_mm = float(self.get_parameter("safe_above_offset_mm").value)
        self.grasp_offset_x_mm = float(self.get_parameter("grasp_offset_x_mm").value)
        self.grasp_offset_y_mm = float(self.get_parameter("grasp_offset_y_mm").value)
        self.grasp_offset_z_mm = float(self.get_parameter("grasp_offset_z_mm").value)
        self.min_safe_z_mm = float(self.get_parameter("min_safe_z_mm").value)
        self.servo_min_z_mm = float(self.get_parameter("servo_min_z_mm").value)
        self.servo_recover_z_margin_mm = float(self.get_parameter("servo_recover_z_margin_mm").value)
        self.servo_recover_max_attempts = int(self.get_parameter("servo_recover_max_attempts").value)

        self.enable_center_first_servo = self.parse_bool(self.get_parameter("enable_center_first_servo").value)
        self.center_pixel_u = float(self.get_parameter("center_pixel_u").value)
        self.center_pixel_v = float(self.get_parameter("center_pixel_v").value)
        self.center_pixel_deadband_u = float(self.get_parameter("center_pixel_deadband_u").value)
        self.center_pixel_deadband_v = float(self.get_parameter("center_pixel_deadband_v").value)
        self.center_stable_frames_required = int(self.get_parameter("center_stable_frames_required").value)
        self.center_b_kp_deg_per_px = float(self.get_parameter("center_b_kp_deg_per_px").value)
        self.center_b_max_step_deg = float(self.get_parameter("center_b_max_step_deg").value)
        self.center_e_kp_deg_per_px = float(self.get_parameter("center_e_kp_deg_per_px").value)
        self.center_e_max_step_deg = float(self.get_parameter("center_e_max_step_deg").value)
        self.center_b_pixel_sign = float(self.get_parameter("center_b_pixel_sign").value)
        self.center_e_pixel_sign = float(self.get_parameter("center_e_pixel_sign").value)
        self.center_step_wait_s = float(self.get_parameter("center_step_wait_s").value)
        self.approach_hold_z_mm = float(self.get_parameter("approach_hold_z_mm").value)

        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)
        self.safe_roi_x_min_ratio = float(self.get_parameter("safe_roi_x_min_ratio").value)
        self.safe_roi_x_max_ratio = float(self.get_parameter("safe_roi_x_max_ratio").value)
        self.safe_roi_y_min_ratio = float(self.get_parameter("safe_roi_y_min_ratio").value)
        self.safe_roi_y_max_ratio = float(self.get_parameter("safe_roi_y_max_ratio").value)

        self.base_x_min = float(self.get_parameter("base_x_min").value)
        self.base_x_max = float(self.get_parameter("base_x_max").value)
        self.base_y_min = float(self.get_parameter("base_y_min").value)
        self.base_y_max = float(self.get_parameter("base_y_max").value)
        self.base_z_min = float(self.get_parameter("base_z_min").value)
        self.base_z_max = float(self.get_parameter("base_z_max").value)

        # 下降测试参数
        self.descend_test_mm = float(self.get_parameter("descend_test_mm").value)
        self.descend_step_mm = float(self.get_parameter("descend_step_mm").value)
        self.descend_control_mode = str(self.get_parameter("descend_control_mode").value).strip().lower()
        self.descend_lock_xy = self.parse_bool(self.get_parameter("descend_lock_xy").value)
        self.descend_xy_step_mm = float(self.get_parameter("descend_xy_step_mm").value)
        self.descend_x_comp_mm_per_mm = float(self.get_parameter("descend_x_comp_mm_per_mm").value)
        self.descend_y_comp_mm_per_mm = float(self.get_parameter("descend_y_comp_mm_per_mm").value)
        self.descend_desired_pixel_u = float(self.get_parameter("descend_desired_pixel_u").value)
        self.descend_desired_pixel_v = float(self.get_parameter("descend_desired_pixel_v").value)
        self.descend_pixel_deadband = float(self.get_parameter("descend_pixel_deadband").value)
        self.descend_pixel_kp_mm_per_px = float(self.get_parameter("descend_pixel_kp_mm_per_px").value)
        self.descend_pixel_max_xy_step_mm = float(self.get_parameter("descend_pixel_max_xy_step_mm").value)
        self.descend_pixel_x_sign = float(self.get_parameter("descend_pixel_x_sign").value)
        self.descend_pixel_y_sign = float(self.get_parameter("descend_pixel_y_sign").value)
        self.descend_min_z_mm = float(self.get_parameter("descend_min_z_mm").value)
        self.descend_min_confidence = float(self.get_parameter("descend_min_confidence").value)
        self.descend_speed = float(self.get_parameter("descend_speed").value)
        self.descend_wait_s = float(self.get_parameter("descend_wait_s").value)
        self.descend_step_wait_s = float(self.get_parameter("descend_step_wait_s").value)

        # 抓取、抬升、放置参数
        self.enable_pick_place_sequence = self.parse_bool(self.get_parameter("enable_pick_place_sequence").value)
        self.gripper_close_deg = float(self.get_parameter("gripper_close_deg").value)
        self.gripper_open_deg = float(self.get_parameter("gripper_open_deg").value)
        self.gripper_speed_deg_s = float(self.get_parameter("gripper_speed_deg_s").value)
        self.gripper_acc = float(self.get_parameter("gripper_acc").value)
        self.gripper_wait_s = float(self.get_parameter("gripper_wait_s").value)
        self.lift_up_mm = float(self.get_parameter("lift_up_mm").value)
        self.lift_speed = float(self.get_parameter("lift_speed").value)
        self.lift_wait_s = float(self.get_parameter("lift_wait_s").value)
        self.place_x_mm = float(self.get_parameter("place_x_mm").value)
        self.place_y_mm = float(self.get_parameter("place_y_mm").value)
        self.place_z_mm = float(self.get_parameter("place_z_mm").value)
        self.place_speed = float(self.get_parameter("place_speed").value)
        self.place_wait_s = float(self.get_parameter("place_wait_s").value)
        self.hold_after_pick_s = float(self.get_parameter("hold_after_pick_s").value)
        self.place_wait_after_open_s = float(self.get_parameter("place_wait_after_open_s").value)
        self.base_adjust_step_m = float(self.get_parameter("base_adjust_step_m").value)
        self.base_adjust_y_margin_mm = float(self.get_parameter("base_adjust_y_margin_mm").value)

        self.pub_cmd = self.create_publisher(String, "/roarm_m3/cmd", 10)
        self.pub_state = self.create_publisher(String, "/red_block/visual_servo_state", 10)

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
        self.sub_visual_cmd = self.create_subscription(
            String,
            self.visual_servo_cmd_topic,
            self.on_visual_servo_cmd,
            10,
        )

        self.latest_arm_state = None
        self.latest_arm_state_time = 0.0

        self.latest_target = None
        self.latest_target_time = 0.0

        self.state = "INIT"
        self.started = False
        self.done = False
        self.busy = False

        self.scan_index = 0
        self.last_scan_switch_time = time.time()
        self.state_enter_time = time.time()
        self.last_command_time = 0.0
        self.pre_grasp_pose = None
        self.descend_done_mm = 0.0
        self.descend_start_z = None
        self.servo_recover_attempts = 0
        self.last_safe_servo_pose = None
        self.next_step_decay = 1.0
        self.center_stable_frames = 0
        self.recovering_after_lost = False
        self.last_move_origin = None
        self.active_task_id = None
        self.active_task_cmd = None
        self.task_result = None
        self.last_error = None
        self.base_adjust_request = None

        self.timer = self.create_timer(0.2, self.on_timer)

        self.get_logger().info("Visual servo task node started.")
        self.get_logger().info("This node only publishes /roarm_m3/cmd. It does not use camera or serial directly.")
        self.get_logger().info(
            f"competition_mode={self.competition_mode}, visual_servo_cmd_topic={self.visual_servo_cmd_topic}"
        )

    @staticmethod
    def parse_offsets(text):
        result = []
        for item in str(text).split(","):
            item = item.strip()
            if item:
                result.append(float(item))
        return result if result else [0.0]

    @staticmethod
    def parse_bool(value):
        if isinstance(value, bool):
            return value

        text = str(value).strip().lower()
        return text in ("1", "true", "yes", "on")

    def publish_cmd(self, cmd):
        msg = String()
        msg.data = json.dumps(cmd, ensure_ascii=False)
        self.pub_cmd.publish(msg)
        self.get_logger().info("PUB /roarm_m3/cmd: " + msg.data)

    def publish_state(self, extra=None):
        now = time.time()

        data = {
            "stamp": now,
            "state": self.state,
            "started": self.started,
            "done": self.done,
            "busy": self.busy,
            "scan_index": self.scan_index,
            "arm_state_age_s": None,
            "target_age_s": None,
            "competition_mode": self.competition_mode,
            "task_id": self.active_task_id,
            "task_cmd": self.active_task_cmd,
            "task_result": self.task_result,
            "error": self.last_error,
            "base_adjust": self.base_adjust_request,
        }

        if self.latest_arm_state is not None:
            data["arm_state_age_s"] = now - self.latest_arm_state_time

        if self.latest_target is not None:
            data["target_age_s"] = now - self.latest_target_time

        if extra:
            if "error" in extra:
                self.last_error = extra["error"]
                data["error"] = self.last_error
            if "base_adjust" in extra:
                self.base_adjust_request = extra["base_adjust"]
                data["base_adjust"] = self.base_adjust_request
            if "task_result" in extra:
                self.task_result = extra["task_result"]
                data["task_result"] = self.task_result
            data.update(extra)

        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        self.pub_state.publish(msg)

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
        pixel = data.get("pixel", None)

        if not isinstance(base, dict):
            return

        if not isinstance(pixel, dict):
            return

        self.latest_target = data
        self.latest_target_time = time.time()

    def on_visual_servo_cmd(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"Invalid visual servo command JSON: {exc}")
            return

        cmd = str(data.get("cmd", "")).strip()
        task_id = data.get("task_id", None)

        if cmd == "pick":
            self.start_competition_pick(task_id)
            return

        if cmd == "place_to_zone":
            self.start_competition_place(task_id)
            return

        self.get_logger().warn(f"Unsupported visual servo command: {cmd}")

    def reset_task_runtime(self, task_id, task_cmd):
        self.active_task_id = task_id
        self.active_task_cmd = task_cmd
        self.task_result = None
        self.last_error = None
        self.base_adjust_request = None
        self.done = False
        self.busy = False
        self.started = False
        self.scan_index = 0
        self.last_scan_switch_time = time.time()
        self.last_command_time = 0.0
        self.pre_grasp_pose = None
        self.descend_done_mm = 0.0
        self.descend_start_z = None
        self.servo_recover_attempts = 0
        self.last_safe_servo_pose = None
        self.next_step_decay = 1.0
        self.center_stable_frames = 0
        self.recovering_after_lost = False
        self.last_move_origin = None

    def start_competition_pick(self, task_id):
        self.reset_task_runtime(task_id, "pick")
        self.get_logger().info(f"Competition pick requested. task_id={task_id}")
        self.enter_state("INIT")

    def start_competition_place(self, task_id):
        self.reset_task_runtime(task_id, "place_to_zone")
        self.started = True
        self.get_logger().info(f"Competition place_to_zone requested. task_id={task_id}")
        self.enter_state("MOVE_TO_PLACE")

    def set_task_failed(self, error, extra=None):
        self.last_error = str(error)
        if self.competition_mode and self.active_task_cmd == "pick":
            self.task_result = "pick_failed"
        elif self.competition_mode and self.active_task_cmd == "place_to_zone":
            self.task_result = "place_failed"
        self.enter_state("FAIL")
        data = {"error": self.last_error, "task_result": self.task_result}
        if extra:
            data.update(extra)
        self.publish_state(data)

    def set_task_success(self, result):
        self.task_result = str(result)
        self.last_error = None
        self.done = True
        self.enter_state("DONE")
        self.publish_state({"task_result": self.task_result, "result": self.task_result})

    def check_competition_base_adjust_needed(self, target):
        if not self.competition_mode or self.active_task_cmd != "pick":
            return False

        base = target.get("base_mm", None)
        if not isinstance(base, dict):
            return False

        try:
            y = float(base["y"]) + self.grasp_offset_y_mm
        except Exception:
            return False

        if y <= self.base_y_min + self.base_adjust_y_margin_mm:
            self.base_adjust_request = {
                "direction": "left",
                "step_m": self.base_adjust_step_m,
                "reason": "target_too_right",
            }
            return True

        if y >= self.base_y_max - self.base_adjust_y_margin_mm:
            self.base_adjust_request = {
                "direction": "right",
                "step_m": self.base_adjust_step_m,
                "reason": "target_too_left",
            }
            return True

        return False

    def target_is_fresh(self):
        if self.latest_target is None:
            return False
        return time.time() - self.latest_target_time <= self.target_timeout_s

    def target_quality_ok(self, min_confidence=None, require_pixel_safe=False):
        if not self.target_is_fresh():
            return False, "target_not_fresh"

        target = self.latest_target
        if target is None:
            return False, "no_target"

        try:
            confidence = float(target.get("confidence", 0.0))
        except Exception:
            return False, "bad_confidence"

        threshold = self.descend_min_confidence if min_confidence is None else float(min_confidence)
        if confidence < threshold:
            return False, f"low_confidence:{confidence:.2f}"

        if require_pixel_safe and not self.target_pixel_is_safe(target):
            return False, "pixel_out_of_safe_roi"

        return True, "ok"

    def arm_state_is_fresh(self):
        if self.latest_arm_state is None:
            return False
        return time.time() - self.latest_arm_state_time <= 1.0

    def enter_state(self, new_state):
        if self.state != new_state:
            self.get_logger().info(f"State {self.state} -> {new_state}")
            self.state = new_state
            self.state_enter_time = time.time()

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

            if abs(value) > 6.5:
                return math.radians(value)

            return value

        return float(default)

    def target_pixel_is_safe(self, target):
        pixel = target.get("pixel", {})
        u = float(pixel.get("x", self.image_width / 2))
        v = float(pixel.get("y", self.image_height / 2))

        x_min = self.image_width * self.safe_roi_x_min_ratio
        x_max = self.image_width * self.safe_roi_x_max_ratio
        y_min = self.image_height * self.safe_roi_y_min_ratio
        y_max = self.image_height * self.safe_roi_y_max_ratio

        return x_min <= u <= x_max and y_min <= v <= y_max

    def target_center_ratio(self, target):
        pixel = target.get("pixel", {})
        u = float(pixel.get("x", self.image_width / 2))
        v = float(pixel.get("y", self.image_height / 2))

        du = u - self.image_width / 2.0
        dv = v - self.image_height / 2.0
        max_dist = math.hypot(self.image_width / 2.0, self.image_height / 2.0)

        return math.hypot(du, dv) / max(max_dist, 1.0)

    def compute_adaptive_step_mm(self, target, pixel_safe):
        if not self.adaptive_step_enabled:
            return self.max_step_mm if pixel_safe else self.edge_step_mm, 1.0, None

        center_ratio = self.target_center_ratio(target)
        good = self.adaptive_center_good_ratio
        bad = max(self.adaptive_center_bad_ratio, good + 1e-6)

        if center_ratio <= good:
            quality = 1.0
        elif center_ratio >= bad:
            quality = 0.0
        else:
            quality = 1.0 - (center_ratio - good) / (bad - good)

        if pixel_safe:
            low = self.adaptive_step_min_mm
            high = min(self.max_step_mm, self.adaptive_step_max_mm)
        else:
            low = self.adaptive_edge_step_min_mm
            high = min(self.edge_step_mm, self.adaptive_step_max_mm)

        step_mm = low + (high - low) * quality
        decay = self.next_step_decay
        step_mm *= decay
        step_mm = self.clamp(step_mm, low, high)
        self.next_step_decay = 1.0

        info = {
            "center_ratio": center_ratio,
            "quality": quality,
            "decay": decay,
            "low": low,
            "high": high,
            "effective_step_mm": step_mm,
        }

        return step_mm, quality, info

    def build_target_eef(self, target_base):
        target_x = float(target_base["x"]) + self.grasp_offset_x_mm
        target_y = float(target_base["y"]) + self.grasp_offset_y_mm
        target_z = max(
            float(target_base["z"]) + self.safe_above_offset_mm + self.grasp_offset_z_mm,
            self.servo_min_z_mm,
        )

        ok, reason = self.check_target_range(target_x, target_y, target_z)
        if not ok:
            raise RuntimeError(reason)

        return {
            "x": target_x,
            "y": target_y,
            "z": target_z,
        }

    def check_target_range(self, x, y, z):
        if not (self.base_x_min <= x <= self.base_x_max):
            return False, f"x out of range: {x:.1f}"
        if not (self.base_y_min <= y <= self.base_y_max):
            return False, f"y out of range: {y:.1f}"
        if not (self.base_z_min <= z <= self.base_z_max):
            return False, f"z out of range: {z:.1f}"
        return True, "ok"

    @staticmethod
    def dist_xy(a, b):
        return math.hypot(a["x"] - b["x"], a["y"] - b["y"])

    @staticmethod
    def limit_delta(value, max_abs):
        return max(-max_abs, min(max_abs, value))

    def compute_step_target(self, current_pose, target_eef, max_step):
        dx = target_eef["x"] - current_pose["x"]
        dy = target_eef["y"] - current_pose["y"]
        dz = target_eef["z"] - current_pose["z"]

        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist <= max_step:
            return dict(target_eef), dist

        ratio = max_step / max(dist, 1e-6)

        return {
            "x": current_pose["x"] + dx * ratio,
            "y": current_pose["y"] + dy * ratio,
            "z": current_pose["z"] + dz * ratio,
        }, dist

    def publish_move_pose(self, step_target, pose, label):
        if step_target["z"] < self.servo_min_z_mm:
            self.get_logger().warn(
                f"视觉闭环目标 z={step_target['z']:.1f} mm 低于 servo_min_z_mm="
                f"{self.servo_min_z_mm:.1f} mm，已钳制。"
            )
            step_target = dict(step_target)
            step_target["z"] = self.servo_min_z_mm

        cmd = {
            "type": "move_pose",
            "x": float(step_target["x"]),
            "y": float(step_target["y"]),
            "z": float(step_target["z"]),
            "t": float(pose["t"]),
            "r": float(pose["r"]),
            "g": float(pose["g"]),
            "speed": self.move_speed,
            "label": label,
        }

        self.publish_cmd(cmd)

    def do_descend_test(self):
        """
        执行视觉守护下降：
        - 每次只下降 descend_step_mm
        - 下降前检查目标新鲜度、置信度和像素安全区域
        - pixel 模式下先用图像误差做小步 XY 修正，目标回到中心附近才下降
        """
        if not self.arm_state_is_fresh():
            self.get_logger().warn("没有新的机械臂状态，等待下降。")
            return

        ok, reason = self.target_quality_ok(
            min_confidence=self.descend_min_confidence,
            require_pixel_safe=True,
        )
        if not ok:
            self.get_logger().error(f"下降前视觉质量不足，停止下降：{reason}")
            self.enter_state("FAIL")
            self.publish_state(
                {
                    "error": "descend_target_quality_bad",
                    "reason": reason,
                    "descend_done_mm": self.descend_done_mm,
                }
            )
            return

        pose = self.get_pose_from_arm_state()
        if self.descend_start_z is None:
            self.descend_start_z = pose["z"]

        remaining_mm = self.descend_test_mm - self.descend_done_mm
        if remaining_mm <= 0.0:
            self.get_logger().info("视觉守护下降累计距离已完成。")
            self.enter_state("WAIT_AFTER_DESCEND")
            self.last_command_time = time.time()
            return

        pixel = self.latest_target.get("pixel", {})
        try:
            pixel_u = float(pixel["x"])
            pixel_v = float(pixel["y"])
        except Exception:
            self.get_logger().error("下降阶段像素信息无效，停止下降。")
            self.enter_state("FAIL")
            self.publish_state({"error": "descend_bad_pixel"})
            return

        err_u = self.descend_desired_pixel_u - pixel_u
        err_v = self.descend_desired_pixel_v - pixel_v
        pixel_err = math.hypot(err_u, err_v)
        center_ok = pixel_err <= self.descend_pixel_deadband

        step_z_mm = min(self.descend_step_mm, remaining_mm) if center_ok else 0.0
        target_z = pose["z"] - step_z_mm

        # 检查安全限制
        if target_z < self.descend_min_z_mm:
            self.get_logger().error(
                f"下降目标 z={target_z:.1f} mm 低于安全限制 {self.descend_min_z_mm:.1f} mm，中止。"
            )
            self.enter_state("FAIL")
            self.publish_state(
                {
                    "error": "descend_below_safety_limit",
                    "target_z": target_z,
                    "min_z": self.descend_min_z_mm,
                }
            )
            return

        anchor_pose = self.pre_grasp_pose if self.pre_grasp_pose is not None else pose
        compensate_x = self.descend_x_comp_mm_per_mm * (self.descend_done_mm + step_z_mm)
        compensate_y = self.descend_y_comp_mm_per_mm * (self.descend_done_mm + step_z_mm)

        if self.descend_control_mode == "pixel":
            dx = self.descend_pixel_x_sign * self.descend_pixel_kp_mm_per_px * err_u
            dy = self.descend_pixel_y_sign * self.descend_pixel_kp_mm_per_px * err_v
            dx = self.limit_delta(dx, self.descend_pixel_max_xy_step_mm)
            dy = self.limit_delta(dy, self.descend_pixel_max_xy_step_mm)

            target_x = pose["x"] + dx + compensate_x
            target_y = pose["y"] + dy + compensate_y

        elif self.descend_lock_xy:
            target_x = anchor_pose["x"] + compensate_x
            target_y = anchor_pose["y"] + compensate_y
        else:
            target_base = self.latest_target["base_mm"]
            desired_x = float(target_base["x"]) + self.grasp_offset_x_mm + compensate_x
            desired_y = float(target_base["y"]) + self.grasp_offset_y_mm + compensate_y
            target_x = pose["x"] + self.limit_delta(desired_x - pose["x"], self.descend_xy_step_mm)
            target_y = pose["y"] + self.limit_delta(desired_y - pose["y"], self.descend_xy_step_mm)

        descend_target = {
            "x": target_x,
            "y": target_y,
            "z": target_z,
        }

        ok, reason = self.check_target_range(
            descend_target["x"],
            descend_target["y"],
            descend_target["z"],
        )
        if not ok:
            self.get_logger().error(f"下降目标超出工作空间：{reason}")
            self.enter_state("FAIL")
            self.publish_state(
                {
                    "error": "descend_out_of_range",
                    "reason": reason,
                    "target": descend_target,
                }
            )
            return

        confidence = float(self.latest_target.get("confidence", 0.0))
        self.get_logger().info(
            f"视觉守护下降：done={self.descend_done_mm:.1f}/{self.descend_test_mm:.1f} mm "
            f"mode={self.descend_control_mode} conf={confidence:.2f} "
            f"pixel=({pixel_u:.1f},{pixel_v:.1f}) err=({err_u:.1f},{err_v:.1f}) "
            f"center_ok={center_ok} "
            f"current=({pose['x']:.1f},{pose['y']:.1f},{pose['z']:.1f}) "
            f"target=({descend_target['x']:.1f},{descend_target['y']:.1f},{descend_target['z']:.1f})"
        )

        cmd = {
            "type": "move_pose",
            "x": float(descend_target["x"]),
            "y": float(descend_target["y"]),
            "z": float(descend_target["z"]),
            "t": float(pose["t"]),
            "r": float(pose["r"]),
            "g": float(pose["g"]),
            "speed": self.descend_speed,
            "label": "descend-servo-step",
        }

        self.publish_cmd(cmd)
        self.last_command_time = time.time()
        self.busy = True
        self.enter_state("WAIT_AFTER_DESCEND_STEP")

    def publish_gripper_joint(self, angle_deg, label):
        cmd = {
            "type": "move_joint",
            "joint": 6,
            "angle": float(angle_deg),
            "speed": self.gripper_speed_deg_s,
            "acc": self.gripper_acc,
            "label": label,
        }

        self.publish_cmd(cmd)
        self.last_command_time = time.time()
        self.busy = True

    def publish_recover_servo_height(self, pose):
        anchor_pose = self.last_safe_servo_pose if self.last_safe_servo_pose is not None else pose
        recover_z = self.servo_min_z_mm + self.servo_recover_z_margin_mm

        target = {
            "x": anchor_pose["x"],
            "y": anchor_pose["y"],
            "z": recover_z,
        }

        self.get_logger().warn(
            f"当前 z={pose['z']:.1f} mm 低于视觉闭环安全高度 "
            f"{self.servo_min_z_mm:.1f} mm，恢复到 z={recover_z:.1f} mm。"
        )
        self.publish_move_pose(target, pose, "recover-servo-min-z")
        self.last_command_time = time.time()
        self.busy = True

    def do_close_gripper(self):
        """
        闭合夹爪。
        夹爪角度先通过参数保守配置，现场根据实际夹持力度微调。
        """
        self.get_logger().info(f"闭爪：g -> {self.gripper_close_deg:.1f} deg")
        self.publish_gripper_joint(self.gripper_close_deg, "close-gripper")
        self.enter_state("WAIT_AFTER_CLOSE_GRIPPER")

    def do_open_gripper(self):
        """
        打开夹爪，用于放置后释放物料。
        """
        self.get_logger().info(f"开爪：g -> {self.gripper_open_deg:.1f} deg")
        self.publish_gripper_joint(self.gripper_open_deg, "open-gripper")
        self.enter_state("WAIT_AFTER_OPEN_GRIPPER")

    def publish_pose_with_speed(self, target, pose, speed, label):
        cmd = {
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

        self.publish_cmd(cmd)
        self.last_command_time = time.time()
        self.busy = True

    def do_lift_after_grasp(self):
        """
        闭爪后沿 z 方向抬升，离开物料区域。
        """
        if not self.arm_state_is_fresh():
            self.get_logger().warn("没有新的机械臂状态，等待抬升。")
            return

        pose = self.get_pose_from_arm_state()
        target = {
            "x": pose["x"],
            "y": pose["y"],
            "z": pose["z"] + self.lift_up_mm,
        }

        ok, reason = self.check_target_range(target["x"], target["y"], target["z"])
        if not ok:
            self.get_logger().error(f"抬升目标超出工作空间：{reason}")
            self.enter_state("FAIL")
            self.publish_state({"error": "lift_out_of_range", "reason": reason, "target": target})
            return

        self.get_logger().info(
            f"抬升：当前 z={pose['z']:.1f} mm -> 目标 z={target['z']:.1f} mm "
            f"（抬升 {self.lift_up_mm:.1f} mm）"
        )
        self.publish_pose_with_speed(target, pose, self.lift_speed, "lift-after-grasp")
        self.enter_state("WAIT_AFTER_LIFT")

    def do_place_move(self):
        """
        移动到放置点。
        放置点使用 base 坐标，由 launch 参数配置。
        """
        if not self.arm_state_is_fresh():
            self.get_logger().warn("没有新的机械臂状态，等待移动到放置点。")
            return

        pose = self.get_pose_from_arm_state()
        target = {
            "x": self.place_x_mm,
            "y": self.place_y_mm,
            "z": self.place_z_mm,
        }

        ok, reason = self.check_target_range(target["x"], target["y"], target["z"])
        if not ok:
            self.get_logger().error(f"放置点超出工作空间：{reason}")
            self.enter_state("FAIL")
            self.publish_state({"error": "place_out_of_range", "reason": reason, "target": target})
            return

        self.get_logger().info(
            f"移动到放置点：x={target['x']:.1f}, y={target['y']:.1f}, z={target['z']:.1f}"
        )
        self.publish_pose_with_speed(target, pose, self.place_speed, "move-to-place")
        self.enter_state("WAIT_AFTER_PLACE")


    def get_current_e_deg(self):
        if self.latest_arm_state is None:
            return None

        # RoArm 原始状态里 e 通常是弧度；部分脚本打印的是角度。
        # 这里做兼容：绝对值小于 6.5 当弧度处理，否则当角度处理。
        try:
            e_value = float(self.latest_arm_state.get("e", 0.0))
        except Exception:
            return None

        if abs(e_value) <= 6.5:
            return math.degrees(e_value)

        return e_value

    def get_current_joint_deg(self, keys, default=0.0):
        if self.latest_arm_state is None:
            return None

        for key in keys:
            if key not in self.latest_arm_state:
                continue
            try:
                value = float(self.latest_arm_state[key])
            except Exception:
                continue

            if abs(value) <= 6.5:
                return math.degrees(value)

            return value

        return float(default)

    def publish_b_joint_correction(self, target_b_deg, reason):
        cmd = {
            "type": "move_joint",
            "joint": 1,
            "angle": float(target_b_deg),
            "speed": self.e_servo_speed_deg_s,
            "acc": self.e_servo_acc,
            "label": reason,
        }
        self.publish_cmd(cmd)

    def target_centered(self, target):
        pixel = target.get("pixel", {})
        try:
            u = float(pixel.get("x", self.center_pixel_u))
            v = float(pixel.get("y", self.center_pixel_v))
        except Exception:
            return False

        return (
            abs(u - self.center_pixel_u) <= self.center_pixel_deadband_u
            and abs(v - self.center_pixel_v) <= self.center_pixel_deadband_v
        )

    def get_target_center_errors(self, target):
        pixel = target.get("pixel", {})
        try:
            u = float(pixel.get("x", self.center_pixel_u))
            v = float(pixel.get("y", self.center_pixel_v))
        except Exception:
            return None, None

        return self.center_pixel_u - u, self.center_pixel_v - v

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))

    def publish_e_joint_correction(self, target_e_deg, reason):
        cmd = {
            "type": "move_joint",
            "joint": 3,
            "angle": float(target_e_deg),
            "speed": self.e_servo_speed_deg_s,
            "acc": self.e_servo_acc,
            "label": reason,
        }
        self.publish_cmd(cmd)

    def should_servo_e_by_pixel(self):
        if not self.enable_e_pixel_servo:
            return False, None

        if self.latest_target is None:
            return False, None

        pixel = self.latest_target.get("pixel", None)
        if not isinstance(pixel, dict):
            return False, None

        try:
            current_v = float(pixel["y"])
        except Exception:
            return False, None

        err_v = self.desired_pixel_v - current_v

        if abs(err_v) <= self.pixel_v_deadband:
            return False, {
                "err_v": err_v,
                "current_v": current_v,
            }

        current_e_deg = self.get_current_e_deg()
        if current_e_deg is None:
            return False, None

        delta_e = self.e_pixel_sign * self.e_kp_deg_per_px * err_v
        delta_e = self.clamp(delta_e, -self.e_max_step_deg, self.e_max_step_deg)

        target_e_deg = current_e_deg + delta_e
        target_e_deg = self.clamp(target_e_deg, self.e_min_deg, self.e_max_deg)

        info = {
            "current_v": current_v,
            "desired_v": self.desired_pixel_v,
            "err_v": err_v,
            "current_e_deg": current_e_deg,
            "delta_e_deg": delta_e,
            "target_e_deg": target_e_deg,
        }

        return True, info

    def do_visual_servo_step(self, fixed_z=None):
        if not self.target_is_fresh():
            self.enter_state("WAIT_TARGET")
            return

        if not self.arm_state_is_fresh():
            self.get_logger().warn("No fresh arm state. Wait.")
            return

        need_e_servo, e_info = self.should_servo_e_by_pixel()
        if need_e_servo:
            self.get_logger().info(
                "E pixel servo: "
                f"v={e_info['current_v']:.1f}, desired={e_info['desired_v']:.1f}, "
                f"err={e_info['err_v']:.1f}, "
                f"e={e_info['current_e_deg']:.1f} -> {e_info['target_e_deg']:.1f}, "
                f"delta={e_info['delta_e_deg']:.2f}"
            )
            self.publish_e_joint_correction(e_info["target_e_deg"], "e-pixel-servo")
            self.last_command_time = time.time()
            self.busy = True
            self.last_move_origin = "SERVO_STEP"
            self.enter_state("WAIT_AFTER_STEP")
            return

        pose = self.get_pose_from_arm_state()
        if fixed_z is None:
            if pose["z"] < self.servo_min_z_mm:
                self.servo_recover_attempts += 1
                if self.servo_recover_attempts > self.servo_recover_max_attempts:
                    self.get_logger().error(
                        f"连续 {self.servo_recover_attempts} 次恢复高度失败，停止视觉闭环。"
                    )
                    self.enter_state("FAIL")
                    self.publish_state(
                        {
                            "error": "servo_recover_failed",
                            "current_pose": pose,
                            "servo_min_z_mm": self.servo_min_z_mm,
                            "recover_attempts": self.servo_recover_attempts,
                        }
                    )
                    return

                self.publish_recover_servo_height(pose)
                self.last_move_origin = "SERVO_STEP"
                self.enter_state("WAIT_AFTER_STEP")
                return

            self.servo_recover_attempts = 0
            self.last_safe_servo_pose = dict(pose)
        else:
            self.servo_recover_attempts = 0

        if self.check_competition_base_adjust_needed(self.latest_target):
            self.get_logger().warn(
                "Competition pick target is near lateral workspace edge. Request base adjustment."
            )
            self.set_task_failed(
                "need_base_adjust",
                {"base_adjust": self.base_adjust_request},
            )
            return

        target_base = self.latest_target["base_mm"]
        try:
            target_eef = self.build_target_eef(target_base)
        except Exception as exc:
            self.set_task_failed("target_out_of_workspace", {"reason": str(exc)})
            return
        if fixed_z is not None:
            target_eef["z"] = max(fixed_z, self.servo_min_z_mm)

        xy_error = self.dist_xy(pose, target_eef)
        z_error = abs(pose["z"] - target_eef["z"])

        if xy_error <= self.target_xy_tolerance_mm and z_error <= self.target_z_tolerance_mm:
            self.get_logger().info("视觉闭环到达预抓取点。现在进入下降测试。")
            # 保存当前位姿用于下降测试
            self.pre_grasp_pose = dict(pose)
            self.enter_state("REACHED_PRE_GRASP")
            self.publish_state(
                {
                    "phase": "reached_pre_grasp",
                    "target_eef": target_eef,
                    "xy_error": xy_error,
                    "z_error": z_error,
                }
            )
            return

        pixel_safe = self.target_pixel_is_safe(self.latest_target)
        effective_step_mm, _, adaptive_info = self.compute_adaptive_step_mm(
            self.latest_target,
            pixel_safe,
        )

        if fixed_z is not None:
            label = "approach-centered-step"
        elif pixel_safe:
            label = "visual-servo-step"
        else:
            label = "edge-safe-step"

        step_target, full_dist = self.compute_step_target(pose, target_eef, effective_step_mm)
        if fixed_z is None and not pixel_safe and step_target["z"] < pose["z"]:
            step_target["z"] = pose["z"]

        adaptive_text = ""
        if adaptive_info is not None:
            adaptive_text = (
                f" effective_step={adaptive_info['effective_step_mm']:.1f} "
                f"center_ratio={adaptive_info['center_ratio']:.2f} "
                f"quality={adaptive_info['quality']:.2f} "
                f"decay={adaptive_info['decay']:.2f}"
            )

        self.get_logger().info(
            f"{label}: current=({pose['x']:.1f},{pose['y']:.1f},{pose['z']:.1f}) "
            f"target=({target_eef['x']:.1f},{target_eef['y']:.1f},{target_eef['z']:.1f}) "
            f"step=({step_target['x']:.1f},{step_target['y']:.1f},{step_target['z']:.1f}) "
            f"full_dist={full_dist:.1f} pixel_safe={pixel_safe}{adaptive_text}"
        )

        self.publish_move_pose(step_target, pose, label)
        self.last_command_time = time.time()
        self.busy = True
        self.last_move_origin = "APPROACH_CENTERED" if fixed_z is not None else "SERVO_STEP"
        self.enter_state("WAIT_AFTER_STEP")

    def do_center_target(self):
        if not self.target_is_fresh():
            self.get_logger().warn("Center target: target not fresh, wait target.")
            self.center_stable_frames = 0
            self.enter_state("WAIT_TARGET")
            return

        if not self.arm_state_is_fresh():
            self.get_logger().warn("Center target: no fresh arm state.")
            return

        if self.latest_target is None:
            self.center_stable_frames = 0
            self.enter_state("WAIT_TARGET")
            return

        pixel = self.latest_target.get("pixel", None)
        if not isinstance(pixel, dict):
            self.center_stable_frames = 0
            self.enter_state("WAIT_TARGET")
            return

        confidence = float(self.latest_target.get("confidence", 0.0))
        if confidence < 0.40:
            self.get_logger().warn(
                f"Center target: low confidence {confidence:.2f}, wait target."
            )
            self.center_stable_frames = 0
            self.enter_state("WAIT_TARGET")
            return

        if self.target_centered(self.latest_target):
            self.center_stable_frames += 1
            self.get_logger().info(
                f"Center target stable {self.center_stable_frames}/{self.center_stable_frames_required}."
            )
            if self.center_stable_frames >= self.center_stable_frames_required:
                self.center_stable_frames = 0
                self.get_logger().info("Target centered. Enter APPROACH_CENTERED.")
                self.enter_state("APPROACH_CENTERED")
                return

            self.last_command_time = time.time()
            self.busy = False
            self.last_move_origin = "CENTER_TARGET"
            self.enter_state("WAIT_AFTER_CENTER_STEP")
            return

        self.center_stable_frames = 0
        err_u, err_v = self.get_target_center_errors(self.latest_target)
        if err_u is None or err_v is None:
            self.center_stable_frames = 0
            self.enter_state("WAIT_TARGET")
            return

        move_b = abs(err_u) > self.center_pixel_deadband_u
        move_e = abs(err_v) > self.center_pixel_deadband_v

        if not move_b and not move_e:
            self.last_command_time = time.time()
            self.busy = False
            self.last_move_origin = "CENTER_TARGET"
            self.enter_state("WAIT_AFTER_CENTER_STEP")
            return

        current_b_deg = self.get_current_joint_deg(["b", "pose_b", "tool_b", "bia"], default=0.0)
        current_e_deg = self.get_current_e_deg()
        if current_e_deg is None:
            self.get_logger().warn("Center target: no current E angle.")
            return

        delta_b = self.center_b_pixel_sign * self.center_b_kp_deg_per_px * err_u
        delta_b = self.clamp(delta_b, -self.center_b_max_step_deg, self.center_b_max_step_deg)
        target_b = self.clamp(current_b_deg + delta_b, -180.0, 180.0)

        delta_e = self.center_e_pixel_sign * self.center_e_kp_deg_per_px * err_v
        delta_e = self.clamp(delta_e, -self.center_e_max_step_deg, self.center_e_max_step_deg)
        target_e = self.clamp(current_e_deg + delta_e, self.e_min_deg, self.e_max_deg)

        if move_b and (not move_e or abs(err_u) >= abs(err_v)):
            self.get_logger().info(
                f"Center target adjust B: err_u={err_u:.1f}, b_sign={self.center_b_pixel_sign:.1f}, "
                f"delta_b={delta_b:.2f}, current_b={current_b_deg:.1f} -> target_b={target_b:.1f}"
            )
            self.publish_b_joint_correction(target_b, "center-b-servo")
        else:
            self.get_logger().info(
                f"Center target adjust E: err_v={err_v:.1f}, e_sign={self.center_e_pixel_sign:.1f}, "
                f"delta_e={delta_e:.2f}, current_e={current_e_deg:.1f} -> target_e={target_e:.1f}"
            )
            self.publish_e_joint_correction(target_e, "center-e-servo")

        self.last_command_time = time.time()
        self.busy = True
        self.last_move_origin = "CENTER_TARGET"
        self.enter_state("WAIT_AFTER_CENTER_STEP")

    def do_approach_centered_step(self):
        if not self.target_is_fresh():
            self.get_logger().warn("Approach centered: target not fresh, wait target.")
            self.enter_state("WAIT_TARGET")
            return

        if not self.arm_state_is_fresh():
            self.get_logger().warn("Approach centered: no fresh arm state.")
            return

        if not self.target_centered(self.latest_target):
            self.get_logger().info("Approach centered: target moved off center, return to CENTER_TARGET.")
            self.enter_state("CENTER_TARGET")
            return

        hold_z = max(self.approach_hold_z_mm, self.servo_min_z_mm)
        self.do_visual_servo_step(fixed_z=hold_z)

    def on_timer(self):
        self.publish_state()

        if not self.auto_start and self.active_task_cmd is None:
            return

        now = time.time()

        if self.state == "INIT":
            self.get_logger().info("INIT: send initial pose.")
            self.send_initial_pose(b_offset_deg=0.0)
            self.scan_index = 0
            self.started = True
            self.last_scan_switch_time = now
            self.enter_state("WAIT_INITIAL")
            return

        if self.state == "WAIT_INITIAL":
            if now - self.state_enter_time >= self.initial_wait_s:
                self.enter_state("WAIT_TARGET")
            return

        if self.state == "WAIT_TARGET":
            if self.done and self.move_once_only:
                self.enter_state("DONE")
                return

            if self.target_is_fresh():
                if self.enable_center_first_servo:
                    self.enter_state("CENTER_TARGET")
                else:
                    self.enter_state("SERVO_STEP")
                return

            if self.enable_b_scan and now - self.last_scan_switch_time > self.scan_timeout_s:
                self.scan_index += 1

                if self.scan_index >= len(self.b_scan_offsets):
                    self.get_logger().warn("All b scan views tried. Task failed.")
                    self.set_task_failed("target_not_found")
                    return

                offset = self.b_scan_offsets[self.scan_index]
                self.get_logger().info(f"No target. Scan next b offset: {offset:.1f} deg")
                self.send_initial_pose(b_offset_deg=offset)
                self.last_scan_switch_time = now + self.after_scan_pose_wait_s
                self.enter_state("WAIT_INITIAL")
                return

        if self.state == "CENTER_TARGET":
            self.do_center_target()
            return

        if self.state == "WAIT_AFTER_CENTER_STEP":
            if now - self.last_command_time >= self.center_step_wait_s:
                self.busy = False
                self.enter_state("CENTER_TARGET")
            return

        if self.state == "SERVO_STEP":
            self.do_visual_servo_step()
            return

        if self.state == "APPROACH_CENTERED":
            self.do_approach_centered_step()
            return

        if self.state == "WAIT_AFTER_STEP":
            if now - self.last_command_time >= self.step_wait_s:
                self.busy = False

                if self.recovering_after_lost:
                    self.recovering_after_lost = False
                    self.get_logger().info("Recovered to safe pose after target loss. Enter WAIT_TARGET.")
                    self.enter_state("WAIT_TARGET")
                    return

                if not self.target_is_fresh():
                    self.get_logger().warn("Target lost after step. Recover and wait/search.")
                    self.next_step_decay = 0.6
                    if self.last_safe_servo_pose is not None and self.arm_state_is_fresh():
                        self.publish_recover_servo_height(self.get_pose_from_arm_state())
                        self.recovering_after_lost = True
                        return
                    self.enter_state("WAIT_TARGET")
                    return

                if self.enable_center_first_servo and self.last_move_origin == "APPROACH_CENTERED":
                    if not self.target_centered(self.latest_target):
                        self.get_logger().info(
                            "Target moved off center after approach step, return to CENTER_TARGET."
                        )
                        self.enter_state("CENTER_TARGET")
                    else:
                        self.enter_state("APPROACH_CENTERED")
                else:
                    self.enter_state("SERVO_STEP")
            return

        if self.state == "REACHED_PRE_GRASP":
            self.get_logger().info("REACHED_PRE_GRASP：准备下降测试。")
            self.descend_done_mm = 0.0
            self.descend_start_z = None
            self.enter_state("DESCEND_TEST")
            return

        if self.state == "DESCEND_TEST":
            self.do_descend_test()
            return

        if self.state == "WAIT_AFTER_DESCEND_STEP":
            if now - self.last_command_time >= self.descend_step_wait_s:
                self.busy = False

                if not self.arm_state_is_fresh():
                    self.get_logger().warn("等待新的机械臂状态以更新下降进度。")
                    return

                pose = self.get_pose_from_arm_state()
                if self.descend_start_z is not None:
                    self.descend_done_mm = max(0.0, self.descend_start_z - pose["z"])

                if self.descend_done_mm >= self.descend_test_mm - 1e-3:
                    self.get_logger().info("视觉守护下降完成。")
                    self.last_command_time = now
                    self.enter_state("WAIT_AFTER_DESCEND")
                else:
                    self.enter_state("DESCEND_TEST")
            return

        if self.state == "WAIT_AFTER_DESCEND":
            if now - self.last_command_time >= self.descend_wait_s:
                self.busy = False

                if self.enable_pick_place_sequence or (
                    self.competition_mode and self.active_task_cmd == "pick"
                ):
                    self.get_logger().info("下降完成。进入闭爪阶段。")
                    self.enter_state("CLOSE_GRIPPER")
                else:
                    self.done = True
                    self.get_logger().info("下降测试完成。进入 DONE 状态。")
                    self.enter_state("DONE")
                    self.publish_state(
                        {
                            "result": "reached_descend_test",
                        }
                    )
            return

        if self.state == "CLOSE_GRIPPER":
            self.do_close_gripper()
            return

        if self.state == "WAIT_AFTER_CLOSE_GRIPPER":
            if now - self.last_command_time >= self.gripper_wait_s:
                self.busy = False
                self.enter_state("LIFT_AFTER_GRASP")
            return

        if self.state == "LIFT_AFTER_GRASP":
            self.do_lift_after_grasp()
            return

        if self.state == "WAIT_AFTER_LIFT":
            if now - self.last_command_time >= self.lift_wait_s:
                self.busy = False
                if self.competition_mode and self.active_task_cmd == "pick":
                    self.last_command_time = now
                    self.enter_state("HOLD_AFTER_PICK")
                    return
                self.enter_state("MOVE_TO_PLACE")
            return

        if self.state == "HOLD_AFTER_PICK":
            if now - self.last_command_time >= self.hold_after_pick_s:
                self.set_task_success("pick_success")
            return

        if self.state == "MOVE_TO_PLACE":
            self.do_place_move()
            return

        if self.state == "WAIT_AFTER_PLACE":
            if now - self.last_command_time >= self.place_wait_s:
                self.busy = False
                self.enter_state("OPEN_GRIPPER")
            return

        if self.state == "OPEN_GRIPPER":
            self.do_open_gripper()
            return

        if self.state == "WAIT_AFTER_OPEN_GRIPPER":
            wait_s = self.place_wait_after_open_s if (
                self.competition_mode and self.active_task_cmd == "place_to_zone"
            ) else self.gripper_wait_s
            if now - self.last_command_time >= wait_s:
                self.busy = False
                if self.competition_mode and self.active_task_cmd == "place_to_zone":
                    self.send_initial_pose(b_offset_deg=0.0)
                    self.last_command_time = now
                    self.busy = True
                    self.enter_state("WAIT_AFTER_PLACE_RESET")
                    return
                self.done = True
                self.get_logger().info("抓取、抬升、放置流程完成。进入 DONE 状态。")
                self.enter_state("DONE")
                self.publish_state(
                    {
                        "result": "picked_and_placed",
                    }
                )
            return

        if self.state == "WAIT_AFTER_PLACE_RESET":
            if now - self.last_command_time >= self.initial_wait_s:
                self.busy = False
                self.set_task_success("place_success")
            return

        if self.state == "DONE":
            return

        if self.state == "FAIL":
            return


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoTaskNode()

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
