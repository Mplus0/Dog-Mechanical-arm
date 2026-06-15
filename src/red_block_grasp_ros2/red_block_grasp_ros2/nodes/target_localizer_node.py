#!/usr/bin/env python3
import json
import math
import os
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from roarm_msgs.srv import GetPoseCmd
from std_msgs.msg import String

from red_block_grasp_ros2.core.camera_rgbd_orbbec import OrbbecRgbdCamera
from red_block_grasp_ros2.core.color_red_block_detector import RedColorBlockDetector
from red_block_grasp_ros2.core.target_localizer import TargetLocalizer
from red_block_grasp_ros2.core.yolo_detector import YoloRedBlockDetector


class TargetLocalizerNode(Node):
    def __init__(self):
        super().__init__("target_localizer_node")

        self.declare_parameter(
            "model_path",
            "/home/sunrise/dog/ros2_red_block_ws/src/red_block_grasp_ros2/models/red_block_yolo11n.pt",
        )
        self.declare_parameter(
            "handeye_path",
            "/home/sunrise/dog/ros2_red_block_ws/src/red_block_grasp_ros2/handeye/handeye_cam_to_eef.json",
        )
        self.declare_parameter("conf_thres", 0.35)
        self.declare_parameter("infer_imgsz", 256)
        self.declare_parameter("max_targets", 2)
        self.declare_parameter("timer_period", 0.08)
        self.declare_parameter("show_window", True)
        self.declare_parameter("display_scale", 0.75)
        self.declare_parameter("debug_overlay_level", "compact")
        self.declare_parameter("detector_device", "")
        self.declare_parameter("detector_half", False)
        self.declare_parameter("perf_log_interval_s", 3.0)

        self.declare_parameter("detector_mode", "fusion")
        self.declare_parameter("yolo_every_n_frames", 5)
        self.declare_parameter("enable_color_detector", True)
        self.declare_parameter("enable_yolo_detector", True)
        self.declare_parameter("color_calib_path", "")
        self.declare_parameter("fusion_iou_threshold", 0.10)
        self.declare_parameter("stable_frame_count", 3)
        self.declare_parameter("stable_position_threshold_mm", 20.0)
        self.declare_parameter("stable_depth_threshold_mm", 30.0)
        self.declare_parameter("publish_only_stable", False)
        self.declare_parameter("target_lock_timeout_s", 1.0)
        self.declare_parameter("fusion_min_color_to_yolo_area_ratio", 0.45)
        self.declare_parameter("fusion_edge_margin_ratio", 0.18)
        self.declare_parameter("enable_target_hold", True)
        self.declare_parameter("target_hold_max_frames", 5)
        self.declare_parameter("target_hold_timeout_s", 0.6)
        self.declare_parameter("target_hold_max_pixel_drift", 80.0)
        self.declare_parameter("target_hold_max_base_drift_mm", 80.0)
        self.declare_parameter("arm_state_source", "dog_arm_topic")
        self.declare_parameter("official_get_pose_service", "/get_pose_cmd")
        self.declare_parameter("official_pose_position_scale", 1000.0)
        self.declare_parameter("official_pose_timeout_s", 0.5)
        self.declare_parameter("official_pose_poll_period_s", 0.1)

        self.declare_parameter("color_min_depth_mm", 100.0)
        self.declare_parameter("color_max_depth_mm", 700.0)
        self.declare_parameter("color_min_area", 250.0)
        self.declare_parameter("color_min_area_ratio", 0.0)
        self.declare_parameter("color_max_area_ratio", 0.35)
        self.declare_parameter("color_aspect_min", 0.35)
        self.declare_parameter("color_aspect_max", 3.0)
        self.declare_parameter("color_extent_min", 0.25)
        self.declare_parameter("color_solidity_min", 0.50)
        self.declare_parameter("color_morph_kernel_size", 5)
        self.declare_parameter("color_erode_kernel_size", 5)
        self.declare_parameter("color_hsv_h1_min", 0)
        self.declare_parameter("color_hsv_h1_max", 10)
        self.declare_parameter("color_hsv_h2_min", 170)
        self.declare_parameter("color_hsv_h2_max", 180)
        self.declare_parameter("color_hsv_s_min", 70)
        self.declare_parameter("color_hsv_v_min", 45)
        self.declare_parameter("color_lab_a_min", 145)
        self.declare_parameter("color_bgr_r_min", 80)
        self.declare_parameter("color_bgr_rg_delta", 35)
        self.declare_parameter("color_bgr_rb_delta", 25)
        self.declare_parameter("color_bgr_b_max", 210)
        self.declare_parameter("color_min_valid_depth_count", 8)
        self.declare_parameter("color_relative_area_min_ratio", 0.25)
        self.declare_parameter("color_locked_replace_min_area_ratio", 0.30)

        self.declare_parameter("safe_roi_x_min_ratio", 0.12)
        self.declare_parameter("safe_roi_x_max_ratio", 0.88)
        self.declare_parameter("safe_roi_y_min_ratio", 0.12)
        self.declare_parameter("safe_roi_y_max_ratio", 0.88)
        self.declare_parameter("enable_target_lock", True)
        self.declare_parameter("lock_max_pixel_jump", 160.0)
        self.declare_parameter("center_weight", 0.25)
        self.declare_parameter("base_filter_alpha", 0.55)

        self.declare_parameter("save_hard_samples", False)
        self.declare_parameter("hard_sample_conf_thres", 0.75)
        self.declare_parameter("hard_sample_interval_s", 0.8)
        self.declare_parameter("force_save_samples", False)
        self.declare_parameter("force_sample_interval_s", 1.5)
        self.declare_parameter("hard_sample_dir", "/home/sunrise/dog/ros2_red_block_ws/hard_samples")

        self.model_path = self.get_parameter("model_path").value
        self.handeye_path = self.get_parameter("handeye_path").value
        self.conf_thres = float(self.get_parameter("conf_thres").value)
        self.infer_imgsz = int(self.get_parameter("infer_imgsz").value)
        self.max_targets = int(self.get_parameter("max_targets").value)
        self.timer_period = float(self.get_parameter("timer_period").value)
        self.show_window = self.parse_bool(self.get_parameter("show_window").value)
        self.display_scale = float(self.get_parameter("display_scale").value)
        self.debug_overlay_level = str(self.get_parameter("debug_overlay_level").value).strip().lower()
        if self.debug_overlay_level not in ("none", "compact", "full"):
            self.debug_overlay_level = "compact"
        self.detector_device = str(self.get_parameter("detector_device").value)
        self.detector_half = self.parse_bool(self.get_parameter("detector_half").value)
        self.perf_log_interval_s = float(self.get_parameter("perf_log_interval_s").value)

        self.detector_mode = str(self.get_parameter("detector_mode").value).strip().lower()
        if self.detector_mode not in ("yolo", "color", "fusion"):
            self.detector_mode = "fusion"
        self.yolo_every_n_frames = max(1, int(self.get_parameter("yolo_every_n_frames").value))
        self.enable_color_detector = self.parse_bool(self.get_parameter("enable_color_detector").value)
        self.enable_yolo_detector = self.parse_bool(self.get_parameter("enable_yolo_detector").value)
        self.color_calib_path = str(self.get_parameter("color_calib_path").value).strip()
        self.fusion_iou_threshold = float(self.get_parameter("fusion_iou_threshold").value)
        self.stable_frame_count = max(1, int(self.get_parameter("stable_frame_count").value))
        self.stable_position_threshold_mm = float(self.get_parameter("stable_position_threshold_mm").value)
        self.stable_depth_threshold_mm = float(self.get_parameter("stable_depth_threshold_mm").value)
        self.publish_only_stable = self.parse_bool(self.get_parameter("publish_only_stable").value)
        self.target_lock_timeout_s = float(self.get_parameter("target_lock_timeout_s").value)
        self.fusion_min_color_to_yolo_area_ratio = float(
            self.get_parameter("fusion_min_color_to_yolo_area_ratio").value
        )
        self.fusion_edge_margin_ratio = float(self.get_parameter("fusion_edge_margin_ratio").value)
        self.enable_target_hold = self.parse_bool(self.get_parameter("enable_target_hold").value)
        self.target_hold_max_frames = max(0, int(self.get_parameter("target_hold_max_frames").value))
        self.target_hold_timeout_s = float(self.get_parameter("target_hold_timeout_s").value)
        self.target_hold_max_pixel_drift = float(self.get_parameter("target_hold_max_pixel_drift").value)
        self.target_hold_max_base_drift_mm = float(self.get_parameter("target_hold_max_base_drift_mm").value)
        self.arm_state_source = str(self.get_parameter("arm_state_source").value).strip().lower()
        if self.arm_state_source not in ("dog_arm_topic", "official_get_pose_cmd", "none"):
            self.arm_state_source = "dog_arm_topic"
        self.official_get_pose_service = str(self.get_parameter("official_get_pose_service").value).strip()
        if not self.official_get_pose_service:
            self.official_get_pose_service = "/get_pose_cmd"
        self.official_pose_position_scale = float(self.get_parameter("official_pose_position_scale").value)
        self.official_pose_timeout_s = max(0.05, float(self.get_parameter("official_pose_timeout_s").value))
        self.official_pose_poll_period_s = max(0.05, float(self.get_parameter("official_pose_poll_period_s").value))

        self.safe_roi_x_min_ratio = float(self.get_parameter("safe_roi_x_min_ratio").value)
        self.safe_roi_x_max_ratio = float(self.get_parameter("safe_roi_x_max_ratio").value)
        self.safe_roi_y_min_ratio = float(self.get_parameter("safe_roi_y_min_ratio").value)
        self.safe_roi_y_max_ratio = float(self.get_parameter("safe_roi_y_max_ratio").value)
        self.enable_target_lock = self.parse_bool(self.get_parameter("enable_target_lock").value)
        self.lock_max_pixel_jump = float(self.get_parameter("lock_max_pixel_jump").value)
        self.center_weight = float(self.get_parameter("center_weight").value)
        self.base_filter_alpha = float(self.get_parameter("base_filter_alpha").value)

        self.save_hard_samples = self.parse_bool(self.get_parameter("save_hard_samples").value)
        self.hard_sample_conf_thres = float(self.get_parameter("hard_sample_conf_thres").value)
        self.hard_sample_interval_s = float(self.get_parameter("hard_sample_interval_s").value)
        self.force_save_samples = self.parse_bool(self.get_parameter("force_save_samples").value)
        self.force_sample_interval_s = float(self.get_parameter("force_sample_interval_s").value)
        self.hard_sample_dir = self.get_parameter("hard_sample_dir").value
        os.makedirs(self.hard_sample_dir, exist_ok=True)

        self.color_params = self.build_color_params()
        self.apply_color_calib_file(self.color_params)

        self.pub_target = self.create_publisher(String, "/red_block/target_base", 10)
        self.sub_arm_state = None
        if self.arm_state_source == "dog_arm_topic":
            self.sub_arm_state = self.create_subscription(String, "/roarm_m3/state", self.on_arm_state, 10)
        self.official_pose_client = None
        self.official_pose_future = None
        self.last_official_pose_request_time = 0.0
        self.last_official_pose_warn_time = 0.0
        if self.arm_state_source == "official_get_pose_cmd":
            self.official_pose_client = self.create_client(GetPoseCmd, self.official_get_pose_service)

        self.latest_arm_state = None
        self.latest_arm_state_time = 0.0
        self.latest_official_pose = None
        self.locked_pixel = None
        self.lock_last_seen_time = 0.0
        self.locked_area = None
        self.last_rejected_locked_small_area_count = 0
        self.filtered_base = None
        self.stable_window = deque(maxlen=self.stable_frame_count)
        self.frame_index = 0
        self.latest_yolo_detections = []
        self.latest_yolo_time = 0.0
        self.last_hard_sample_time = 0.0
        self.last_force_sample_time = 0.0
        self.last_valid_detection = None
        self.last_valid_target_msg = None
        self.last_valid_pixel = None
        self.last_valid_base = None
        self.last_valid_stamp = 0.0
        self.lost_frame_count = 0
        self.hold_active = False

        self.camera = OrbbecRgbdCamera()
        self.yolo_detector = None
        if self.enable_yolo_detector and self.detector_mode in ("yolo", "fusion"):
            self.yolo_detector = YoloRedBlockDetector(
                model_path=self.model_path,
                conf_thres=self.conf_thres,
                max_targets=self.max_targets,
                imgsz=self.infer_imgsz,
                device=self.detector_device,
                half=self.detector_half,
            )

        self.color_detector = None
        if self.enable_color_detector and self.detector_mode in ("color", "fusion"):
            self.color_detector = RedColorBlockDetector(
                min_depth_mm=self.color_params["color_min_depth_mm"],
                max_depth_mm=self.color_params["color_max_depth_mm"],
                min_area=self.color_params["color_min_area"],
                min_area_ratio=self.color_params["color_min_area_ratio"],
                max_area_ratio=self.color_params["color_max_area_ratio"],
                aspect_min=self.color_params["color_aspect_min"],
                aspect_max=self.color_params["color_aspect_max"],
                extent_min=self.color_params["color_extent_min"],
                solidity_min=self.color_params["color_solidity_min"],
                morph_kernel_size=self.color_params["color_morph_kernel_size"],
                erode_kernel_size=self.color_params["color_erode_kernel_size"],
                hsv_h1_min=self.color_params["color_hsv_h1_min"],
                hsv_h1_max=self.color_params["color_hsv_h1_max"],
                hsv_h2_min=self.color_params["color_hsv_h2_min"],
                hsv_h2_max=self.color_params["color_hsv_h2_max"],
                hsv_s_min=self.color_params["color_hsv_s_min"],
                hsv_v_min=self.color_params["color_hsv_v_min"],
                lab_a_min=self.color_params["color_lab_a_min"],
                bgr_r_min=self.color_params["color_bgr_r_min"],
                bgr_rg_delta=self.color_params["color_bgr_rg_delta"],
                bgr_rb_delta=self.color_params["color_bgr_rb_delta"],
                bgr_b_max=self.color_params["color_bgr_b_max"],
                min_valid_depth_count=self.color_params["color_min_valid_depth_count"],
                max_targets=self.max_targets,
            )
            self.log_color_params()

        self.localizer = TargetLocalizer(
            handeye_path=self.handeye_path,
            depth_roi_radius=6,
            min_depth_mm=100,
            max_depth_mm=700,
        )

        self.window_name = "ROS2 Target Localizer"
        self.last_time = time.time()
        self.last_perf_log_time = 0.0
        self.fps = 0.0

        self.get_logger().info("Starting RGBD camera...")
        self.camera.start()
        if self.yolo_detector is not None:
            self.get_logger().info("Loading YOLO model...")
            self.yolo_detector.load()

        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 800, 520)

        self.timer = self.create_timer(self.timer_period, self.on_timer)
        self.get_logger().info(
            f"Target localizer started. mode={self.detector_mode}, yolo_every_n_frames={self.yolo_every_n_frames}, "
            f"arm_state_source={self.arm_state_source}"
        )

    @staticmethod
    def parse_bool(value):
        if isinstance(value, bool):
            return value
        if value is None:
            return False
        text = str(value).strip().lower()
        return text in ("1", "true", "yes", "on")

    def build_color_params(self):
        return {
            "color_hsv_h1_min": int(self.get_parameter("color_hsv_h1_min").value),
            "color_hsv_h1_max": int(self.get_parameter("color_hsv_h1_max").value),
            "color_hsv_h2_min": int(self.get_parameter("color_hsv_h2_min").value),
            "color_hsv_h2_max": int(self.get_parameter("color_hsv_h2_max").value),
            "color_hsv_s_min": int(self.get_parameter("color_hsv_s_min").value),
            "color_hsv_v_min": int(self.get_parameter("color_hsv_v_min").value),
            "color_lab_a_min": int(self.get_parameter("color_lab_a_min").value),
            "color_bgr_r_min": int(self.get_parameter("color_bgr_r_min").value),
            "color_bgr_rg_delta": int(self.get_parameter("color_bgr_rg_delta").value),
            "color_bgr_rb_delta": int(self.get_parameter("color_bgr_rb_delta").value),
            "color_bgr_b_max": int(self.get_parameter("color_bgr_b_max").value),
            "color_min_valid_depth_count": int(self.get_parameter("color_min_valid_depth_count").value),
            "color_min_area": float(self.get_parameter("color_min_area").value),
            "color_min_area_ratio": float(self.get_parameter("color_min_area_ratio").value),
            "color_max_area_ratio": float(self.get_parameter("color_max_area_ratio").value),
            "color_aspect_min": float(self.get_parameter("color_aspect_min").value),
            "color_aspect_max": float(self.get_parameter("color_aspect_max").value),
            "color_extent_min": float(self.get_parameter("color_extent_min").value),
            "color_solidity_min": float(self.get_parameter("color_solidity_min").value),
            "color_morph_kernel_size": int(self.get_parameter("color_morph_kernel_size").value),
            "color_erode_kernel_size": int(self.get_parameter("color_erode_kernel_size").value),
            "color_relative_area_min_ratio": float(self.get_parameter("color_relative_area_min_ratio").value),
            "color_locked_replace_min_area_ratio": float(
                self.get_parameter("color_locked_replace_min_area_ratio").value
            ),
            "color_min_depth_mm": float(self.get_parameter("color_min_depth_mm").value),
            "color_max_depth_mm": float(self.get_parameter("color_max_depth_mm").value),
        }

    def read_calib_file(self, path):
        ext = os.path.splitext(path)[1].lower()
        with open(path, "r", encoding="utf-8") as f:
            text = f.read()

        if ext == ".json":
            return json.loads(text)

        if ext in (".yaml", ".yml"):
            try:
                import yaml
            except ImportError as exc:
                raise RuntimeError(
                    "PyYAML is required to read YAML color calibration files. "
                    "Install python3-yaml or provide a JSON file."
                ) from exc
            return yaml.safe_load(text)

        try:
            import yaml
            data = yaml.safe_load(text)
            if isinstance(data, dict):
                return data
        except Exception:
            pass
        return json.loads(text)

    def apply_color_calib_file(self, color_params):
        if not self.color_calib_path:
            return

        if not os.path.exists(self.color_calib_path):
            self.get_logger().warn(
                f"Color calibration file does not exist: {self.color_calib_path}. Using ROS/default color parameters."
            )
            return

        try:
            data = self.read_calib_file(self.color_calib_path)
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to load color calibration file {self.color_calib_path}: {exc}. "
                "Using ROS/default color parameters."
            )
            return

        if not isinstance(data, dict):
            self.get_logger().warn(
                f"Color calibration file {self.color_calib_path} did not contain a mapping. "
                "Using ROS/default color parameters."
            )
            return

        loaded_keys = []
        for key in color_params:
            if key not in data:
                continue
            try:
                if isinstance(color_params[key], int):
                    color_params[key] = int(data[key])
                else:
                    color_params[key] = float(data[key])
                loaded_keys.append(key)
            except Exception as exc:
                self.get_logger().warn(f"Ignoring invalid color calibration value for {key}: {exc}")

        self.get_logger().info(
            f"Loaded color calibration from {self.color_calib_path}. Updated keys: {', '.join(loaded_keys)}"
        )

    def log_color_params(self):
        p = self.color_params
        self.get_logger().info(
            "Color detector params: "
            f"H=[{p['color_hsv_h1_min']},{p['color_hsv_h1_max']}]|"
            f"[{p['color_hsv_h2_min']},{p['color_hsv_h2_max']}], "
            f"S_min={p['color_hsv_s_min']}, V_min={p['color_hsv_v_min']}, "
            f"Lab_a_min={p['color_lab_a_min']}, "
            f"R_min={p['color_bgr_r_min']}, "
            f"R-G>={p['color_bgr_rg_delta']}, R-B>={p['color_bgr_rb_delta']}, "
            f"B_max={p['color_bgr_b_max']}, min_area={p['color_min_area']}"
        )

    def on_arm_state(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Invalid /roarm_m3/state JSON: {e}")
            return
        if not data.get("connected", False) or not data.get("state_valid", False):
            return
        state = data.get("state", None)
        if isinstance(state, dict):
            self.latest_arm_state = state
            self.latest_arm_state_time = time.time()

    def convert_official_pose_to_arm_state(self, response):
        # 官方 /get_pose_cmd 返回 m，这里按现有手眼链路统一转换成 mm。
        scale = self.official_pose_position_scale
        arm_state = {
            "x": float(response.x) * scale,
            "y": float(response.y) * scale,
            "z": float(response.z) * scale,
            "r": float(response.roll),
            "tit": float(response.pitch),
            "b": float(response.yaw),
            "roll": float(response.roll),
            "pitch": float(response.pitch),
            "yaw": float(response.yaw),
        }
        return arm_state

    def refresh_official_arm_pose(self, now):
        if self.arm_state_source != "official_get_pose_cmd" or self.official_pose_client is None:
            return

        if self.official_pose_future is not None:
            if not self.official_pose_future.done():
                return
            future = self.official_pose_future
            self.official_pose_future = None
            exc = future.exception()
            if exc is not None:
                self.get_logger().warn(f"official_get_pose_cmd call failed: {exc}")
            else:
                response = future.result()
                if response is not None:
                    self.latest_official_pose = response
                    self.latest_arm_state = self.convert_official_pose_to_arm_state(response)
                    self.latest_arm_state_time = now

        if now - self.last_official_pose_request_time < self.official_pose_poll_period_s:
            return

        if not self.official_pose_client.service_is_ready():
            if now - self.last_official_pose_warn_time >= 2.0:
                self.last_official_pose_warn_time = now
                self.get_logger().warn(
                    "official_get_pose_cmd unavailable; please start official roarm_moveit_cmd command_control"
                )
            return

        self.last_official_pose_request_time = now
        self.official_pose_future = self.official_pose_client.call_async(GetPoseCmd.Request())

    def get_arm_state_for_localization(self, now):
        if self.arm_state_source == "none":
            return None, "arm_state_disabled"
        if self.latest_arm_state is None:
            if self.arm_state_source == "official_get_pose_cmd":
                return None, "official_pose_unavailable"
            return None, "no_arm_state"

        timeout_s = self.official_pose_timeout_s if self.arm_state_source == "official_get_pose_cmd" else 1.0
        if now - self.latest_arm_state_time > timeout_s:
            return None, "arm_state_timeout"
        return self.latest_arm_state, None

    def detection_in_safe_roi(self, det, image_width, image_height):
        cx, cy = det.center
        return (
            image_width * self.safe_roi_x_min_ratio <= cx <= image_width * self.safe_roi_x_max_ratio
            and image_height * self.safe_roi_y_min_ratio <= cy <= image_height * self.safe_roi_y_max_ratio
        )

    @staticmethod
    def pixel_distance(p1, p2):
        return math.hypot(float(p1[0]) - float(p2[0]), float(p1[1]) - float(p2[1]))

    @staticmethod
    def bbox_iou(a, b):
        ix1 = max(a.x1, b.x1)
        iy1 = max(a.y1, b.y1)
        ix2 = min(a.x2, b.x2)
        iy2 = min(a.y2, b.y2)
        iw = max(0, ix2 - ix1)
        ih = max(0, iy2 - iy1)
        inter = iw * ih
        if inter <= 0:
            return 0.0
        area_a = max(1, (a.x2 - a.x1) * (a.y2 - a.y1))
        area_b = max(1, (b.x2 - b.x1) * (b.y2 - b.y1))
        return float(inter) / float(area_a + area_b - inter)

    @staticmethod
    def bbox_area(det):
        return max(1, int(det.x2 - det.x1) * int(det.y2 - det.y1))

    def clear_target_lock(self):
        # 锁丢失后一起清理滤波状态，避免两个红块的 base 坐标混在同一窗口里。
        self.locked_pixel = None
        self.lock_last_seen_time = 0.0
        self.locked_area = None
        self.filtered_base = None
        self.stable_window.clear()

    def clear_target_hold(self):
        # 超时后彻底清空短时保持缓存，避免旧目标被长期复用。
        self.last_valid_detection = None
        self.last_valid_target_msg = None
        self.last_valid_pixel = None
        self.last_valid_base = None
        self.last_valid_stamp = 0.0
        self.lost_frame_count = 0
        self.hold_active = False

    def target_lock_active(self, now):
        return (
            self.enable_target_lock
            and self.locked_pixel is not None
            and (now - self.lock_last_seen_time) <= max(self.target_lock_timeout_s, self.target_hold_timeout_s)
        )

    @staticmethod
    def base_distance(base_a, base_b):
        if base_a is None or base_b is None:
            return None
        dx = float(base_a["x"]) - float(base_b["x"])
        dy = float(base_a["y"]) - float(base_b["y"])
        dz = float(base_a["z"]) - float(base_b["z"])
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def update_last_valid_target(self, selected, msg_dict, now):
        # 只有当前帧真正有效时才刷新缓存，短时保持不覆盖新鲜目标。
        self.last_valid_detection = selected
        self.last_valid_target_msg = json.loads(json.dumps(msg_dict))
        self.last_valid_pixel = dict(msg_dict.get("pixel", {})) if msg_dict.get("pixel") else None
        self.last_valid_base = dict(msg_dict.get("base_mm", {})) if msg_dict.get("base_mm") else None
        self.last_valid_stamp = float(now)
        self.lost_frame_count = 0
        self.hold_active = False

    def can_hold_target(self, now):
        if not self.enable_target_hold or self.last_valid_target_msg is None or self.last_valid_stamp <= 0.0:
            return False, None
        hold_age = max(0.0, float(now) - float(self.last_valid_stamp))
        if self.lost_frame_count > self.target_hold_max_frames:
            return False, hold_age
        if hold_age > self.target_hold_timeout_s:
            return False, hold_age
        return True, hold_age

    def build_hold_message(self, now, reason):
        can_hold, hold_age = self.can_hold_target(now)
        if not can_hold:
            return None
        # 保持只用于短时抗抖，沿用上一帧已验证过的目标信息。
        hold_msg = json.loads(json.dumps(self.last_valid_target_msg))
        hold_msg["stamp"] = now
        hold_msg["reason"] = reason
        hold_msg["is_hold"] = True
        hold_msg["hold_frames"] = int(self.lost_frame_count)
        hold_msg["hold_age_s"] = float(hold_age)
        return hold_msg

    def select_detection(self, detections, image_width, image_height, now=None):
        if now is None:
            now = time.time()
        self.last_rejected_locked_small_area_count = 0
        if not detections:
            if self.locked_pixel is not None and not self.target_lock_active(now):
                self.clear_target_lock()
            return None, "no_detection"

        safe = [det for det in detections if self.detection_in_safe_roi(det, image_width, image_height)]
        if not safe:
            if self.locked_pixel is not None and not self.target_lock_active(now):
                self.clear_target_lock()
            return None, "no_safe_detection"

        if self.target_lock_active(now):
            min_replace_ratio = float(self.color_params.get("color_locked_replace_min_area_ratio", 0.30))
            if self.locked_area is not None:
                for det in safe:
                    if not hasattr(det, "debug_info") or getattr(det, "debug_info") is None:
                        det.debug_info = {}
                    area = float(det.debug_info.get("area", self.bbox_area(det)))
                    ratio = area / max(float(self.locked_area), 1.0)
                    det.debug_info["locked_area"] = float(self.locked_area)
                    det.debug_info["locked_area_ratio"] = float(ratio)
                    det.debug_info["color_locked_replace_min_area_ratio"] = float(min_replace_ratio)
                    det.debug_info["rejected_locked_small_area"] = bool(
                        area < float(self.locked_area) * min_replace_ratio
                    )
                    if det.debug_info["rejected_locked_small_area"]:
                        self.last_rejected_locked_small_area_count += 1
            nearest = min(safe, key=lambda det: self.pixel_distance(det.center, self.locked_pixel))
            nearest_debug = getattr(nearest, "debug_info", {})
            nearest_area = float(nearest_debug.get("area", self.bbox_area(nearest)))
            if (
                self.locked_area is not None
                and nearest_area < float(self.locked_area) * min_replace_ratio
            ):
                if hasattr(nearest, "debug_info"):
                    nearest.debug_info["rejected_locked_small_area"] = True
                return None, "locked_target_small_area"
            if self.pixel_distance(nearest.center, self.locked_pixel) <= self.lock_max_pixel_jump:
                return nearest, "locked_target"
            return None, "locked_target_wait_timeout"
        elif self.locked_pixel is not None:
            self.clear_target_lock()

        img_cx = image_width / 2.0
        img_cy = image_height / 2.0
        norm = max(1.0, math.hypot(img_cx, img_cy))

        def score(det):
            cx, cy = det.center
            det_score = float(getattr(det, "score", getattr(det, "conf", 0.0)))
            return det_score - self.center_weight * (math.hypot(cx - img_cx, cy - img_cy) / norm)

        return max(safe, key=score), "new_target"

    def summarize_color_candidates(self, color_detections):
        if not color_detections:
            return {
                "max_candidate_area": 0.0,
                "rejected_small_area_count": 0,
                "color_candidates_filtered": [],
            }
        max_candidate_area = max(float(det.debug_info.get("area", self.bbox_area(det))) for det in color_detections)
        min_ratio = float(self.color_params.get("color_relative_area_min_ratio", 0.25))
        min_valid_depth_count = int(self.color_params.get("color_min_valid_depth_count", 8))
        min_area_allowed = max_candidate_area * min_ratio
        filtered = []
        rejected_small_area_count = 0
        for det in color_detections:
            area = float(det.debug_info.get("area", self.bbox_area(det)))
            visibility_score = 0.0 if max_candidate_area <= 1e-6 else area / max_candidate_area
            valid_depth_count = int(det.debug_info.get("valid_depth_count", 0))
            det.debug_info["max_candidate_area"] = float(max_candidate_area)
            det.debug_info["visibility_score"] = float(visibility_score)
            det.debug_info["relative_area_ratio"] = float(visibility_score)
            det.debug_info["area_ratio_to_max"] = float(visibility_score)
            det.debug_info["color_min_valid_depth_count"] = int(min_valid_depth_count)
            det.debug_info["rejected_small_area"] = bool(area < min_area_allowed)
            if area < min_area_allowed:
                rejected_small_area_count += 1
                continue
            if not bool(det.valid_depth) or valid_depth_count < min_valid_depth_count:
                det.debug_info["rejected_invalid_depth"] = True
                continue
            det.debug_info["rejected_invalid_depth"] = False
            filtered.append(det)
        return {
            "max_candidate_area": float(max_candidate_area),
            "rejected_small_area_count": int(rejected_small_area_count),
            "color_candidates_filtered": filtered,
        }

    def annotate_color_with_yolo_support(self, color_detections, yolo_detections):
        for det in color_detections:
            det.debug_info["best_yolo_iou"] = 0.0
            det.debug_info["best_yolo_area_ratio"] = None
            det.debug_info["fusion_small_local"] = False
            det.debug_info["fusion_edge_local"] = False
            if not yolo_detections:
                continue

            best_yolo = max(yolo_detections, key=lambda ydet: self.bbox_iou(det, ydet))
            best_iou = self.bbox_iou(det, best_yolo)
            det.debug_info["best_yolo_iou"] = float(best_iou)
            yolo_area = float(self.bbox_area(best_yolo))
            color_area = float(self.bbox_area(det))
            area_ratio = color_area / max(yolo_area, 1.0)
            yolo_w = max(1, best_yolo.width)
            yolo_h = max(1, best_yolo.height)
            edge_margin_x = self.fusion_edge_margin_ratio * yolo_w
            edge_margin_y = self.fusion_edge_margin_ratio * yolo_h
            cx, cy = det.center
            edge_local = (
                cx < best_yolo.x1 + edge_margin_x
                or cx > best_yolo.x2 - edge_margin_x
                or cy < best_yolo.y1 + edge_margin_y
                or cy > best_yolo.y2 - edge_margin_y
            )
            det.debug_info["best_yolo_area_ratio"] = float(area_ratio)
            det.debug_info["fusion_edge_local"] = bool(edge_local)
            det.debug_info["fusion_small_local"] = bool(
                best_iou >= self.fusion_iou_threshold
                and area_ratio < self.fusion_min_color_to_yolo_area_ratio
                and edge_local
            )

    def select_fusion_detection(self, color_detections, yolo_detections, image_width, image_height, now=None):
        if now is None:
            now = time.time()
        color_summary = self.summarize_color_candidates(color_detections)
        filtered_color = color_summary["color_candidates_filtered"]
        if self.detector_mode == "color":
            selected, reason = self.select_detection(filtered_color, image_width, image_height, now=now)
            return selected, reason, "color", color_summary

        if self.detector_mode == "yolo":
            selected, reason = self.select_detection(yolo_detections, image_width, image_height, now=now)
            return selected, reason, "yolo", color_summary

        self.annotate_color_with_yolo_support(filtered_color, yolo_detections)

        preferred_color = [det for det in filtered_color if not det.debug_info.get("fusion_small_local", False)]
        if preferred_color:
            selected, reason = self.select_detection(preferred_color, image_width, image_height, now=now)
            if selected is not None:
                return selected, "fusion_color_primary" if reason == "new_target" else reason, "color", color_summary

        if filtered_color:
            selected, reason = self.select_detection(filtered_color, image_width, image_height, now=now)
            if selected is not None and not selected.debug_info.get("fusion_small_local", False):
                return selected, "fusion_color_only" if reason == "new_target" else reason, "color", color_summary

        if yolo_detections:
            selected, reason = self.select_detection(yolo_detections, image_width, image_height, now=now)
            if selected is not None and filtered_color:
                return selected, "fusion_yolo_color_confirm" if reason == "new_target" else reason, "yolo_assisted", color_summary
        if filtered_color:
            selected, reason = self.select_detection(filtered_color, image_width, image_height, now=now)
            return selected, "fusion_color_local_only" if selected is not None else reason, "color_local", color_summary

        # fusion 模式下没有颜色确认时不盲信 YOLO，避免现场光照波动时误抓。
        return None, "fusion_no_color_confirmation", "none", color_summary

    def filter_base(self, base):
        current = {key: float(base[key]) for key in ("x", "y", "z")}
        if self.filtered_base is None:
            self.filtered_base = dict(current)
            return dict(current)
        a = self.base_filter_alpha
        self.filtered_base = {
            "x": a * current["x"] + (1.0 - a) * self.filtered_base["x"],
            "y": a * current["y"] + (1.0 - a) * self.filtered_base["y"],
            "z": a * current["z"] + (1.0 - a) * self.filtered_base["z"],
        }
        return dict(self.filtered_base)

    def update_stability(self, base_mm, depth_mm):
        self.stable_window.append(
            {
                "x": float(base_mm["x"]),
                "y": float(base_mm["y"]),
                "z": float(base_mm["z"]),
                "depth": float(depth_mm),
            }
        )
        if len(self.stable_window) < self.stable_frame_count:
            return False, len(self.stable_window), dict(base_mm)

        arr = np.array([[p["x"], p["y"], p["z"], p["depth"]] for p in self.stable_window], dtype=np.float64)
        med = np.median(arr, axis=0)
        pos_dist = np.linalg.norm(arr[:, :3] - med[:3], axis=1)
        depth_diff = np.abs(arr[:, 3] - med[3])
        stable = (
            float(np.max(pos_dist)) <= self.stable_position_threshold_mm
            and float(np.max(depth_diff)) <= self.stable_depth_threshold_mm
        )
        stable_base = {"x": float(med[0]), "y": float(med[1]), "z": float(med[2])}
        return stable, len(self.stable_window), stable_base

    def empty_message(self, now, reason, select_reason, image_width=0, image_height=0):
        return {
            "stamp": now,
            "valid": False,
            "stable": False,
            "is_hold": False,
            "hold_frames": 0,
            "hold_age_s": 0.0,
            "stable_frames": 0,
            "color_stable": False,
            "reason": reason,
            "select_reason": select_reason,
            "detector_mode": self.detector_mode,
            "source": None,
            "target_id": None,
            "confidence": None,
            "score": None,
            "pixel": None,
            "bbox": None,
            "depth_mm": None,
            "camera_mm": None,
            "raw_base_mm": None,
            "base_mm": None,
            "detections_count": 0,
            "color_candidates_count": 0,
            "yolo_candidates_count": 0,
            "max_candidate_area": 0.0,
            "rejected_small_area_count": 0,
            "rejected_locked_small_area_count": 0,
            "locked_area": None,
            "visibility_score": 0.0,
            "relative_area_ratio": 0.0,
            "depth_valid": False,
            "valid_depth_count": 0,
            "color_min_valid_depth_count": int(self.color_params.get("color_min_valid_depth_count", 8)),
            "depth_source": "none",
            "image_width": int(image_width),
            "image_height": int(image_height),
            "arm_state_source": self.arm_state_source,
            "arm_state_age_s": None,
            "official_pose_age_s": None,
        }

    def publish_json(self, data):
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        self.pub_target.publish(msg)

    def on_timer(self):
        frame_start = time.time()
        bgr, depth_mm, camera_matrix = self.camera.read(timeout_ms=100)
        if bgr is None or depth_mm is None or camera_matrix is None:
            return
        read_done = time.time()

        now = time.time()
        self.refresh_official_arm_pose(now)
        dt = now - self.last_time
        self.last_time = now
        if dt > 1e-6:
            self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)

        self.frame_index += 1
        image_height, image_width = bgr.shape[:2]

        color_detections = []
        if self.color_detector is not None:
            color_detections = self.color_detector.detect(bgr, depth_mm=depth_mm, camera_matrix=camera_matrix)
        color_done = time.time()

        yolo_detections = self.latest_yolo_detections
        run_yolo = self.yolo_detector is not None and (
            self.detector_mode == "yolo" or self.frame_index % self.yolo_every_n_frames == 1
        )
        if run_yolo:
            yolo_detections = self.yolo_detector.detect(bgr)
            self.latest_yolo_detections = yolo_detections
            self.latest_yolo_time = now
        infer_done = time.time()

        selected, select_reason, source, color_summary = self.select_fusion_detection(
            color_detections, yolo_detections, image_width, image_height, now=now
        )

        msg_dict = self.empty_message(now, "", select_reason, image_width, image_height)
        msg_dict.update(
            {
                "source": source,
                "detections_count": len(color_detections) + len(yolo_detections),
                "color_candidates_count": len(color_detections),
                "yolo_candidates_count": len(yolo_detections),
                "max_candidate_area": float(color_summary.get("max_candidate_area", 0.0)),
                "rejected_small_area_count": int(color_summary.get("rejected_small_area_count", 0)),
                "rejected_locked_small_area_count": int(self.last_rejected_locked_small_area_count),
                "locked_area": float(self.locked_area) if self.locked_area is not None else None,
            }
        )
        if self.latest_arm_state is not None:
            msg_dict["arm_state_age_s"] = now - self.latest_arm_state_time
        if self.arm_state_source == "official_get_pose_cmd" and self.latest_arm_state_time > 0.0:
            msg_dict["official_pose_age_s"] = now - self.latest_arm_state_time

        hold_reason = None
        current_arm_state, arm_state_reason = self.get_arm_state_for_localization(now)
        if selected is None:
            msg_dict["reason"] = select_reason
            self.stable_window.clear()
            self.lost_frame_count += 1
            hold_reason = select_reason
        elif current_arm_state is None:
            self.locked_pixel = selected.center
            self.lock_last_seen_time = now
            msg_dict["reason"] = arm_state_reason
            self.stable_window.clear()
            self.lost_frame_count += 1
            hold_reason = arm_state_reason
        else:
            self.locked_pixel = selected.center
            self.lock_last_seen_time = now
            location = self.localizer.localize(selected, bgr, depth_mm, camera_matrix, current_arm_state)
            if location is None:
                msg_dict["reason"] = "invalid_depth"
                self.stable_window.clear()
                self.lost_frame_count += 1
                hold_reason = "invalid_depth"
            else:
                if self.hold_active and self.last_valid_pixel is not None:
                    pixel_jump = self.pixel_distance(
                        selected.center,
                        (self.last_valid_pixel["x"], self.last_valid_pixel["y"]),
                    )
                    if pixel_jump > self.target_hold_max_pixel_drift:
                        msg_dict["reason"] = "hold_reject_far_pixel"
                        self.stable_window.clear()
                        self.lost_frame_count += 1
                        hold_reason = "hold_reject_far_pixel"
                        location = None

                if location is not None and self.hold_active and self.last_valid_base is not None:
                    base_jump = self.base_distance(location["base_mm"], self.last_valid_base)
                    if base_jump is not None and base_jump > self.target_hold_max_base_drift_mm:
                        msg_dict["reason"] = "hold_reject_far_base"
                        self.stable_window.clear()
                        self.lost_frame_count += 1
                        hold_reason = "hold_reject_far_base"
                        location = None

            if location is not None:
                raw_base = location["base_mm"]
                filtered_base = self.filter_base(raw_base)
                stable, stable_frames, stable_base = self.update_stability(filtered_base, location["depth_mm"])
                output_base = stable_base if stable else filtered_base
                msg_dict.update(
                    {
                        "valid": True,
                        "stable": bool(stable),
                        "color_stable": bool(stable),
                        "stable_frames": int(stable_frames),
                        "reason": "ok",
                        "target_id": 0,
                        "confidence": float(getattr(selected, "conf", 0.0)),
                        "score": float(getattr(selected, "score", getattr(selected, "conf", 0.0))),
                        "pixel": location["pixel"],
                        "bbox": {
                            "x1": int(selected.x1),
                            "y1": int(selected.y1),
                            "x2": int(selected.x2),
                            "y2": int(selected.y2),
                            "w": int(selected.width),
                            "h": int(selected.height),
                        },
                        "depth_mm": location["depth_mm"],
                        "camera_mm": location["camera_mm"],
                        "raw_base_mm": raw_base,
                        "base_mm": output_base,
                    }
                )
                msg_dict["visibility_score"] = float(selected.debug_info.get("visibility_score", 0.0))
                msg_dict["relative_area_ratio"] = float(selected.debug_info.get("relative_area_ratio", 0.0))
                msg_dict["depth_valid"] = bool(selected.valid_depth)
                msg_dict["valid_depth_count"] = int(selected.debug_info.get("valid_depth_count", 0))
                msg_dict["color_min_valid_depth_count"] = int(
                    selected.debug_info.get("color_min_valid_depth_count", self.color_params["color_min_valid_depth_count"])
                )
                msg_dict["depth_source"] = str(selected.debug_info.get("depth_source", "none"))
                self.locked_area = float(selected.debug_info.get("area", self.bbox_area(selected)))
                self.update_last_valid_target(selected, msg_dict, now)

        publish_dict = msg_dict
        active_selected = selected
        if not msg_dict.get("valid", False):
            hold_msg = self.build_hold_message(now, hold_reason or msg_dict.get("reason", "no_detection"))
            if hold_msg is not None:
                publish_dict = hold_msg
                active_selected = self.last_valid_detection
                self.hold_active = True
            else:
                self.hold_active = False
                can_hold, _ = self.can_hold_target(now)
                if not can_hold:
                    self.clear_target_hold()
                    self.clear_target_lock()
        else:
            self.hold_active = False

        if not self.publish_only_stable or publish_dict.get("stable", False) or not publish_dict.get("valid", False):
            self.publish_json(publish_dict)
        publish_done = time.time()

        self.maybe_save_samples(bgr, active_selected, publish_dict)
        display_done = publish_done
        if self.show_window:
            display = self.draw_debug(bgr, color_detections, yolo_detections, active_selected, publish_dict)
            if 0.05 < self.display_scale < 1.0:
                display = cv2.resize(display, None, fx=self.display_scale, fy=self.display_scale, interpolation=cv2.INTER_AREA)
            cv2.imshow(self.window_name, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                rclpy.shutdown()
            display_done = time.time()

        if self.perf_log_interval_s > 0 and now - self.last_perf_log_time >= self.perf_log_interval_s:
            self.last_perf_log_time = now
            self.get_logger().info(
                "localizer-perf: "
                f"fps={self.fps:.1f} read_ms={(read_done - frame_start) * 1000.0:.1f} "
                f"color_ms={(color_done - read_done) * 1000.0:.1f} "
                f"yolo_ms={(infer_done - color_done) * 1000.0:.1f} "
                f"publish_ms={(publish_done - infer_done) * 1000.0:.1f} "
                f"display_ms={(display_done - publish_done) * 1000.0:.1f} mode={self.detector_mode}"
            )

    def save_sample_image(self, bgr, selected, msg_dict, tag):
        stamp_ms = int(time.time() * 1000)
        safe_tag = str(tag).replace("/", "_").replace(" ", "_")
        image_path = os.path.join(self.hard_sample_dir, f"sample_{stamp_ms}_{safe_tag}.jpg")
        json_path = os.path.join(self.hard_sample_dir, f"sample_{stamp_ms}_{safe_tag}.json")
        image = bgr.copy()
        if selected is not None:
            cv2.rectangle(image, (selected.x1, selected.y1), (selected.x2, selected.y2), (0, 255, 255), 2)
            cv2.circle(image, selected.center, 5, (0, 255, 255), -1)
        ok = cv2.imwrite(image_path, image)
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(msg_dict, f, ensure_ascii=False, indent=2)
        if ok:
            self.get_logger().info(f"Saved sample: {image_path}")

    def maybe_save_samples(self, bgr, selected, msg_dict):
        now = time.time()
        if self.force_save_samples and now - self.last_force_sample_time >= self.force_sample_interval_s:
            self.save_sample_image(bgr, selected, msg_dict, "force")
            self.last_force_sample_time = now
            return
        if not self.save_hard_samples or now - self.last_hard_sample_time < self.hard_sample_interval_s:
            return
        if not msg_dict.get("valid", False):
            self.save_sample_image(bgr, selected, msg_dict, str(msg_dict.get("reason", "invalid")))
            self.last_hard_sample_time = now
            return
        conf = float(msg_dict.get("confidence", 0.0))
        if conf < self.hard_sample_conf_thres:
            self.save_sample_image(bgr, selected, msg_dict, f"lowconf_{conf:.2f}")
            self.last_hard_sample_time = now

    def draw_overlay_lines(self, display, lines, start_y=35, color=(0, 255, 255)):
        for idx, line in enumerate(lines):
            cv2.putText(
                display,
                line,
                (20, start_y + idx * 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                color,
                2,
            )

    def compact_overlay_lines(self, msg_dict):
        valid = bool(msg_dict.get("valid", False))
        reason = str(msg_dict.get("reason", ""))
        source = str(msg_dict.get("source", ""))
        confidence = float(msg_dict.get("confidence", 0.0) or 0.0)
        pixel = msg_dict.get("pixel") or {}
        pixel_x = int(pixel.get("x", -1)) if isinstance(pixel, dict) else -1
        pixel_y = int(pixel.get("y", -1)) if isinstance(pixel, dict) else -1
        depth_mm = msg_dict.get("depth_mm", None)
        depth_text = "n/a" if depth_mm is None else f"{float(depth_mm):.1f}mm"
        base = msg_dict.get("base_mm") or {}
        if isinstance(base, dict) and {"x", "y", "z"} <= set(base.keys()):
            base_text = f"x={float(base['x']):.1f} y={float(base['y']):.1f} z={float(base['z']):.1f}"
        else:
            base_text = "n/a"
        lock_text = "ON" if self.target_lock_active(time.time()) else "OFF"
        hold_text = "ON" if msg_dict.get("is_hold", False) else "OFF"
        return [
            f"valid: {valid} reason: {reason}",
            f"src: {source} conf: {confidence:.2f}",
            f"pixel: ({pixel_x},{pixel_y}) depth: {depth_text}",
            f"base_mm: {base_text}",
            f"lock: {lock_text} hold: {hold_text}",
            f"fps: {self.fps:.1f}",
        ]

    def full_overlay_lines(self, msg_dict):
        lock_active = self.target_lock_active(time.time())
        lock_age = 0.0 if self.locked_pixel is None else max(0.0, time.time() - self.lock_last_seen_time)
        locked_pixel_text = "none" if self.locked_pixel is None else f"({self.locked_pixel[0]},{self.locked_pixel[1]})"
        return [
            f"Target Localizer | {self.detector_mode} | FPS={self.fps:.1f}",
            f"LOCKED: {'yes' if lock_active else 'no'}",
            f"HOLD: {'yes' if msg_dict.get('is_hold', False) else 'no'}",
            f"lost_frames: {self.lost_frame_count}",
            f"hold_age: {float(msg_dict.get('hold_age_s', 0.0)):.2f}s",
            f"hold_timeout: {self.target_hold_timeout_s:.2f}s",
            f"lock_age: {lock_age:.2f}s",
            f"locked_pixel: {locked_pixel_text}",
            f"selected_source: {msg_dict.get('source')}",
            f"detector_mode: {self.detector_mode}",
            f"arm_state_source: {self.arm_state_source}",
            f"official_pose_age_s: {float(msg_dict.get('official_pose_age_s', 0.0) or 0.0):.2f}",
            f"color_stable: {msg_dict.get('color_stable', False)}",
        ]

    def draw_debug(self, image, color_detections, yolo_detections, selected, msg_dict):
        display = image.copy()
        image_height, image_width = image.shape[:2]
        x_min = int(image_width * self.safe_roi_x_min_ratio)
        x_max = int(image_width * self.safe_roi_x_max_ratio)
        y_min = int(image_height * self.safe_roi_y_min_ratio)
        y_max = int(image_height * self.safe_roi_y_max_ratio)
        cv2.rectangle(display, (x_min, y_min), (x_max, y_max), (255, 255, 0), 2)

        overlay_level = self.debug_overlay_level
        show_text = overlay_level != "none"
        full_overlay = overlay_level == "full"

        for det in yolo_detections:
            cv2.rectangle(display, (det.x1, det.y1), (det.x2, det.y2), (255, 0, 0), 2)
            if full_overlay:
                label = f"YOLO conf={float(getattr(det, 'conf', 0.0)):.2f}"
                cv2.putText(
                    display,
                    label,
                    (det.x1, max(18, det.y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (255, 0, 0),
                    2,
                )

        for det in color_detections:
            is_selected = selected is det
            color = (0, 255, 255) if is_selected else (0, 0, 255)
            cv2.rectangle(display, (det.x1, det.y1), (det.x2, det.y2), color, 3 if is_selected else 2)
            cv2.circle(display, det.center, 5, color, -1)
            if full_overlay:
                det_area = float(det.debug_info.get("area", self.bbox_area(det)))
                extent = float(det.debug_info.get("extent", 0.0))
                solidity = float(det.debug_info.get("solidity", 0.0))
                label = (
                    f"COLOR score={float(getattr(det, 'score', getattr(det, 'conf', 0.0))):.2f} "
                    f"area={det_area:.0f} extent={extent:.2f} solidity={solidity:.2f}"
                )
                if det.debug_info.get("fusion_small_local", False):
                    label += " local"
                if det.debug_info.get("rejected_small_area", False):
                    label += " small"
                cv2.putText(
                    display,
                    label,
                    (det.x1, min(image_height - 8, det.y2 + 16 if det.y1 < 24 else det.y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    color,
                    2,
                )

        if selected is not None and all(selected is not det for det in color_detections):
            cv2.rectangle(display, (selected.x1, selected.y1), (selected.x2, selected.y2), (0, 255, 255), 3)
            cv2.circle(display, selected.center, 5, (0, 255, 255), -1)
            if full_overlay:
                cv2.putText(
                    display,
                    "HOLD TARGET" if msg_dict.get("is_hold", False) else "SELECTED",
                    (selected.x1, max(18, selected.y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.50,
                    (0, 255, 255),
                    2,
                )
        elif selected is not None and full_overlay:
            cv2.putText(
                display,
                "HOLD TARGET" if msg_dict.get("is_hold", False) else "SELECTED",
                (selected.x1, max(18, selected.y1 - 24)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (0, 255, 255),
                2,
            )

        if not show_text:
            return display

        if overlay_level == "compact":
            self.draw_overlay_lines(display, self.compact_overlay_lines(msg_dict))
            return display

        self.draw_overlay_lines(display, self.full_overlay_lines(msg_dict))
        if msg_dict.get("valid", False):
            base = msg_dict["base_mm"]
            text = (
                f"{msg_dict.get('source')} stable={msg_dict.get('stable')} "
                f"base=({base['x']:.1f},{base['y']:.1f},{base['z']:.1f})"
            )
            if msg_dict.get("is_hold", False):
                text += " HOLD TARGET"
            cv2.putText(display, text, (20, 195), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 255, 255), 2)
            if selected is not None:
                dbg = getattr(selected, "debug_info", {})
                selected_lines = [
                    f"selected_area: {float(dbg.get('area', self.bbox_area(selected))):.0f}",
                    f"max_candidate_area: {float(msg_dict.get('max_candidate_area', 0.0)):.0f}",
                    f"relative_area_ratio: {float(msg_dict.get('relative_area_ratio', 0.0)):.2f}",
                    f"visibility_score: {float(msg_dict.get('visibility_score', 0.0)):.2f}",
                    f"depth_valid: {msg_dict.get('depth_valid', False)}",
                    f"valid_depth_count: {int(msg_dict.get('valid_depth_count', 0))}",
                    f"min_valid_depth_count: {int(msg_dict.get('color_min_valid_depth_count', 0))}",
                    f"rejected_small_area_count: {int(msg_dict.get('rejected_small_area_count', 0))}",
                    f"rejected_locked_small_area_count: {int(msg_dict.get('rejected_locked_small_area_count', 0))}",
                    f"locked_area: {float(msg_dict.get('locked_area', 0.0) or 0.0):.0f}",
                    f"selected_score: {float(getattr(selected, 'score', getattr(selected, 'conf', 0.0))):.2f}",
                    f"selected_depth: {float(msg_dict.get('depth_mm', 0.0)):.1f}",
                    f"depth_source: {msg_dict.get('depth_source', dbg.get('depth_source', 'window'))}",
                    f"color_mean_r: {float(dbg.get('mean_r', 0.0)):.1f}",
                    f"color_rg_delta: {float(dbg.get('mean_rg_delta', 0.0)):.1f}",
                    f"color_rb_delta: {float(dbg.get('mean_rb_delta', 0.0)):.1f}",
                ]
                self.draw_overlay_lines(display, selected_lines, start_y=220)
        else:
            cv2.putText(
                display,
                f"invalid: {msg_dict.get('reason')} / {msg_dict.get('select_reason')}",
                (20, 195),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 0, 255),
                2,
            )
        return display

    def destroy_node(self):
        try:
            self.camera.stop()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TargetLocalizerNode()
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
