#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path = LaunchConfiguration("model_path")
    handeye_path = LaunchConfiguration("handeye_path")
    arm_port = LaunchConfiguration("arm_port")
    show_window = LaunchConfiguration("show_window")
    enable_fill_light = LaunchConfiguration("enable_fill_light")
    detector_mode = LaunchConfiguration("detector_mode")
    yolo_every_n_frames = LaunchConfiguration("yolo_every_n_frames")
    color_calib_path = LaunchConfiguration("color_calib_path")
    enable_execution_logger = LaunchConfiguration("enable_execution_logger")
    infer_imgsz = LaunchConfiguration("infer_imgsz")
    target_timer_period = LaunchConfiguration("target_timer_period")
    enable_target_lock = LaunchConfiguration("enable_target_lock")
    lock_max_pixel_jump = LaunchConfiguration("lock_max_pixel_jump")
    color_morph_kernel_size = LaunchConfiguration("color_morph_kernel_size")
    color_erode_kernel_size = LaunchConfiguration("color_erode_kernel_size")
    color_min_area = LaunchConfiguration("color_min_area")
    color_max_area_ratio = LaunchConfiguration("color_max_area_ratio")
    color_aspect_min = LaunchConfiguration("color_aspect_min")
    color_aspect_max = LaunchConfiguration("color_aspect_max")
    color_extent_min = LaunchConfiguration("color_extent_min")
    color_solidity_min = LaunchConfiguration("color_solidity_min")
    enable_target_hold = LaunchConfiguration("enable_target_hold")
    target_hold_max_frames = LaunchConfiguration("target_hold_max_frames")
    target_hold_timeout_s = LaunchConfiguration("target_hold_timeout_s")
    target_hold_max_pixel_drift = LaunchConfiguration("target_hold_max_pixel_drift")
    target_hold_max_base_drift_mm = LaunchConfiguration("target_hold_max_base_drift_mm")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value="/home/sunrise/dog/ros2_red_block_ws/src/red_block_grasp_ros2/models/red_block_yolo11n_v3_mix.pt",
            ),
            DeclareLaunchArgument(
                "handeye_path",
                default_value="/home/sunrise/dog/ros2_red_block_ws/src/red_block_grasp_ros2/handeye/handeye_cam_to_eef.json",
            ),
            DeclareLaunchArgument("arm_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("show_window", default_value="false"),
            DeclareLaunchArgument("enable_fill_light", default_value="false"),
            DeclareLaunchArgument("detector_mode", default_value="fusion"),
            DeclareLaunchArgument("yolo_every_n_frames", default_value="5"),
            DeclareLaunchArgument(
                "color_calib_path",
                default_value="/home/sunrise/dog/ros2_red_block_ws/src/red_block_grasp_ros2/config/red_color_calib.yaml",
            ),
            DeclareLaunchArgument("enable_execution_logger", default_value="true"),
            DeclareLaunchArgument("infer_imgsz", default_value="256"),
            DeclareLaunchArgument("target_timer_period", default_value="0.08"),
            DeclareLaunchArgument("enable_target_lock", default_value="true"),
            DeclareLaunchArgument("lock_max_pixel_jump", default_value="160.0"),
            DeclareLaunchArgument("color_morph_kernel_size", default_value="5"),
            DeclareLaunchArgument("color_erode_kernel_size", default_value="5"),
            DeclareLaunchArgument("color_min_area", default_value="250.0"),
            DeclareLaunchArgument("color_max_area_ratio", default_value="0.35"),
            DeclareLaunchArgument("color_aspect_min", default_value="0.35"),
            DeclareLaunchArgument("color_aspect_max", default_value="3.0"),
            DeclareLaunchArgument("color_extent_min", default_value="0.25"),
            DeclareLaunchArgument("color_solidity_min", default_value="0.50"),
            DeclareLaunchArgument("enable_target_hold", default_value="true"),
            DeclareLaunchArgument("target_hold_max_frames", default_value="5"),
            DeclareLaunchArgument("target_hold_timeout_s", default_value="0.6"),
            DeclareLaunchArgument("target_hold_max_pixel_drift", default_value="80.0"),
            DeclareLaunchArgument("target_hold_max_base_drift_mm", default_value="80.0"),
            Node(
                package="red_block_grasp_ros2",
                executable="roarm_driver_node",
                name="roarm_driver_node",
                output="screen",
                parameters=[
                    {
                        "port": arm_port,
                        "state_period": 0.2,
                        "auto_connect": True,
                        "enable_fill_light": enable_fill_light,
                    }
                ],
            ),
            Node(
                package="red_block_grasp_ros2",
                executable="target_localizer_node",
                name="target_localizer_node",
                output="screen",
                parameters=[
                    {
                        "model_path": model_path,
                        "handeye_path": handeye_path,
                        "conf_thres": 0.25,
                        "infer_imgsz": infer_imgsz,
                        "max_targets": 4,
                        "timer_period": target_timer_period,
                        "show_window": show_window,
                        "display_scale": 0.65,
                        "perf_log_interval_s": 3.0,
                        "detector_mode": detector_mode,
                        "yolo_every_n_frames": yolo_every_n_frames,
                        "color_calib_path": color_calib_path,
                        "enable_color_detector": True,
                        "enable_yolo_detector": True,
                        "fusion_iou_threshold": 0.10,
                        "stable_frame_count": 3,
                        "stable_position_threshold_mm": 20.0,
                        "stable_depth_threshold_mm": 30.0,
                        "publish_only_stable": False,
                        "color_min_depth_mm": 100.0,
                        "color_max_depth_mm": 700.0,
                        "color_min_area": color_min_area,
                        "color_max_area_ratio": color_max_area_ratio,
                        "color_aspect_min": color_aspect_min,
                        "color_aspect_max": color_aspect_max,
                        "color_extent_min": color_extent_min,
                        "color_solidity_min": color_solidity_min,
                        "color_morph_kernel_size": color_morph_kernel_size,
                        "color_erode_kernel_size": color_erode_kernel_size,
                        "safe_roi_x_min_ratio": 0.12,
                        "safe_roi_x_max_ratio": 0.88,
                        "safe_roi_y_min_ratio": 0.12,
                        "safe_roi_y_max_ratio": 0.88,
                        "enable_target_lock": enable_target_lock,
                        "lock_max_pixel_jump": lock_max_pixel_jump,
                        "enable_target_hold": enable_target_hold,
                        "target_hold_max_frames": target_hold_max_frames,
                        "target_hold_timeout_s": target_hold_timeout_s,
                        "target_hold_max_pixel_drift": target_hold_max_pixel_drift,
                        "target_hold_max_base_drift_mm": target_hold_max_base_drift_mm,
                        "center_weight": 0.25,
                        "base_filter_alpha": 0.55,
                    }
                ],
            ),
            Node(
                package="red_block_grasp_ros2",
                executable="open_loop_grasp_task_node",
                name="open_loop_grasp_task_node",
                output="screen",
                parameters=[
                    {
                        "auto_start": True,
                        "loop_hz": 5.0,
                        "target_timeout_s": 0.8,
                        "stable_frame_count": 3,
                        "stable_position_threshold_mm": 20.0,
                        "stable_depth_threshold_mm": 30.0,
                        "pre_grasp_z_offset_mm": 120.0,
                        "grasp_offset_x_mm": 0.0,
                        "grasp_offset_y_mm": 0.0,
                        "grasp_offset_z_mm": 0.0,
                        "lift_up_mm": 80.0,
                        "fast_move_speed": 0.18,
                        "grasp_move_speed": 0.08,
                        "lift_move_speed": 0.10,
                        "retreat_move_speed": 0.15,
                        "fast_max_step_mm": 120.0,
                        "max_pre_grasp_segments": 3,
                        "motion_timeout_s": 8.0,
                        "motion_wait_s": 1.5,
                        "motion_min_wait_s": 0.8,
                        "position_reached_tolerance_mm": 25.0,
                        "settle_time_s": 0.3,
                        "max_retarget_drift_mm": 60.0,
                        "final_correction_enabled": True,
                        "max_final_corrections": 2,
                        "final_correction_step_mm": 20.0,
                        "gripper_close_deg": 55.0,
                        "gripper_open_deg": 110.0,
                        "gripper_speed_deg_s": 25.0,
                        "gripper_acc": 25.0,
                        "gripper_wait_s": 1.0,
                        "safe_x_mm": 260.0,
                        "safe_y_mm": 0.0,
                        "safe_z_mm": 180.0,
                        "workspace_x_min": 80.0,
                        "workspace_x_max": 700.0,
                        "workspace_y_min": -450.0,
                        "workspace_y_max": 450.0,
                        "workspace_z_min": 20.0,
                        "workspace_z_max": 380.0,
                    }
                ],
            ),
            Node(
                package="red_block_grasp_ros2",
                executable="execution_logger_node",
                name="execution_logger_node",
                output="screen",
                condition=IfCondition(enable_execution_logger),
                parameters=[
                    {
                        "record_dir": "/home/sunrise/dog/ros2_red_block_ws/run_records",
                        "flush_interval_s": 1.0,
                    }
                ],
            ),
        ]
    )
