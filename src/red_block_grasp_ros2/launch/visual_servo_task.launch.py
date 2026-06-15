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
    enable_pick_place_sequence = LaunchConfiguration("enable_pick_place_sequence")
    servo_min_z_mm = LaunchConfiguration("servo_min_z_mm")
    enable_execution_logger = LaunchConfiguration("enable_execution_logger")
    infer_imgsz = LaunchConfiguration("infer_imgsz")
    target_timer_period = LaunchConfiguration("target_timer_period")
    move_speed = LaunchConfiguration("move_speed")
    step_wait_s = LaunchConfiguration("step_wait_s")

    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path",
            default_value="/home/sunrise/dog/ros2_red_block_ws/src/red_block_grasp_ros2/models/red_block_yolo11n_v3_mix.pt",
        ),
        DeclareLaunchArgument(
            "handeye_path",
            default_value="/home/sunrise/dog/ros2_red_block_ws/src/red_block_grasp_ros2/handeye/handeye_cam_to_eef.json",
        ),
        DeclareLaunchArgument(
            "arm_port",
            default_value="/dev/ttyUSB0",
        ),
        DeclareLaunchArgument(
            "show_window",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "enable_fill_light",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "enable_pick_place_sequence",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "servo_min_z_mm",
            default_value="110.0",
        ),
        DeclareLaunchArgument(
            "enable_execution_logger",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "infer_imgsz",
            default_value="256",
        ),
        DeclareLaunchArgument(
            "target_timer_period",
            default_value="0.08",
        ),
        DeclareLaunchArgument(
            "move_speed",
            default_value="0.12",
        ),
        DeclareLaunchArgument(
            "step_wait_s",
            default_value="0.8",
        ),

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
                    "max_targets": 2,
                    "timer_period": target_timer_period,
                    "show_window": show_window,
                    "display_scale": 0.65,
                    "perf_log_interval_s": 3.0,

                    "safe_roi_x_min_ratio": 0.12,
                    "safe_roi_x_max_ratio": 0.88,
                    "safe_roi_y_min_ratio": 0.12,
                    "safe_roi_y_max_ratio": 0.88,
                    "enable_target_lock": True,
                    "lock_max_pixel_jump": 160.0,
                    "center_weight": 0.25,
                    "base_filter_alpha": 0.55,

                    "save_hard_samples": False,
                    "hard_sample_conf_thres": 0.75,
                    "hard_sample_interval_s": 0.8,
                    "force_save_samples": False,
                    "force_sample_interval_s": 1.5,
                    "hard_sample_dir": "/home/sunrise/dog/ros2_red_block_ws/hard_samples",

                }
            ],
        ),

        Node(
            package="red_block_grasp_ros2",
            executable="visual_servo_task_node",
            name="visual_servo_task_node",
            output="screen",
            parameters=[
                {
                    "auto_start": True,
                    "move_once_only": True,

                    "init_b_deg": 0.0,
                    "init_s_deg": 0.0,
                    "init_e_deg": 70.0,
                    "init_t_deg": 90.0,
                    "init_r_deg": -90.0,
                    "init_speed_deg_s": 35.0,
                    "init_acc": 35.0,
                    "initial_wait_s": 3.0,

                    "enable_b_scan": True,
                    "b_scan_offsets": "0,-8,8,-16,16,-24,24",
                    "scan_timeout_s": 8.0,
                    "after_scan_pose_wait_s": 2.5,

                    "target_timeout_s": 1.0,
                    "max_step_mm": 25.0,
                    "edge_step_mm": 12.0,
                    "move_speed": move_speed,
                    "step_wait_s": step_wait_s,
                    "adaptive_step_enabled": True,
                    "adaptive_step_min_mm": 5.0,
                    "adaptive_step_max_mm": 25.0,
                    "adaptive_edge_step_min_mm": 5.0,
                    "adaptive_center_good_ratio": 0.25,
                    "adaptive_center_bad_ratio": 0.55,

                    "enable_e_pixel_servo": False,

                    "target_xy_tolerance_mm": 25.0,
                    "target_z_tolerance_mm": 25.0,

                    "safe_above_offset_mm": 120.0,
                    "grasp_offset_x_mm": 0.0,
                    "grasp_offset_y_mm": 0.0,
                    "grasp_offset_z_mm": 0.0,
                    "min_safe_z_mm": 30.0,
                    "servo_min_z_mm": servo_min_z_mm,
                    "servo_recover_z_margin_mm": 30.0,
                    "servo_recover_max_attempts": 3,

                    "enable_center_first_servo": True,
                    "center_pixel_u": 320.0,
                    "center_pixel_v": 240.0,
                    "center_pixel_deadband_u": 70.0,
                    "center_pixel_deadband_v": 50.0,
                    "center_stable_frames_required": 2,
                    "center_b_kp_deg_per_px": 0.018,
                    "center_b_max_step_deg": 2.0,
                    "center_e_kp_deg_per_px": 0.020,
                    "center_e_max_step_deg": 2.0,
                    "center_b_pixel_sign": 1.0,
                    "center_e_pixel_sign": -1.0,
                    "center_step_wait_s": 1.0,
                    "approach_hold_z_mm": 140.0,

                    "image_width": 640,
                    "image_height": 480,
                    "safe_roi_x_min_ratio": 0.20,
                    "safe_roi_x_max_ratio": 0.80,
                    "safe_roi_y_min_ratio": 0.20,
                    "safe_roi_y_max_ratio": 0.80,

                    "base_x_min": 80.0,
                    "base_x_max": 700.0,
                    "base_y_min": -450.0,
                    "base_y_max": 450.0,
                    "base_z_min": -30.0,
                    "base_z_max": 380.0,

                    "descend_test_mm": 30.0,
                    "descend_step_mm": 5.0,
                    "descend_control_mode": "pixel",
                    "descend_lock_xy": True,
                    "descend_xy_step_mm": 0.0,
                    "descend_x_comp_mm_per_mm": 0.0,
                    "descend_y_comp_mm_per_mm": 0.0,
                    "descend_desired_pixel_u": 320.0,
                    "descend_desired_pixel_v": 240.0,
                    "descend_pixel_deadband": 35.0,
                    "descend_pixel_kp_mm_per_px": 0.04,
                    "descend_pixel_max_xy_step_mm": 6.0,
                    "descend_pixel_x_sign": 1.0,
                    "descend_pixel_y_sign": 1.0,
                    "descend_min_z_mm": 30.0,
                    "descend_min_confidence": 0.60,
                    "descend_speed": 0.06,
                    "descend_wait_s": 2.0,
                    "descend_step_wait_s": 1.0,

                    "enable_pick_place_sequence": enable_pick_place_sequence,
                    "gripper_close_deg": 55.0,
                    "gripper_open_deg": 110.0,
                    "gripper_speed_deg_s": 25.0,
                    "gripper_acc": 25.0,
                    "gripper_wait_s": 1.0,
                    "lift_up_mm": 80.0,
                    "lift_speed": 0.08,
                    "lift_wait_s": 2.0,
                    "place_x_mm": 260.0,
                    "place_y_mm": 180.0,
                    "place_z_mm": 120.0,
                    "place_speed": 0.10,
                    "place_wait_s": 2.0,
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
    ])
