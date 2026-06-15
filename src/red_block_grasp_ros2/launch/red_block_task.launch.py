#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path = LaunchConfiguration("model_path")
    handeye_path = LaunchConfiguration("handeye_path")
    arm_port = LaunchConfiguration("arm_port")
    show_window = LaunchConfiguration("show_window")
    enable_fill_light = LaunchConfiguration("enable_fill_light")

    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path",
            default_value="/home/sunrise/dog/ros2_red_block_ws/src/red_block_grasp_ros2/models/red_block_yolo11n.pt",
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
            default_value="true",
        ),
        DeclareLaunchArgument(
            "enable_fill_light",
            default_value="false",
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
                    "conf_thres": 0.35,
                    "max_targets": 4,
                    "timer_period": 0.3,
                    "show_window": show_window,
                }
            ],
        ),

        Node(
            package="red_block_grasp_ros2",
            executable="task_manager_node",
            name="task_manager_node",
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

                    "enable_b_scan": True,
                    "b_scan_offsets": "0,-8,8,-16,16,-24,24",
                    "scan_timeout_s": 8.0,
                    "after_scan_pose_wait_s": 2.5,

                    "stable_count_required": 3,
                    "stable_position_tol_mm": 25.0,
                    "target_timeout_s": 1.0,

                    "safe_above_offset_mm": 120.0,
                    "grasp_offset_x_mm": 0.0,
                    "grasp_offset_y_mm": 0.0,
                    "grasp_offset_z_mm": 0.0,
                    "min_safe_z_mm": 30.0,
                    "max_step_mm": 60.0,
                    "move_speed": 0.15,
                    "direct_move_to_target": True,
                    "step_wait_s": 3.0,

                    "base_x_min": 80.0,
                    "base_x_max": 700.0,
                    "base_y_min": -450.0,
                    "base_y_max": 450.0,
                    "base_z_min": -30.0,
                    "base_z_max": 380.0,
                }
            ],
        ),
    ])
