"""Launch the standalone AprilTag pick pipeline with staged motion gating."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port")
    execution_limit = LaunchConfiguration("execution_limit")
    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("execution_limit", default_value="approach"),
            Node(
                package="apriltag_block_grasp",
                executable="roarm_driver_node",
                name="apriltag_roarm_driver_node",
                output="screen",
                parameters=[
                    {
                        "port": port,
                        "enable_b_joint_motion": True,
                        "enable_observation_motion": True,
                        "enable_gripper_open_motion": True,
                        "enable_pre_grasp_motion": True,
                        "enable_pick_sequence_motion": True,
                        "execution_limit": execution_limit,
                    }
                ],
            ),
            Node(
                package="apriltag_block_grasp",
                executable="apriltag_pnp_node",
                name="apriltag_pnp_node",
                output="screen",
            ),
            Node(
                package="apriltag_block_grasp",
                executable="target_candidate_node",
                name="apriltag_target_candidate_node",
                output="screen",
            ),
            Node(
                package="apriltag_block_grasp",
                executable="manipulation_task_node",
                name="apriltag_manipulation_task_node",
                output="screen",
                parameters=[
                    {
                        "enable_b_search_motion": True,
                        "enable_observation_motion": True,
                        "enable_gripper_open_motion": True,
                        "enable_pre_grasp_motion": True,
                        "enable_pick_sequence_motion": True,
                        "execution_limit": execution_limit,
                    }
                ],
            ),
        ]
    )
