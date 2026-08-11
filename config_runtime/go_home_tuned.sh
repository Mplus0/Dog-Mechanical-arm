#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /home/sunrise/dog/roarm_ws/install/setup.bash

echo "===== go tuned HOME ====="
ros2 service call /move_joint_cmd roarm_msgs/srv/MoveJointCmd "{x: 0.1809, y: -0.0097, z: 0.0734, roll: -1.5754, pitch: 1.4987, yaw: 0.0000}"

sleep 1

echo "===== gripper close 0.0 for HOME ====="
ros2 topic pub --once /gripper_cmd std_msgs/msg/Float32 "{data: 0.0}"
