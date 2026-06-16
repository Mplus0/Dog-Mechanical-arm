#!/usr/bin/env bash

source /opt/ros/humble/setup.bash

# 加载 RoArm 官方工作空间，提供 roarm_msgs 等依赖
source /home/sunrise/dog/roarm_ws/install/setup.bash

WS=/home/sunrise/dog/ros2_red_block_ws
PKG_PREFIX=$WS/install/red_block_grasp_ros2

export AMENT_PREFIX_PATH=$PKG_PREFIX:${AMENT_PREFIX_PATH:-}
export CMAKE_PREFIX_PATH=$PKG_PREFIX:${CMAKE_PREFIX_PATH:-}
export COLCON_PREFIX_PATH=$PKG_PREFIX:${COLCON_PREFIX_PATH:-}
export PYTHONPATH=$PKG_PREFIX/lib/python3.10/site-packages:${PYTHONPATH:-}
export PATH=$PKG_PREFIX/lib/red_block_grasp_ros2:${PATH:-}