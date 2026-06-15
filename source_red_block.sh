#!/usr/bin/env bash

source /opt/ros/humble/setup.bash

WS=/home/sunrise/dog/ros2_red_block_ws
PKG_PREFIX=$WS/install/red_block_grasp_ros2

export AMENT_PREFIX_PATH=$PKG_PREFIX:${AMENT_PREFIX_PATH:-}
export CMAKE_PREFIX_PATH=$PKG_PREFIX:${CMAKE_PREFIX_PATH:-}
export PYTHONPATH=$PKG_PREFIX/lib/python3.10/site-packages:${PYTHONPATH:-}
export PATH=$PKG_PREFIX/lib/red_block_grasp_ros2:${PATH:-}
