# apriltag_block_grasp

独立的 AprilTag 物块定位与 RoArm-M3 抓取功能包，按
[`docs/apriltag_block_grasp_development_plan.md`](../../docs/apriltag_block_grasp_development_plan.md)
分阶段开发。

当前只完成阶段 0：最小 ROS 2 `ament_python` 包骨架和只读环境检查工具。
尚未实现相机取流、AprilTag 检测、定位或任何机械臂动作。

## 阶段 0：主控环境检查

在 RDK X5 的 ROS 2 工作空间中同步本仓库后执行：

```bash
cd /home/sunrise/dog/ros2_red_block_ws
source /opt/ros/humble/setup.bash
source /home/sunrise/dog/roarm_ws/install/setup.bash

colcon list | grep apriltag_block_grasp
colcon build --packages-select apriltag_block_grasp --event-handlers console_direct+
source install/setup.bash

ros2 run apriltag_block_grasp check_environment
```

如需保存机器可读结果：

```bash
ros2 run apriltag_block_grasp check_environment --json
```

检查工具只导入依赖并检查 API，不会打开 Orbbec 相机、连接机械臂串口、创建 ROS
节点或发送运动命令。

请保留完整输出。阶段 1 开始前，需要据此确定：

- OpenCV 是否提供 `cv2.aruco.DICT_APRILTAG_25h9`；
- 是否提供 `cv2.SOLVEPNP_IPPE_SQUARE`；
- `pyorbbecsdk` 是否可导入并包含所需 API；
- ROS 2、`std_msgs`、`ament_index_python` 和 `roarm_msgs/GetPoseCmd` 是否可用。

`opencv_apriltag_25h9` 或 `opencv_ippe_square` 显示 `WARN` 不代表基础环境检查
失败：后续分别允许使用专用 AprilTag 检测器或普通 PnP 回退。任何标为 `required`
的项目显示 `FAIL` 时，不进入阶段 1。
