# Action flow V1 archive

这是新功能包建立时的旧动作流程基线快照。归档日期：2026-08-12。

## 归档范围

### 动作与任务节点

- `nodes/visual_servo_task_node.py`：居中、视觉靠近、下降、闭爪、抬升和固定点放置状态机。
- `nodes/open_loop_grasp_task_node.py`：开环预抓取、抓取、抬升和撤退流程。
- `nodes/task_manager_node.py`：较早的目标上方移动流程。
- `nodes/competition_arm_task_node.py`：机器狗高层任务和动作状态机之间的通信适配。
- `nodes/roarm_driver_node.py`：动作命令到 RoArm-M3 串口驱动的 ROS2 适配。
- `roarm_m3/driver.py`：RoArm-M3 串口执行封装。

### 启动与动作配置

- `launch/visual_servo_task.launch.py`
- `launch/open_loop_grasp.launch.py`
- `launch/red_block_task.launch.py`
- `launch/competition_arm_task.launch.py`
- `launch/competition_arm_ros2.launch.py`
- `config/competition_arm.yaml`

## 未归档内容

相机、YOLO、颜色检测、检测模型、深度定位和手眼标定仍是正式复用基础，因此不复制到旧动作档案。其正式位置为：

- `red_block_grasp_mplus0/core/`
- `red_block_grasp_mplus0/nodes/target_localizer_node.py`
- `red_block_grasp_mplus0/nodes/yolo_camera_node.py`
- `models/`
- `handeye/`
- `config/red_color_calib.yaml`
- `launch/localization_only.launch.py`

## 使用约束

本目录仅作只读参考，不参与构建和安装。正式入口目前仍保留旧动作实现；后续按节点、launch 和配置逐项替换为新动作流程。每次替换都应先确认接口、坐标系、速度、工作空间、安全高度、失败恢复和验收条件。
