# Dog-Mechanical-arm 代码复现流程记录

本文档用于记录本次 `Dog-Mechanical-arm` 仓库中 `red_block_grasp_ros2` 功能包的复现流程，适用于当前现场路径：

```bash
/home/sunrise/dog/ros2_red_block_ws
```

外部 RoArm 工作空间路径为：

```bash
/home/sunrise/dog/roarm_ws
```

---

## 1. 进入工作空间

```bash
cd /home/sunrise/dog/ros2_red_block_ws
```

---

## 2. 编译前加载基础环境

编译前需要先加载 ROS2 Humble 和 RoArm 官方工作空间：

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/dog/roarm_ws/install/setup.bash
```

---

## 3. 编译 red_block_grasp_ros2 功能包

```bash
cd /home/sunrise/dog/ros2_red_block_ws
colcon build --packages-select red_block_grasp_ros2 --event-handlers console_direct+
```

正常情况下会看到类似输出：

```text
Finished <<< red_block_grasp_ros2
Summary: 1 package finished
```

注意：当前环境不建议使用：

```bash
--symlink-install
```

因为之前测试时出现过：

```text
error: option --editable not recognized
```

---

## 4. 运行前加载本仓库环境

每次打开新终端后，都需要执行：

```bash
cd /home/sunrise/dog/ros2_red_block_ws
source source_red_block.sh
```

这个脚本已经包含：

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/dog/roarm_ws/install/setup.bash
```

并且会手动加入 `red_block_grasp_ros2` 的运行路径。

---

## 5. 检查环境是否正常

执行：

```bash
ros2 pkg list | grep red_block
ros2 pkg list | grep roarm
python3 -c "from roarm_msgs.srv import GetPoseCmd; print('roarm_msgs ok')"
python3 -c "import red_block_grasp_ros2; print('red_block_grasp_ros2 ok')"
```

正常输出应包含：

```text
red_block_grasp_ros2
roarm_msgs
roarm_msgs ok
red_block_grasp_ros2 ok
```

---

## 6. 启动定位测试

第一条复现运行命令为：

```bash
cd /home/sunrise/dog/ros2_red_block_ws
source source_red_block.sh
ros2 launch red_block_grasp_ros2 localization_only.launch.py show_window:=true
```

该命令用于启动红色方块识别和目标定位流程。

主要作用：

```text
启动 roarm_driver_node
启动 target_localizer_node
打开 Orbbec 相机
加载 YOLO / 颜色检测
发布 /red_block/target_base
```

如果 `show_window:=true`，会显示 OpenCV 调试窗口。

---

## 7. 新开终端查看话题

新开一个终端后，也要先加载环境：

```bash
cd /home/sunrise/dog/ros2_red_block_ws
source source_red_block.sh
```

查看当前 ROS2 话题：

```bash
ros2 topic list
```

查看目标定位结果：

```bash
ros2 topic echo /red_block/target_base
```

查看机械臂状态：

```bash
ros2 topic echo /roarm_m3/state
```

---

## 8. 启动视觉闭环抓取 / 下降测试

```bash
cd /home/sunrise/dog/ros2_red_block_ws
source source_red_block.sh
ros2 launch red_block_grasp_ros2 visual_servo_task.launch.py show_window:=false
```

如果需要显示调试窗口：

```bash
ros2 launch red_block_grasp_ros2 visual_servo_task.launch.py show_window:=true
```

如果需要启用完整抓取、抬升、放置流程：

```bash
ros2 launch red_block_grasp_ros2 visual_servo_task.launch.py show_window:=false enable_pick_place_sequence:=true
```

低算力调试时可以降低推理尺寸：

```bash
ros2 launch red_block_grasp_ros2 visual_servo_task.launch.py show_window:=false infer_imgsz:=224 target_timer_period:=0.05
```

---

## 9. 启动开环抓取流程

```bash
cd /home/sunrise/dog/ros2_red_block_ws
source source_red_block.sh
ros2 launch red_block_grasp_ros2 open_loop_grasp.launch.py show_window:=false
```

该流程会等待稳定目标，然后移动到预抓取点，再执行抓取、抬升和撤退。

---

## 10. 启动比赛机械臂 ROS2 侧任务

```bash
cd /home/sunrise/dog/ros2_red_block_ws
source source_red_block.sh
ros2 launch red_block_grasp_ros2 competition_arm_ros2.launch.py show_window:=false
```

另开一个终端发送测试任务：

```bash
cd /home/sunrise/dog/ros2_red_block_ws
source source_red_block.sh
ros2 topic pub --once /dog_arm/task_cmd std_msgs/msg/String "{data: '{\"task_id\":1,\"cmd\":\"pick\"}'}"
```

查看任务结果：

```bash
ros2 topic echo /dog_arm/task_result
```

查看底盘微调请求：

```bash
ros2 topic echo /dog_arm/base_adjust_req
```

---

## 11. 使用官方 MoveIt2 / RoArm 桥接脚本

```bash
cd /home/sunrise/dog/ros2_red_block_ws
source source_red_block.sh
python3 /home/sunrise/dog/ros2_red_block_ws/dog_visual_grasp_bridge/visual_moveit_grasp.py
```

该方式依赖外部官方 RoArm 工作空间提供的接口，例如：

```text
/move_line_cmd
/move_joint_cmd
/gripper_cmd
```

---

## 12. 本次排查中遇到的问题

### 12.1 Package 'red_block_grasp_ros2' not found

问题现象：

```text
Package 'red_block_grasp_ros2' not found
```

原因：

```text
直接 source install/setup.bash 后，red_block_grasp_ros2 没有正确加入 AMENT_PREFIX_PATH。
```

解决方法：

```bash
cd /home/sunrise/dog/ros2_red_block_ws
source source_red_block.sh
```

然后检查：

```bash
ros2 pkg list | grep red_block
```

正常输出：

```text
red_block_grasp_ros2
```

---

### 12.2 ModuleNotFoundError: No module named 'roarm_msgs'

问题现象：

```text
ModuleNotFoundError: No module named 'roarm_msgs'
```

原因：

```text
target_localizer_node.py 中导入了 roarm_msgs.srv.GetPoseCmd，
但当前终端没有加载 /home/sunrise/dog/roarm_ws/install/setup.bash。
```

解决方法：

确认 `source_red_block.sh` 中包含：

```bash
source /home/sunrise/dog/roarm_ws/install/setup.bash
```

然后重新执行：

```bash
cd /home/sunrise/dog/ros2_red_block_ws
source source_red_block.sh
```

检查：

```bash
ros2 pkg list | grep roarm
python3 -c "from roarm_msgs.srv import GetPoseCmd; print('roarm_msgs ok')"
```

正常输出应包含：

```text
roarm_msgs
roarm_msgs ok
```

---

## 13. 当前推荐的最小复现流程

如果只是想快速重新运行第一条定位测试命令，直接执行：

```bash
cd /home/sunrise/dog/ros2_red_block_ws
source source_red_block.sh
ros2 launch red_block_grasp_ros2 localization_only.launch.py show_window:=true
```

另开终端查看结果：

```bash
cd /home/sunrise/dog/ros2_red_block_ws
source source_red_block.sh
ros2 topic echo /red_block/target_base
```
