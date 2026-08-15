# Dog-Mechanical-arm

本仓库是一个基于 ROS2 的机械臂视觉抓取测试工程，当前代码重点围绕 **红色方块识别、RoArm-M3 机械臂控制、视觉伺服抓取/下降测试、比赛任务接口** 展开。项目中可以确认的硬件与运行对象包括：

- 机械臂：`RoArm-M3`
- 相机：`Orbbec` RGB / RGBD 相机
- 识别方法：YOLO 模型、红色阈值检测、二者融合
- 当前正式 ROS2 功能包：`red_block_grasp_mplus0`
- 主要目标：识别红色目标块，将目标从相机坐标转换到机械臂 base 坐标，控制机械臂靠近、下降、夹爪闭合、抬升和放置

> 注意：本仓库没有发现 URDF、Gazebo、rviz2、ros2_control 或完整 MoveIt2 配置文件。仓库中的 `dog_visual_grasp_bridge` 会调用外部官方 RoArm/MoveIt2 接口，但官方工作空间本身不在本仓库内，需要根据现场平台确认。

> 当前通信基线（2026-08-15）：机器狗 ROS1 端运行 TCP 客户端，机械臂 ROS2 端运行 TCP 服务端，双方通过一条带 HMAC 认证的全双工 TCP 连接双向收发。正式流程不依赖 `ros1_bridge`、`dynamic_bridge` 或 `parameter_bridge`。旧包 `red_block_grasp_ros2` 和 `docs/ros1_ros2_bridge_arm_side.md` 仅保留为历史实现参考，不是现场启动入口。

## 1. 项目简介

项目主要用于在 ROS2 Humble 环境下复现和二次开发机械臂红色方块抓取流程。整体流程是：

```text
Orbbec 相机采集 RGBD 图像
-> YOLO / 颜色阈值检测红色目标
-> 结合深度图得到目标相机坐标
-> 使用手眼标定转换到机械臂 base 坐标
-> 通过 /roarm_m3/cmd 控制 RoArm-M3
-> 视觉闭环小步靠近目标
-> 下降测试 / 夹爪闭合 / 抬升 / 放置
```

适合这些场景：

- ROS2 初学者学习 `launch`、`node`、`topic`、参数和 Python 功能包结构；
- 有 ROS1 基础的同学理解 ROS2 中 `ament_python`、`colcon`、`launch.py` 的用法；
- 机械臂共创者复现红色方块抓取实验；
- 后续继续调参与扩展比赛任务接口。

## 2. 代码目录结构

```text
dog-arm/
├── README.md                         # 本说明文档
├── source_red_block.sh               # 旧环境脚本；当前建议直接 source install/setup.bash
├── docs/
│   └── ros1_ros2_bridge_arm_side.md  # 历史 ROS1/ROS2 bridge 方案，不用于当前运行
├── config_runtime/
│   └── red_color_calib_tuned.yaml    # 现场调过的红色阈值配置
├── dog_visual_grasp_bridge/
│   ├── visual_moveit_grasp.py        # 将 /red_block/target_base 转成官方 MoveIt2/RoArm 服务调用的桥接脚本
│   └── config/grasp_config.yaml      # 桥接脚本的抓取、补偿、夹爪、工作空间参数
├── tools/
│   ├── calibrate_red_green_threshold.py       # 交互式采样红/绿/背景，生成颜色阈值
│   ├── manual_yolo_labeler.py                 # 手工标注 YOLO 数据集
│   ├── prepare_red_block_v2_dataset.py        # 数据集整理脚本
│   ├── build_v2_dataset_from_saved_samples.py # 从保存样本构建数据集
│   └── build_red_block_v3_mix.py              # 构建混合训练数据集
└── src/
    ├── red_block_grasp_mplus0/        # 当前正式 ROS2 功能包
    │   ├── package.xml                # ROS2 ament_python 功能包依赖
    │   ├── setup.py                   # Python 节点入口与 launch/config 安装配置
    │   ├── launch/                    # ROS2 启动文件
    │   ├── config/                    # 比赛任务与颜色阈值配置
    │   ├── models/                    # YOLO 红色方块模型
    │   ├── handeye/                   # 手眼标定结果
    │   ├── docs/                      # TCP 协议与现场调试文档
    │   └── red_block_grasp_mplus0/
    │       ├── nodes/                 # ROS2 节点
    │       ├── core/                  # 相机、检测、定位核心逻辑
    │       ├── roarm_m3/              # RoArm-M3 串口驱动封装
    │       └── tools/                 # 包内工具脚本
    ├── models/                        # 额外保存的 YOLO 模型
    └── handeye/                       # 额外保存的手眼标定文件
```

`backup/` 中是历史备份文件，不是当前主要运行入口。`build/`、`install/`、`log/` 等编译产物不应提交。

## 3. 功能包说明

### red_block_grasp_mplus0

这是当前正式 ROS2 功能包，类型是 `ament_python`。它负责相机读取、红色目标检测、目标三维定位、RoArm-M3 串口控制、视觉伺服抓取任务、比赛高层任务接口，以及机械臂侧 TCP 服务。旧包 `red_block_grasp_ros2` 仅作为历史基线保留。

| 文件/目录 | 作用 |
|---|---|
| `package.xml` | 声明 ROS2 依赖：`rclpy`、`std_msgs`、`roarm_msgs`、`launch`、`launch_ros` |
| `setup.py` | 注册 `ros2 run` 可执行节点，安装 `launch/` 与 `config/` |
| `launch/visual_servo_task.launch.py` | 当前推荐的视觉闭环抓取/下降测试启动文件 |
| `launch/competition_arm_task.launch.py` | 当前比赛入口；启动 TCP 服务端、比赛任务接口和视觉伺服，读取 `competition_arm.yaml` |
| `launch/competition_arm_ros2.launch.py` | 兼容入口；同样包含 TCP 服务端和 `/dog_arm/task_cmd` 高层命令接口 |
| `launch/localization_only.launch.py` | 只启动识别定位，用于调相机、YOLO、颜色阈值和手眼标定 |
| `launch/open_loop_grasp.launch.py` | 开环抓取流程，依赖稳定目标快照后直接执行预抓取、抓取、抬升、撤退 |
| `launch/red_block_task.launch.py` | 较早期的任务管理启动方式，使用 `task_manager_node` |
| `config/red_color_calib.yaml` | 红色颜色检测阈值 |
| `config/competition_arm.yaml` | 比赛任务接口和视觉伺服比赛模式参数 |
| `models/*.pt` | YOLO 红色目标模型 |
| `handeye/handeye_cam_to_eef.json` | 相机到末端执行器的手眼标定矩阵 |
| `nodes/` | ROS2 节点源码 |
| `core/` | 相机、YOLO、颜色检测、坐标转换等核心逻辑 |
| `roarm_m3/driver.py` | RoArm-M3 串口底层封装 |

### dog_visual_grasp_bridge

这个目录不是标准 ROS2 功能包，没有 `package.xml`。它是一个独立 Python 脚本，用于把本项目发布的视觉定位结果桥接到外部官方 RoArm/MoveIt2 控制接口。

| 文件/目录 | 作用 |
|---|---|
| `visual_moveit_grasp.py` | 订阅 `/red_block/target_base`，调用 `/move_line_cmd`、`/move_joint_cmd`，发布 `/gripper_cmd` |
| `config/grasp_config.yaml` | 坐标缩放、抓取偏置、工作空间、夹爪值、官方关节 topic 等配置 |

适用场景：现场已有官方 `roarm_ws`、`roarm_msgs`、MoveIt2/控制服务时，用视觉结果驱动官方控制链路。该脚本不负责相机、YOLO 或手眼标定。

## 4. 关键代码文件说明

### launch 文件

| 文件 | 启动内容 | 适合场景 |
|---|---|---|
| `visual_servo_task.launch.py` | `roarm_driver_node`、`target_localizer_node`、`visual_servo_task_node`，可选 `execution_logger_node` | 当前主要抓取/下降测试入口 |
| `localization_only.launch.py` | 可选 `roarm_driver_node`、`target_localizer_node` | 只看红色目标识别、深度定位和 `/red_block/target_base` 是否稳定 |
| `open_loop_grasp.launch.py` | `roarm_driver_node`、`target_localizer_node`、`open_loop_grasp_task_node`，可选日志节点 | 开环抓取测试 |
| `competition_arm_task.launch.py` | TCP 服务端、`roarm_driver_node`、`target_localizer_node`、`visual_servo_task_node`、`competition_arm_task_node`、日志节点 | 当前完整比赛入口 |
| `competition_arm_ros2.launch.py` | 兼容的完整比赛入口，`enable_pick_place_sequence` 默认为 true | 需要保留旧启动名时使用 |
| `red_block_task.launch.py` | `roarm_driver_node`、`target_localizer_node`、`task_manager_node` | 早期直接移动到目标上方的流程，需要进一步确认是否仍推荐 |

常用 launch 参数：

- `arm_port`：机械臂串口，默认 `/dev/ttyUSB0`。
- `model_path`：YOLO 模型路径，部分 launch 默认写死为 `/home/sunrise/dog/ros2_red_block_ws/...`。
- `handeye_path`：手眼标定文件路径。
- `show_window`：是否显示 OpenCV 调试窗口，X5 现场高帧率调试建议设为 `false`。
- `infer_imgsz`：YOLO 推理尺寸，越小越快，但可能降低稳定性。
- `target_timer_period`：定位节点周期。
- `enable_pick_place_sequence`：在视觉靠近和下降后，是否继续执行闭爪、抬升、放置、开爪。

### 主要节点

| 节点 | 文件 | 订阅 | 发布/调用 | 作用 |
|---|---|---|---|---|
| `roarm_driver_node` | `nodes/roarm_driver_node.py` | `/roarm_m3/cmd` | `/roarm_m3/state` | 唯一占用 RoArm-M3 串口的节点，执行关节、末端位姿、夹爪/LED 等 JSON 命令 |
| `target_localizer_node` | `nodes/target_localizer_node.py` | `/roarm_m3/state` 或可选 `/get_pose_cmd` 服务 | `/red_block/target_base` | 读取 Orbbec RGBD，融合 YOLO/颜色检测，输出目标 base 坐标 |
| `visual_servo_task_node` | `nodes/visual_servo_task_node.py` | `/roarm_m3/state`、`/red_block/target_base`、`/red_block/visual_servo_cmd` | `/roarm_m3/cmd`、`/red_block/visual_servo_state` | 视觉闭环状态机，小步靠近、下降、闭爪、抬升、放置 |
| `open_loop_grasp_task_node` | `nodes/open_loop_grasp_task_node.py` | `/roarm_m3/state`、`/red_block/target_base` | `/roarm_m3/cmd`、`/red_block/open_loop_grasp_state` | 开环抓取状态机 |
| `competition_arm_task_node` | `nodes/competition_arm_task_node.py` | `/dog_arm/task_cmd`、`/red_block/visual_servo_state` | `/dog_arm/task_result`、`/dog_arm/base_adjust_req`、`/red_block/visual_servo_cmd` | 将比赛高层命令转成视觉伺服命令，并回传结果 |
| `execution_logger_node` | `nodes/execution_logger_node.py` | `/roarm_m3/state`、`/red_block/target_base`、`/red_block/visual_servo_state`、`/roarm_m3/cmd` | JSONL 文件 | 记录现场运行数据 |
| `yolo_camera_node` | `nodes/yolo_camera_node.py` | 无 | `/red_block/detections` | 只做 Orbbec 彩色图 YOLO 检测，偏调试用途 |
| `task_manager_node` | `nodes/task_manager_node.py` | `/roarm_m3/state`、`/red_block/target_base` | `/roarm_m3/cmd`、`/red_block/task_state` | 早期任务管理节点，主要移动到目标上方，需要进一步确认是否仍使用 |

### core 和配置文件

- `core/camera_rgbd_orbbec.py`：启动 Orbbec RGBD 流，读取彩色图、深度图、相机内参。
- `core/camera_orbbec.py`：只读取 Orbbec 彩色图。
- `core/yolo_detector.py`：加载 `ultralytics.YOLO` 模型并返回检测框。
- `core/color_red_block_detector.py`：基于 HSV/LAB/BGR 阈值和形态学处理检测红色块。
- `core/target_localizer.py`：用深度图把像素转换到相机坐标，再结合手眼标定和机械臂状态转换到 base 坐标。
- `config/red_color_calib.yaml`：红色阈值参数，如 HSV 双区间、LAB a 通道阈值、面积范围、深度范围。
- `config/competition_arm.yaml`：比赛接口 topic、超时、底盘左右微调、放置点等参数。
- `handeye/handeye_cam_to_eef.json`：手眼标定结果，含 `T_eef_camera`、`R_cam2eef`、`t_cam2eef_mm` 和标定质量指标。

本功能包没有 `CMakeLists.txt`，因为它是 `ament_python` 包；编译和安装逻辑在 `setup.py` 与 `setup.cfg` 中。

## 5. 环境依赖

代码中能确认的建议环境：

- ROS2：Humble
- Python：ROS2 Humble 通常对应 Python 3.10
- 平台：已有 README 和脚本路径显示现场使用 RDK X5 与 `/home/sunrise/dog/ros2_red_block_ws`
- 机械臂：RoArm-M3，默认串口 `/dev/ttyUSB0`
- 相机：Orbbec RGBD，需要 `pyorbbecsdk`

ROS 依赖：

```bash
sudo apt install ros-humble-rclpy ros-humble-std-msgs ros-humble-sensor-msgs
sudo apt install ros-humble-launch ros-humble-launch-ros
```

还需要现场提供或另行安装：

- `roarm_msgs`：本仓库依赖它的 `GetPoseCmd`、`MoveLineCmd`、`MoveJointCmd` 服务类型；
- 跨机通信不需要 `ros1_bridge`；当前功能包自带 TCP 服务端，机器狗仓库自带 TCP 客户端；
- MoveIt2 / 官方 RoArm 工作空间：只在使用 `dog_visual_grasp_bridge/visual_moveit_grasp.py` 时需要。

Python 依赖：

```bash
python3 -m pip install ultralytics opencv-python numpy pyserial pyyaml
```

`pyorbbecsdk` 的安装方式通常与平台和 Orbbec SDK 版本有关，仓库中没有锁定安装命令，需要根据相机型号和系统环境确认。

Ubuntu 版本没有在代码中明确写出。由于 ROS2 Humble 官方常用 Ubuntu 22.04，建议优先使用 Ubuntu 22.04 + ROS2 Humble；如果在 RDK X5 上运行，以现场已验证系统为准。

## 6. 编译方法

以下命令假设仓库是 ROS2 工作空间根目录，且功能包位于 `src/red_block_grasp_mplus0`：

```bash
cd /home/sunrise/dog/ros2_red_block_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select red_block_grasp_mplus0 --event-handlers console_direct+
source install/setup.bash
```

如果你的路径不是 `/home/sunrise/dog/ros2_red_block_ws`，仍可按实际路径进入工作空间编译：

```bash
cd <your_ros2_ws>
source /opt/ros/humble/setup.bash
colcon build --packages-select red_block_grasp_mplus0
source install/setup.bash
```

注意事项：

- 每打开一个新终端，都需要重新执行 `cd /home/sunrise/dog/ros2_red_block_ws && source install/setup.bash`。
- 修改 `nodes/`、`core/`、`launch/`、`config/` 后建议重新 `colcon build`。
- `source_red_block.sh` 写死了 `/home/sunrise/dog/ros2_red_block_ws`，只适合现场同路径环境。
- Python 脚本如果直接执行，需要检查执行权限：`chmod +x path/to/script.py`。
- 如果 `rosdep` 找不到 `roarm_msgs`，说明官方 RoArm 消息包不在当前环境中，需要先编译或 source 对应工作空间。

## 7. 运行方法

### 7.1 只验证相机识别和目标定位

用于确认 Orbbec、YOLO/颜色检测、手眼标定和 `/red_block/target_base` 是否正常。

```bash
cd /home/sunrise/dog/ros2_red_block_ws
source install/setup.bash
ros2 launch red_block_grasp_mplus0 localization_only.launch.py show_window:=true
```

如果已经有官方 RoArm driver 占用串口，可关闭本仓库 driver：

```bash
ros2 launch red_block_grasp_mplus0 localization_only.launch.py start_roarm_driver:=false show_window:=true
```

### 7.2 启动视觉闭环抓取/下降测试

这是当前仓库最核心的运行入口：

```bash
ros2 launch red_block_grasp_mplus0 visual_servo_task.launch.py show_window:=false
```

显示 OpenCV 调试窗口：

```bash
ros2 launch red_block_grasp_mplus0 visual_servo_task.launch.py show_window:=true
```

启用完整抓取放置流程：

```bash
ros2 launch red_block_grasp_mplus0 visual_servo_task.launch.py show_window:=false enable_pick_place_sequence:=true
```

低算力现场可尝试降低推理尺寸：

```bash
ros2 launch red_block_grasp_mplus0 visual_servo_task.launch.py show_window:=false infer_imgsz:=224 target_timer_period:=0.05
```

### 7.3 启动开环抓取流程

```bash
ros2 launch red_block_grasp_mplus0 open_loop_grasp.launch.py show_window:=false
```

该流程会等待稳定目标，移动到预抓取点，再直接抓取、抬升、撤退。相比视觉闭环，目标丢失后的容错更弱，建议先用 `visual_servo_task.launch.py`。

### 7.4 启动比赛机械臂 ROS2 侧任务

```bash
ros2 launch red_block_grasp_mplus0 competition_arm_ros2.launch.py show_window:=false
```

在另一个终端发送测试任务：

```bash
ros2 topic pub --once /dog_arm/task_cmd std_msgs/msg/String "{data: '{\"task_id\":1,\"cmd\":\"pick\"}'}"
ros2 topic pub --once /dog_arm/task_cmd std_msgs/msg/String "{data: '{\"task_id\":2,\"cmd\":\"place_to_zone\"}'}"
```

查看结果：

```bash
ros2 topic echo /dog_arm/task_result
ros2 topic echo /dog_arm/base_adjust_req
```

### 7.5 使用官方 MoveIt2/RoArm 桥接脚本

该方式需要外部官方 `roarm_ws` 提供 `/move_line_cmd`、`/move_joint_cmd` 和 `/gripper_cmd` 等接口。

```bash
cd /home/sunrise/dog/ros2_red_block_ws
source install/setup.bash
python3 /home/sunrise/dog/ros2_red_block_ws/dog_visual_grasp_bridge/visual_moveit_grasp.py
```

如果你的仓库路径不同，请把脚本路径改成实际路径。

## 8. 机器狗与机械臂通信策略

根据当前代码，机械臂与机器狗的跨机通信采用 **全双工 TCP + JSON 消息协议**。ROS1/ROS2 topic 只在各自主机内部使用；机械臂内部的相机、目标定位、串口控制、视觉伺服等 topic 不直接暴露给机器狗。

当前设计可以概括为：

```text
机器狗侧 ROS1
  发布 /dog_arm/task_cmd
  订阅 /dog_arm/task_result
  订阅 /dog_arm/base_adjust_req
        |
        | dog_arm_tcp_client_node.py
        | HMAC 认证的全双工 TCP（默认端口 47001）
        v
机械臂侧 ROS2
  dog_arm_tcp_server_node
        |
  competition_arm_task_node
        |
        | /red_block/visual_servo_cmd
        v
  visual_servo_task_node
        |
        | /roarm_m3/cmd
        v
  roarm_driver_node -> RoArm-M3
```

### 8.1 当前已实现的对外接口

业务接口仍使用下面 3 个 topic，类型均为 `std_msgs/String`，消息内容是 JSON 字符串。TCP 节点负责在两台主机各自的内部 topic 与网络消息之间转换。

| 方向 | topic | 类型 | 作用 |
|---|---|---|---|
| 机器狗 -> 机械臂 | `/dog_arm/task_cmd` | `std_msgs/String` | 发送高层任务命令 |
| 机械臂 -> 机器狗 | `/dog_arm/task_result` | `std_msgs/String` | 返回抓取/放置结果 |
| 机械臂 -> 机器狗 | `/dog_arm/base_adjust_req` | `std_msgs/String` | 请求机器狗左右微调底盘 |

接口实现在：

- `src/red_block_grasp_mplus0/red_block_grasp_mplus0/nodes/competition_arm_task_node.py`
- `src/red_block_grasp_mplus0/red_block_grasp_mplus0/nodes/dog_arm_tcp_server_node.py`
- `src/red_block_grasp_mplus0/config/competition_arm.yaml`
- `src/red_block_grasp_mplus0/docs/dog_arm_tcp_transport.md`

### 8.2 任务命令协议

当前 `competition_arm_task_node` 只接受两个命令：

```json
{"task_id": 1, "cmd": "pick"}
{"task_id": 2, "cmd": "place_to_zone"}
```

含义：

- `pick`：机械臂在抓取区识别红色目标，执行视觉伺服靠近、下降、闭爪、抬升，并在抬升后保持一小段时间。
- `place_to_zone`：默认机器狗已经移动到正确放置区前并站稳，机械臂执行固定放置点移动、开爪、等待、回到安全/初始姿态。

当前代码没有让机械臂识别 A/B/C/D 区域，也没有让机械臂控制机器狗导航。机器狗负责移动到抓取区/放置区并保持稳定，机械臂只负责本体抓取和放置动作。

ROS2 侧测试命令：

```bash
ros2 topic pub --once /dog_arm/task_cmd std_msgs/msg/String "{data: '{\"task_id\":1,\"cmd\":\"pick\"}'}"
ros2 topic pub --once /dog_arm/task_cmd std_msgs/msg/String "{data: '{\"task_id\":2,\"cmd\":\"place_to_zone\"}'}"
```

机器狗 ROS1 侧的本地 topic 测试命令（需已启动 TCP 客户端）：

```bash
rostopic pub -1 /dog_arm/task_cmd std_msgs/String '{"data":"{\"task_id\":1,\"cmd\":\"pick\"}"}'
rostopic pub -1 /dog_arm/task_cmd std_msgs/String '{"data":"{\"task_id\":2,\"cmd\":\"place_to_zone\"}"}'
```

### 8.3 结果回传协议

机械臂通过 `/dog_arm/task_result` 返回结果：

```json
{"task_id": 1, "result": "pick_success"}
{"task_id": 2, "result": "place_success"}
{"task_id": 1, "result": "pick_failed", "error": "target_not_found"}
{"task_id": 1, "result": "pick_failed", "error": "target_lost"}
{"task_id": 1, "result": "pick_failed", "error": "need_base_adjust"}
{"task_id": 2, "result": "place_failed", "error": "place_motion_failed"}
{"task_id": 1, "result": "pick_failed", "error": "task_timeout"}
```

常见错误含义：

- `target_not_found`：扫描后没有找到红色目标；
- `target_lost`：视觉伺服移动后目标丢失；
- `need_base_adjust`：目标位于机械臂横向工作空间边缘，需要机器狗左右微调；
- `arm_task_busy`：机械臂已有任务在执行；
- `task_timeout`：抓取或放置超时。

### 8.4 底盘微调请求

当红色目标太靠近机械臂横向工作空间边缘时，`visual_servo_task_node` 会在比赛模式下生成 `base_adjust` 请求，`competition_arm_task_node` 会转发到 `/dog_arm/base_adjust_req`。

消息示例：

```json
{"task_id": 1, "direction": "left", "step_m": 0.05, "reason": "target_too_right"}
{"task_id": 1, "direction": "right", "step_m": 0.05, "reason": "target_too_left"}
```

说明：

- `direction` 是机器狗自身坐标系的左右移动方向，不是图像左右。
- `step_m` 默认来自 `competition_arm.yaml` 中的 `base_adjust_step_m`，当前为 `0.05` m。
- 当前实现会同时返回 `pick_failed` + `need_base_adjust`，机器狗侧完成微调并站稳后，应重新发送新的 `pick` 任务。

相关参数：

```yaml
competition_arm_task_node:
  ros__parameters:
    base_adjust_step_m: 0.05

visual_servo_task_node:
  ros__parameters:
    base_adjust_y_margin_mm: 40.0
    base_adjust_y_threshold_mm: 260.0
```

### 8.5 当前 TCP 传输策略

TCP 的“客户端/服务端”只表示由谁发起连接，不限制消息方向。连接建立后，机器狗可以发送任务命令，机械臂可以在同一连接中回传任务结果和底盘微调请求。传输层包含 HMAC 双向身份认证、心跳、断线重连、确认重发和重复消息抑制，不需要 ROS1/ROS2 bridge。

两端必须预先写入相同的共享密钥，且密钥文件不能提交到 Git：

```bash
mkdir -p ~/.ros
printf '%s' '<现场随机长密钥>' > ~/.ros/dog_arm_shared_secret
chmod 600 ~/.ros/dog_arm_shared_secret
```

机械臂侧启动完整任务和 TCP 服务端：

```bash
cd /home/sunrise/dog/ros2_red_block_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch red_block_grasp_mplus0 competition_arm_task.launch.py \
  show_window:=false \
  tcp_bind_host:=192.168.31.56 \
  tcp_allowed_client_ip:=192.168.31.192
```

机器狗侧启动 ROS1 协议适配器和 TCP 客户端：

```bash
cd ~/comp2026_ws
source devel/setup.bash
roslaunch dog_arm_bridge dog_arm_bridge.launch \
  arm_server_host:=192.168.31.56
```

连接检查：

```bash
# 机械臂端
ros2 node list | grep dog_arm_tcp_server
ss -lntp | grep 47001

# 机器狗端；期望输出 data: True
rostopic echo -n 1 /dog_arm/transport_connected
```

默认地址是机械臂 `192.168.31.56`、机器狗 `192.168.31.192`、端口 `47001`。现场 IP 改变时只改 launch 参数，不修改代码。TCP 帧未加密，因此只能用于受控比赛局域网。

### 8.6 后续通信接口预留

后续如果要扩展机器狗与机械臂协作，建议继续保持“高层 JSON 协议”风格，优先在 `/dog_arm/*` 命名空间下新增字段或 topic，而不是直接暴露机械臂内部 topic。

建议预留方向：

| 需求 | 推荐扩展方式 |
|---|---|
| 机器狗通知已经到达抓取区/放置区 | 在 `/dog_arm/task_cmd` 中增加 `zone`、`ready`、`pose_hint` 等字段，或新增 `/dog_arm/base_state` |
| 机械臂请求机器狗前后移动 | 扩展 `/dog_arm/base_adjust_req` 的 `direction`，但需要同步修改 `competition_arm_task_node.py` 中的方向校验 |
| 增加放置区 A/B/C/D | 在 `place_to_zone` 命令中增加 `zone` 字段，并在 `visual_servo_task_node.py` 中按 zone 选择放置点 |
| 增加任务取消 | 新增 `cmd: "cancel"`，并在 `competition_arm_task_node.py` 与 `visual_servo_task_node.py` 中实现状态清理 |
| 更严格的业务协议 | 后续可从 `std_msgs/String` JSON 升级为自定义结构；当前 TCP 传输保持 JSON 是为了兼容 ROS1/ROS2 两端 |

当前最小协作闭环建议：

```text
机器狗移动到抓取区并站稳
-> 发送 {"cmd":"pick"}
-> 机械臂抓取并返回 pick_success
-> 机器狗移动到目标放置区并站稳
-> 发送 {"cmd":"place_to_zone"}
-> 机械臂放置并返回 place_success
```

如果收到 `base_adjust_req`：

```text
机器狗按 direction 横移 step_m
-> 站稳
-> 重新发送 pick
```

## 9. 机械臂抓取测试流程

视觉闭环流程可以理解为：

```text
启动 roarm_driver_node，占用 RoArm-M3 串口
-> 启动 target_localizer_node，读取 Orbbec RGBD
-> 发布 /red_block/target_base
-> visual_servo_task_node 初始化机械臂姿态
-> 若找不到目标，按 b 关节偏移扫描
-> 先把目标调到图像中心
-> 高位小步靠近目标上方
-> 到达预抓取位置 REACHED_PRE_GRASP
-> 在像素保护下分步下降 DESCEND_TEST
-> 默认到 DONE
-> 如果 enable_pick_place_sequence:=true，继续闭爪、抬升、移动到放置点、开爪
```

当前主要状态机包括：

```text
INIT
-> WAIT_INITIAL
-> WAIT_TARGET
-> CENTER_TARGET
-> APPROACH_CENTERED / SERVO_STEP
-> REACHED_PRE_GRASP
-> DESCEND_TEST
-> WAIT_AFTER_DESCEND
-> DONE
```

启用完整抓取放置后继续：

```text
CLOSE_GRIPPER
-> LIFT_AFTER_GRASP
-> MOVE_TO_PLACE
-> OPEN_GRIPPER
-> DONE
```

运行后预期现象：

- `/red_block/target_base` 持续输出目标是否有效、像素坐标、深度、base 坐标；
- `/roarm_m3/state` 持续输出机械臂连接状态和当前状态；
- `/roarm_m3/cmd` 能看到 `set_initial_pose`、`move_pose`、`move_joint` 等 JSON 命令；
- `/red_block/visual_servo_state` 能看到任务状态机变化。

## 10. 常用调试命令

查看节点：

```bash
ros2 node list
```

查看 topic：

```bash
ros2 topic list
```

查看目标定位：

```bash
ros2 topic echo /red_block/target_base
```

查看机械臂状态和命令：

```bash
ros2 topic echo /roarm_m3/state
ros2 topic echo /roarm_m3/cmd
```

查看视觉伺服状态：

```bash
ros2 topic echo /red_block/visual_servo_state
```

查看比赛接口：

```bash
ros2 topic echo /dog_arm/task_cmd
ros2 topic echo /dog_arm/task_result
ros2 topic echo /dog_arm/base_adjust_req
```

查看 launch 参数：

```bash
ros2 launch red_block_grasp_mplus0 visual_servo_task.launch.py --show-args
```

查看服务和动作：

```bash
ros2 service list
ros2 action list
```

本仓库主流程基本使用 topic 传 JSON；如果使用官方 MoveIt2/RoArm 桥接脚本，应能看到 `/move_line_cmd`、`/move_joint_cmd` 等服务。

查看运行日志文件：

```bash
ls -lh /home/sunrise/dog/ros2_red_block_ws/run_records
tail -f /home/sunrise/dog/ros2_red_block_ws/run_records/<latest_run_file>.jsonl
```

## 11. 常见问题与解决方法

### 找不到功能包 `red_block_grasp_mplus0`

原因通常是没有编译或没有 source。

```bash
colcon build --packages-select red_block_grasp_mplus0
source install/setup.bash
ros2 pkg list | grep red_block_grasp_mplus0
```

### launch 中模型或手眼文件找不到

多个 launch 默认路径写死为 `/home/sunrise/dog/ros2_red_block_ws/...`。如果仓库放在别处，请通过 launch 参数覆盖：

```bash
ros2 launch red_block_grasp_mplus0 visual_servo_task.launch.py \
  model_path:=<your_ws>/src/red_block_grasp_mplus0/models/red_block_yolo11n_v3_mix.pt \
  handeye_path:=<your_ws>/src/red_block_grasp_mplus0/handeye/handeye_cam_to_eef.json
```

### 机械臂不动

排查顺序：

```bash
ros2 topic echo /roarm_m3/state
ros2 topic echo /roarm_m3/cmd
```

如果 `connected=false`，检查串口是否正确、是否被其他程序占用、当前用户是否有串口权限。默认串口是 `/dev/ttyUSB0`，可用 `arm_port:=/dev/ttyXXX` 修改。

### Orbbec 相机启动失败

检查是否安装 `pyorbbecsdk`，相机是否被其他节点占用。不要同时启动多个会占用 Orbbec 的节点，例如不要同时运行多个 `target_localizer_node` 或 `yolo_camera_node`。

### `/red_block/target_base` 没有有效目标

可能原因：

- 红色目标不在画面中；
- YOLO 模型路径错误；
- 颜色阈值不适合现场光照；
- 深度无效或目标超出 `color_min_depth_mm` / `color_max_depth_mm`；
- 手眼标定或机械臂状态不可用。

建议先运行：

```bash
ros2 launch red_block_grasp_mplus0 localization_only.launch.py show_window:=true
ros2 topic echo /red_block/target_base
```

### 目标识别慢或卡顿

可关闭窗口、降低 YOLO 输入尺寸、增大或减小定位周期：

```bash
ros2 launch red_block_grasp_mplus0 visual_servo_task.launch.py show_window:=false infer_imgsz:=224 target_timer_period:=0.05
```

### 目标在移动后丢失

视觉伺服会小步移动并重新识别。如果仍丢失，优先调小：

- `max_step_mm`
- `edge_step_mm`
- `move_speed`
- `step_wait_s`

并检查 `center_b_pixel_sign`、`center_e_pixel_sign` 是否方向相反。

### 下降方向不对或越降越偏

重点检查：

- `descend_pixel_x_sign`
- `descend_pixel_y_sign`
- `descend_pixel_deadband`
- `descend_pixel_max_xy_step_mm`
- `descend_step_mm`

建议先用较小 `descend_test_mm` 做安全测试。

### MoveIt2 / 官方服务不可用

本仓库没有提供官方 RoArm MoveIt2 工作空间。使用 `dog_visual_grasp_bridge` 前，先确认：

```bash
ros2 service list | grep move
ros2 topic list | grep gripper
```

并确认已经 source 官方工作空间：

```bash
source /home/sunrise/dog/roarm_ws/install/setup.bash
```

### Python 脚本没有执行权限

如果直接运行脚本失败：

```bash
chmod +x dog_visual_grasp_bridge/visual_moveit_grasp.py
chmod +x src/red_block_grasp_mplus0/red_block_grasp_mplus0/tools/calibrate_red_threshold.py
```

也可以用 `python3 script.py` 方式运行。

## 12. 面向二次开发的说明

初学者建议先看这些文件：

1. `src/red_block_grasp_mplus0/setup.py`：理解 ROS2 Python 节点如何注册为 `ros2 run` 命令。
2. `src/red_block_grasp_mplus0/launch/visual_servo_task.launch.py`：理解一个完整任务如何同时启动多个节点。
3. `src/red_block_grasp_mplus0/red_block_grasp_mplus0/nodes/roarm_driver_node.py`：理解机械臂命令 topic 的 JSON 协议。
4. `src/red_block_grasp_mplus0/red_block_grasp_mplus0/nodes/target_localizer_node.py`：理解相机、检测、深度定位、手眼转换。
5. `src/red_block_grasp_mplus0/red_block_grasp_mplus0/nodes/visual_servo_task_node.py`：理解抓取状态机。

常见修改入口：

| 想修改的内容 | 优先查看 |
|---|---|
| 机械臂串口、初始姿态、移动速度 | `launch/*.launch.py` 中 `roarm_driver_node` 和任务节点参数 |
| 红色阈值 | `config/red_color_calib.yaml`、`config_runtime/red_color_calib_tuned.yaml` |
| YOLO 模型 | `models/*.pt`、launch 参数 `model_path` |
| 手眼标定 | `handeye/handeye_cam_to_eef.json` |
| 目标定位输出 | `nodes/target_localizer_node.py`、`core/target_localizer.py` |
| 视觉伺服策略 | `nodes/visual_servo_task_node.py` |
| 夹爪角度、放置点 | `visual_servo_task.launch.py`、`competition_arm.yaml` |
| 比赛命令协议 | `nodes/competition_arm_task_node.py`、`config/competition_arm.yaml` |
| 狗臂 TCP 通信 | `nodes/dog_arm_tcp_server_node.py`、`docs/dog_arm_tcp_transport.md`、`config/competition_arm.yaml` |
| 官方 MoveIt2/RoArm 桥接 | `dog_visual_grasp_bridge/visual_moveit_grasp.py`、`dog_visual_grasp_bridge/config/grasp_config.yaml` |
| 新增 ROS2 节点 | 在 `red_block_grasp_mplus0/nodes/` 新增文件，并在 `setup.py` 的 `console_scripts` 注册 |

如果要新增 launch 文件，放在 `src/red_block_grasp_mplus0/launch/` 下即可，`setup.py` 已经通过 `glob("launch/*.launch.py")` 安装所有 launch 文件。

## 13. 后续待完善内容

根据当前仓库实际情况，建议后续补充：

- 真实机械臂现场接线、串口权限、RDK X5 系统版本说明；
- `roarm_msgs` 和官方 `roarm_ws` 的获取、编译、source 方法；
- Orbbec SDK / `pyorbbecsdk` 的平台安装步骤；
- 手眼标定采集和重新标定流程；
- 夹爪角度与实际开合距离的对应表；
- 抓取点偏置参数表和推荐调参顺序；
- 成功运行的视频、截图或 RViz/相机调试画面；
- 明确 `red_block_task.launch.py`、`task_manager_node.py` 是否保留为旧流程；
- 如果未来引入 URDF、Gazebo、rviz2、ros2_control 或 MoveIt2 配置，应新增对应目录和启动说明。
