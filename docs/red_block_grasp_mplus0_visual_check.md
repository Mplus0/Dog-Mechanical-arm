# red_block_grasp_mplus0 视觉功能检查流程

本文用于在 RDK X5 机械臂主控上验证二次开发功能包 `red_block_grasp_mplus0` 的视觉部分，检查范围包括：

- ROS2 功能包、模型和手眼标定文件是否正确安装；
- Orbbec 彩色相机是否能取流；
- YOLO 是否能识别红色目标并输出二维检测框；
- RGBD、颜色检测、YOLO 融合和坐标转换是否能输出有效三维目标坐标；
- 目标静止时，识别结果是否连续、稳定。

本流程**不会启动抓取、下降、夹爪、放置或比赛任务节点**。

## 1. 测试前安全检查

1. 将机械臂放在安全、无遮挡的位置，确保急停或断电手段可用。
2. 关闭之前运行的以下节点或 launch：
   - `visual_servo_task_node`
   - `open_loop_grasp_task_node`
   - `task_manager_node`
   - `competition_arm_task_node`
   - 旧功能包的相机和定位节点
3. 确保同一时刻只有一个程序占用 Orbbec 相机。
4. 如果准备启动本项目的 `roarm_driver_node`，还要确保没有其他程序占用 `/dev/ttyUSB0`。

先查看当前 ROS2 节点：

```bash
ros2 node list
```

正常情况下，开始本次检查前不应看到上述动作节点。若看到旧的相机、YOLO 或定位节点，应先在启动它们的终端按 `Ctrl+C` 停止。

> 注意：`localization_only.launch.py` 不启动任何机械臂动作状态机。不过，当 `start_roarm_driver:=true` 时，驱动连接串口后会发送一次关闭补光灯的命令，不会主动发送位姿或关节运动命令。

## 2. 设置工作空间环境

以下命令假设主控中的工作空间仍为：

```text
/home/sunrise/dog/ros2_red_block_ws
```

每打开一个新终端，都执行：

```bash
cd /home/sunrise/dog/ros2_red_block_ws
source /opt/ros/humble/setup.bash
source /home/sunrise/dog/roarm_ws/install/setup.bash
source install/setup.bash
```

不要只执行仓库根目录的 `source_red_block.sh`。该脚本当前仍只手工加入第一版 `red_block_grasp_ros2` 的安装路径，不能保证新包被加载。

检查当前加载到的是哪个安装目录：

```bash
ros2 pkg prefix red_block_grasp_mplus0
```

正常结果应类似：

```text
/home/sunrise/dog/ros2_red_block_ws/install/red_block_grasp_mplus0
```

如果提示 `Package 'red_block_grasp_mplus0' not found`，先执行下一节的编译步骤。

## 3. 编译并检查安装内容

在工作空间根目录执行：

```bash
cd /home/sunrise/dog/ros2_red_block_ws
source /opt/ros/humble/setup.bash
source /home/sunrise/dog/roarm_ws/install/setup.bash

# 必须识别为 ros.ament_python；如果显示 (python)，不要继续构建。
colcon list | grep red_block_grasp_mplus0

colcon build --packages-select red_block_grasp_mplus0 --event-handlers console_direct+
source install/setup.bash
```

正确的构建类型应为：

```text
red_block_grasp_mplus0  src/red_block_grasp_mplus0  (ros.ament_python)
```

如果显示 `(python)`，说明 `package.xml` 没有通过 ROS 清单解析。可使用下面的命令查看
具体错误，不要把普通 Python 构建的“成功”误认为 ROS2 包安装成功：

```bash
colcon --log-level debug list 2>&1 | \
  grep -E 'red_block_grasp_mplus0|Failed to parse|ERROR|Exception' | \
  head -n 100
```

修正清单后，应删除此前按普通 Python 类型生成的这个新包的构建和安装目录，再重新
构建。不要删除整个工作空间的 `build/` 或 `install/`：

```bash
rm -rf \
  build/red_block_grasp_mplus0 \
  install/red_block_grasp_mplus0

colcon build --packages-select red_block_grasp_mplus0 --event-handlers console_direct+
source install/local_setup.bash
```

这里不使用 `--symlink-install`。部分主控环境中的 `colcon` 与较新版本
`setuptools` 在可编辑安装方式上不兼容，会在调用 `setup.py` 时出现：

```text
error: option --editable not recognized
```

该错误发生在 Python 包的符号链接/可编辑安装阶段，不是本项目视觉源码、模型或
ROS2 节点的错误。出现此错误时，直接去掉 `--symlink-install`，使用上面的普通编译
命令重新执行；暂时不要为此升级或降级主控的系统 Python 包。

如果去掉 `--symlink-install` 后仍然出现同一错误，再检查实际执行的命令和环境版本：

```bash
history | tail -n 10
python3 -c "import setuptools; print('setuptools:', setuptools.__version__)"
colcon --help | head
```

正常情况下，结尾应包含类似信息：

```text
Finished <<< red_block_grasp_mplus0
Summary: 1 package finished
```

检查节点入口：

```bash
ros2 pkg executables red_block_grasp_mplus0
```

正常情况下至少应看到：

```text
red_block_grasp_mplus0 yolo_camera_node
red_block_grasp_mplus0 target_localizer_node
red_block_grasp_mplus0 roarm_driver_node
red_block_grasp_mplus0 visual_servo_task_node
```

检查模型、配置和手眼标定是否已安装：

```bash
PKG_SHARE="$(ros2 pkg prefix --share red_block_grasp_mplus0)"
echo "$PKG_SHARE"
ls -lh "$PKG_SHARE/models"
ls -lh "$PKG_SHARE/handeye"
ls -lh "$PKG_SHARE/config"
```

正常情况下应能找到：

```text
models/red_block_yolo11n.pt
models/red_block_yolo11n_v3_mix.pt
handeye/handeye_cam_to_eef.json
config/red_color_calib.yaml
```

检查关键 Python 依赖：

```bash
python3 -c "import cv2, numpy, ultralytics, pyorbbecsdk; print('visual python dependencies: OK')"
ros2 interface show roarm_msgs/srv/GetPoseCmd
```

正常情况下第一条命令输出：

```text
visual python dependencies: OK
```

第二条命令应打印 `GetPoseCmd` 服务定义，而不是提示接口不存在。

## 4. 第一步：只检查彩色相机和 YOLO 二维识别

这一步不连接机械臂、不读取深度、不进行坐标转换，是风险最低的视觉检查。

### 4.1 启动二维识别节点

终端 A：

```bash
cd /home/sunrise/dog/ros2_red_block_ws
source /opt/ros/humble/setup.bash
source /home/sunrise/dog/roarm_ws/install/setup.bash
source install/setup.bash

PKG_SHARE="$(ros2 pkg prefix --share red_block_grasp_mplus0)"
ros2 run red_block_grasp_mplus0 yolo_camera_node --ros-args \
  -p model_path:="$PKG_SHARE/models/red_block_yolo11n_v3_mix.pt" \
  -p show_window:=false \
  -p conf_thres:=0.35 \
  -p timer_period:=0.2
```

通过 SSH 或没有桌面环境时保持 `show_window:=false`。如果主控连接了显示器并且图形桌面可用，可以改成：

```bash
-p show_window:=true
```

正常启动时，终端 A 应依次出现类似日志：

```text
Starting Orbbec camera...
Orbbec color camera started.
Loading YOLO model...
YOLO camera node started.
```

Ultralytics 第一次加载模型时可能额外输出模型或运行环境信息，这属于正常现象。

### 4.2 检查二维识别消息

在相机前放置训练目标类型的红色物块。终端 B 执行：

```bash
cd /home/sunrise/dog/ros2_red_block_ws
source /opt/ros/humble/setup.bash
source /home/sunrise/dog/roarm_ws/install/setup.bash
source install/setup.bash

ros2 topic list | grep red_block
ros2 topic info /red_block/detections
ros2 topic echo --once /red_block/detections
```

正常情况下：

- topic 类型为 `std_msgs/msg/String`；
- 没有红色目标时，JSON 中通常为 `"count": 0`；
- 成功识别红色目标时，`count` 大于 0；
- `detections` 中包含置信度、检测框和中心像素。

成功识别时的消息结构类似：

```yaml
data: '{"stamp": 1234567890.0, "frame_id": "camera_color", "count": 1,
  "detections": [{"id": 0, "confidence": 0.82, "bbox": {...}, "center": {...}}]}'
```

实际字段显示顺序和数值可能不同。判断成功的关键是：

```text
count >= 1
confidence >= 0.35
检测框和中心点随红色目标位置变化
```

检查发布频率：

```bash
ros2 topic hz /red_block/detections
```

正常情况下应持续输出频率统计，而不是长时间显示没有新消息。实际 FPS 取决于 X5 负载、模型和相机帧率，不要求达到固定数值。

完成后，在终端 A 按 `Ctrl+C`，确认相机已释放。正常退出时可能看到：

```text
Orbbec color camera stopped.
```

## 5. 第二步：检查 RGBD 融合识别和三维定位

三维定位除了相机和目标深度，还需要当前机械臂位姿。请根据现场实际控制链路，在下面两种方式中选择一种，**不要同时启动两个机械臂驱动**。

测试时建议把红色目标放在相机可见且深度约为 `100～700 mm` 的范围内，并先保持机械臂和目标静止。

### 5.1 方式 A：由新功能包连接机械臂串口

适用于 `/dev/ttyUSB0` 没有被官方驱动或其他程序占用的情况。

终端 A：

```bash
cd /home/sunrise/dog/ros2_red_block_ws
source /opt/ros/humble/setup.bash
source /home/sunrise/dog/roarm_ws/install/setup.bash
source install/setup.bash

PKG_SHARE="$(ros2 pkg prefix --share red_block_grasp_mplus0)"
ros2 launch red_block_grasp_mplus0 localization_only.launch.py \
  start_roarm_driver:=true \
  arm_port:=/dev/ttyUSB0 \
  arm_state_source:=dog_arm_topic \
  model_path:="$PKG_SHARE/models/red_block_yolo11n_v3_mix.pt" \
  handeye_path:="$PKG_SHARE/handeye/handeye_cam_to_eef.json" \
  color_calib_path:="$PKG_SHARE/config/red_color_calib.yaml" \
  detector_mode:=fusion \
  show_window:=false
```

正常启动日志应包含类似内容：

```text
RoArm-M3 connected.
RoArm driver node started.
Starting RGBD camera...
Depth align mode: SW_MODE
Orbbec RGBD camera started.
Loading YOLO model...
Target localizer started. mode=fusion, ... arm_state_source=dog_arm_topic
```

### 5.2 方式 B：复用已经运行的官方 RoArm 服务

适用于官方驱动已经占用串口，并且 `/get_pose_cmd` 服务可用的情况。

先检查服务：

```bash
ros2 service list | grep get_pose_cmd
ros2 service type /get_pose_cmd
```

正常情况下应看到：

```text
/get_pose_cmd
roarm_msgs/srv/GetPoseCmd
```

然后启动定位，明确禁止新包再次占用串口：

```bash
cd /home/sunrise/dog/ros2_red_block_ws
source /opt/ros/humble/setup.bash
source /home/sunrise/dog/roarm_ws/install/setup.bash
source install/setup.bash

PKG_SHARE="$(ros2 pkg prefix --share red_block_grasp_mplus0)"
ros2 launch red_block_grasp_mplus0 localization_only.launch.py \
  start_roarm_driver:=false \
  arm_state_source:=official_get_pose_cmd \
  official_get_pose_service:=/get_pose_cmd \
  model_path:="$PKG_SHARE/models/red_block_yolo11n_v3_mix.pt" \
  handeye_path:="$PKG_SHARE/handeye/handeye_cam_to_eef.json" \
  color_calib_path:="$PKG_SHARE/config/red_color_calib.yaml" \
  detector_mode:=fusion \
  show_window:=false
```

正常启动日志应包含：

```text
Starting RGBD camera...
Orbbec RGBD camera started.
Loading YOLO model...
Target localizer started. mode=fusion, ... arm_state_source=official_get_pose_cmd
```

不应持续出现：

```text
official_get_pose_cmd unavailable
```

### 5.3 检查机械臂状态和三维目标消息

使用方式 A 时，终端 B 先检查机械臂状态：

```bash
ros2 topic echo --once /roarm_m3/state
```

正常结果应包含：

```json
{
  "connected": true,
  "state_valid": true,
  "state": {
    "x": "数值",
    "y": "数值",
    "z": "数值"
  }
}
```

随后，两种方式都执行：

```bash
ros2 topic info /red_block/target_base
ros2 topic echo --once /red_block/target_base
```

相机正常但没有找到目标时，节点仍会发布消息，例如：

```json
{
  "valid": false,
  "reason": "no_detection",
  "source": null,
  "base_mm": null
}
```

这说明相机和节点正在运行，但当前帧没有通过筛选的目标，不能据此判断三维定位已经成功。

红色目标被正确识别、深度有效且机械臂状态有效时，应看到类似结构：

```json
{
  "valid": true,
  "stable": true,
  "reason": "ok",
  "source": "fusion或颜色/YOLO来源",
  "confidence": 0.8,
  "pixel": {"x": 420, "y": 265},
  "depth_mm": 350.0,
  "camera_mm": {"x": "数值", "y": "数值", "z": "数值"},
  "base_mm": {"x": "数值", "y": "数值", "z": "数值"}
}
```

第一次识别后的前几帧中，`valid` 可能已经为 `true`，但 `stable` 暂时为 `false`；目标静止并连续识别约 3 帧后，正常情况下应变成：

```text
valid: true
stable: true
reason: ok
depth_mm: 100～700 范围内的合理数值
base_mm.x/y/z: 有限数值，不是 null、NaN 或无限大
```

持续观察稳定性：

```bash
ros2 topic echo /red_block/target_base
```

观察约 10～20 秒后按 `Ctrl+C`。目标和机械臂静止时，正常表现是：

- 消息持续发布；
- `valid` 大多数时间为 `true`；
- `pixel` 位于实际红色目标附近；
- `depth_mm` 不发生大幅跳变；
- `base_mm` 在小范围内波动，不应在不相关坐标间跳跃；
- 偶发丢帧时可能出现 `is_hold: true`，持续时间应较短，随后恢复真实检测。

检查消息频率：

```bash
ros2 topic hz /red_block/target_base
```

正常情况下会持续输出统计。默认调度周期为 `0.08 s`，但实际频率会受 RGBD 取流、YOLO 推理和主控负载影响，因此不要求严格达到 `12.5 Hz`。

## 6. 可选：显示调试画面

只有主控有本地图形桌面或正确配置了 X11 转发时，才使用：

```bash
ros2 launch red_block_grasp_mplus0 localization_only.launch.py \
  start_roarm_driver:=true \
  arm_state_source:=dog_arm_topic \
  detector_mode:=fusion \
  debug_overlay_level:=compact \
  show_window:=true
```

其余模型、标定和颜色配置参数可按第 5 节补充。

正常画面应显示：

- 红色目标检测框和中心点；
- `valid: True`、`reason: ok`；
- 检测来源和置信度；
- 像素坐标、深度和 `base_mm`；
- `lock`、`hold` 和 FPS 状态。

按窗口中的 `q`、`Esc`，或在启动终端按 `Ctrl+C` 退出。

## 7. 可选：识别异常时隔离检测模式

以下测试一次只运行一个定位 launch，并保证相机未被其他节点占用。

只检查 YOLO：

```bash
ros2 launch red_block_grasp_mplus0 localization_only.launch.py \
  start_roarm_driver:=true \
  arm_state_source:=dog_arm_topic \
  detector_mode:=yolo \
  show_window:=false
```

只检查颜色检测：

```bash
PKG_SHARE="$(ros2 pkg prefix --share red_block_grasp_mplus0)"
ros2 launch red_block_grasp_mplus0 localization_only.launch.py \
  start_roarm_driver:=true \
  arm_state_source:=dog_arm_topic \
  detector_mode:=color \
  color_calib_path:="$PKG_SHARE/config/red_color_calib.yaml" \
  show_window:=false
```

恢复正式融合模式：

```bash
ros2 launch red_block_grasp_mplus0 localization_only.launch.py \
  start_roarm_driver:=true \
  arm_state_source:=dog_arm_topic \
  detector_mode:=fusion \
  show_window:=false
```

如果官方驱动已占用串口，应把上述命令统一改为：

```text
start_roarm_driver:=false arm_state_source:=official_get_pose_cmd
```

## 8. 测试通过标准

满足以下条件即可认为新功能包的视觉基础链路能够正常工作：

1. `colcon build` 成功，ROS2 能找到 `red_block_grasp_mplus0`。
2. 安装目录中存在两个模型、手眼标定和颜色配置。
3. `yolo_camera_node` 能持续发布 `/red_block/detections`。
4. 放入正确红色目标后，二维消息中 `count >= 1`，检测框位置正确。
5. `localization_only.launch.py` 能启动 RGBD 相机和融合检测器。
6. 机械臂位姿源有效时，`/red_block/target_base` 能输出 `valid: true`、合理深度和有限的 `base_mm`。
7. 目标静止时可以进入 `stable: true`，结果没有明显跳变。
8. 整个测试期间机械臂没有收到或执行抓取、下降、夹爪和放置动作。

## 9. 出现问题时需要保留的信息

再次反馈问题时，请尽量提供以下内容，避免只截取最后一行报错：

```bash
git branch --show-current
git log -1 --oneline
ros2 pkg prefix red_block_grasp_mplus0
ros2 node list
ros2 topic list
ros2 topic info /red_block/target_base
```

还请一并说明：

- 使用的是第 4 节二维检查，还是第 5 节三维定位检查；
- 完整启动命令；
- 从启动开始到报错为止的完整终端日志；
- `/red_block/detections` 或 `/red_block/target_base` 的一条实际消息；
- 是否有官方 RoArm 驱动占用 `/dev/ttyUSB0`；
- 是否使用显示窗口、SSH 或本地桌面；
- 红色目标大致距离、光照和画面位置；
- 是否出现相机被占用、模型找不到、机械臂状态无效或深度无效等现象。

完成所有测试后，在启动节点的终端按 `Ctrl+C`。如果使用了第 5.1 节的新包驱动，还应确认 `roarm_driver_node` 和 `target_localizer_node` 均已退出：

```bash
ros2 node list
```
