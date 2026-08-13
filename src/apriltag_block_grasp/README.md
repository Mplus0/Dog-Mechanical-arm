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

## 阶段 1A：只检查 Orbbec 彩色取流

阶段 0 全部通过后，可以运行：

```bash
colcon build --packages-select apriltag_block_grasp --event-handlers console_direct+
source install/setup.bash

ros2 run apriltag_block_grasp color_camera_check_node --ros-args \
  -p show_window:=false
```

该节点只打开 Orbbec 彩色流，不读取深度、不检测 AprilTag、不连接机械臂，也不会发送
任何运动命令。它每隔约 2 秒发布并打印一次状态：

```text
/apriltag_grasp/camera_status
```

另一个终端可执行：

```bash
ros2 topic echo /apriltag_grasp/camera_status
```

本地有图形桌面时，可以单独测试窗口：

```bash
ros2 run apriltag_block_grasp color_camera_check_node --ros-args \
  -p show_window:=true
```

按 `q`、`Esc` 或在终端按 `Ctrl+C` 退出。退出日志应出现：

```text
Orbbec color stream stopped.
```

阶段 1A 通过标准：

- `valid=true` 持续发布；
- `width`、`height`、`format` 是稳定的实际值；
- `average_fps` 持续大于 0；
- `empty_frame_count` 和 `decode_failure_count` 不持续快速增加；
- `arm_connected=false`、`motion_commands_enabled=false`；
- `Ctrl+C` 后相机正常释放，节点可再次启动。

请反馈启动至稳定运行至少 10 秒的完整终端日志、一条
`/apriltag_grasp/camera_status` 消息，以及停止后第二次启动是否成功。窗口测试为可选项。

## 阶段 1B：只检查 tag25h9 二维检测

阶段 1A 通过后，将打印的 ID 0、1 标签放入相机画面并运行：

```bash
colcon build --packages-select apriltag_block_grasp --event-handlers console_direct+
source install/setup.bash

ros2 run apriltag_block_grasp apriltag_detection_2d_node --ros-args \
  -p show_window:=false \
  -p save_images:=false
```

另一个终端查看二维结果：

```bash
ros2 topic echo /apriltag_grasp/detections_2d
```

节点只检测 `tag25h9` 的 ID 0、1，发布每个标签的中心、四角点、像素面积和周长。
它不读取深度、不执行 PnP、不读取机械臂状态，也不会发送运动命令。

有图形桌面时可显示检测框、角点序号和中心点：

```bash
ros2 run apriltag_block_grasp apriltag_detection_2d_node --ros-args \
  -p show_window:=true \
  -p save_images:=false
```

如需保存标注图，显式设置：

```bash
-p save_images:=true
```

默认保存目录为：

```text
~/.ros/apriltag_block_grasp/detection_2d/
```

阶段 1B 需要分别测试：

1. 画面中无标签：`count=0`、`reason=no_allowed_tag`；
2. 只有 ID 0：`count=1` 且 `tag_id=0`；
3. 只有 ID 1：`count=1` 且 `tag_id=1`；
4. ID 0、1 同时出现：`count=2`，输出顺序为 0、1；
5. 将其他已打印 ID 放入画面：它们只能出现在 `ignored_ids`，不能进入 `detections`；
6. 标签靠近画面四周并改变观察距离/倾角，记录漏检范围和实际 FPS；
7. `Ctrl+C` 后相机释放，再次启动成功。

请反馈上述场景各一条 JSON 消息、持续运行日志、窗口截图（若有）以及实际可稳定识别
的距离和倾角范围。此阶段不判断标签姿态或物块坐标。

## 阶段 2A 前置：探测彩色相机标定参数

阶段 1B 通过后先运行标定参数探针：

```bash
colcon build --packages-select apriltag_block_grasp --event-handlers console_direct+
source install/setup.bash

ros2 run apriltag_block_grasp probe_color_calibration
```

该工具打开当前默认彩色流、读取一帧，然后尝试两条 SDK 标定参数路径：

```text
color_profile.get_intrinsic/get_distortion
pipeline.get_camera_param().rgb_intrinsic/rgb_distortion
```

每条路径的成功或失败都会写入 JSON。探针不会打开深度流、执行 PnP、连接机械臂或
发送动作，并在输出后立即释放相机。

只有满足以下条件才进入 PnP 实现：

- 至少一个来源同时提供内参和畸变；
- `fx`、`fy` 为正且所有数值有限；
- 标定分辨率为当前帧的 `848×530`，或者 SDK 未暴露标定分辨率但确认参数来自当前活动 profile；
- 输出 `ready_for_pnp=true`。

如果输出 `require_yaml_override=true`，不使用零畸变继续计算；应先获取与当前分辨率
对应的独立相机标定 YAML。

PnP 使用的 Tag 右手坐标系固定为：

```text
原点：有效黑色区域中心
+X：图案左 → 右
+Y：图案上 → 下
+Z：标签正面 → 标签背面（远离相机）
```

以标签纸下方说明文字正常可读作为“打印方向”基准：标签正对相机时，调试图中的红色
X 轴应向打印纸右侧，绿色 Y 轴应向打印纸下方。若将整张相机图像向右旋转 90°
观看，红色 X 轴应随之向下，绿色 Y 轴应随之向左。

请反馈探针的完整 JSON 输出。

## 阶段 2A：只检查 AprilTag PnP

标定探针输出 `ready_for_pnp=true` 后运行：

```bash
colcon build --packages-select apriltag_block_grasp --event-handlers console_direct+
source install/setup.bash

ros2 run apriltag_block_grasp apriltag_pnp_node --ros-args \
  -p show_window:=false \
  -p tag_size_mm:=38.9
```

查看完整结果：

```bash
ros2 topic echo --once --full-length \
  /apriltag_grasp/pnp std_msgs/msg/String
```

当前节点只输出 `T_camera_tag`：

- 优先 `IPPE_SQUARE`，失败时回退 `ITERATIVE`；
- 使用 SDK 的 `848×530` 彩色内参与五项 OpenCV 畸变系数；
- 输出相机坐标下的 `camera_tag_mm`、旋转矩阵、四元数和重投影误差；
- 不启用 RGBD 深度、手眼变换、机械臂状态或动作。

有图形桌面时可以核对坐标轴：

```bash
ros2 run apriltag_block_grasp apriltag_pnp_node --ros-args \
  -p show_window:=true \
  -p tag_size_mm:=38.9
```

请在标签正对相机时先测试，并记录：

1. ID 0、ID 1 和两者同时出现时的完整 JSON；
2. `method` 是否为 `IPPE_SQUARE`；
3. `camera_tag_mm.z` 是否为正，且随标签远离相机而增大；
4. 标签向图像右侧移动时 `camera_tag_mm.x` 是否增大；
5. 标签向图像下方移动时 `camera_tag_mm.y` 是否增大；
6. 正对相机时旋转矩阵是否接近单位阵；
7. `reprojection_error_px` 的实际范围；
8. 调试窗口中红 X、绿 Y、蓝 Z 轴方向是否符合文档约定。

此阶段尚未设定距离、面积和重投影误差拒绝阈值，只记录实测数据；不得据此控制机械臂。

## 阶段 2B 前置：探测 RGBD 对齐能力

阶段 2A 的坐标轴、尺度和 XYZ 趋势通过后运行：

```bash
colcon build --packages-select apriltag_block_grasp --event-handlers console_direct+
source install/setup.bash

ros2 run apriltag_block_grasp probe_rgbd_alignment
```

运行时让 ID 0 或 ID 1 保持在画面内。该探针只请求 Orbbec 软件深度到彩色对齐，
采集有限帧并读取标签中心 `5×5 px` 邻域的中位深度。它不执行 PnP、不连接机械臂、
不发布动作，也不会把 RGBD 深度作为目标位置来源。

请反馈完整 JSON，并确认：

1. `alignment.request_succeeded=true`；
2. 彩色和深度尺寸相同，`resolution_mismatch_count=0`；
3. `depth_scale_mm_values` 为有限正数；
4. `tag_center_depths` 中能观察到非空的 `median_depth_mm`；
5. `ready_for_depth_consistency_check=true`。

尺寸相同只是必要条件，不单独证明像素语义已经正确对齐。通过本探针后，再实现 PnP Z
与 RGBD 深度差的只读现场比较。

## 阶段 2B：只读比较 PnP 与 RGBD 深度

RGBD 对齐能力探针通过后运行：

```bash
colcon build --packages-select apriltag_block_grasp --event-handlers console_direct+
source install/setup.bash

ros2 run apriltag_block_grasp probe_pnp_depth_consistency
```

运行期间保持标签静止且正对相机。命令采集有限帧，使用相同彩色帧完成 AprilTag PnP，
同时读取对齐深度图中标签中心 `5×5 px` 邻域的中位深度。输出中的
`pnp_minus_rgbd_mm = pnp_z_mm - rgbd_depth_mm`。

请先分别在约 `180 mm`、`250 mm`、`350 mm` 三个距离运行一次并反馈完整 JSON。
这一小步只收集每个 ID 的差值分布，`depth_rejection_threshold_enabled=false`，不会根据
单次结果选择阈值或拒绝 PnP，也不会连接机械臂或发送动作。

## 阶段 3A：只读检查机械臂末端位姿

深度检查完成后，先独立验证官方机械臂位姿服务。启动提供 `/get_pose_cmd` 的官方
RoArm 节点，让机械臂保持静止，然后运行：

```bash
colcon build --packages-select apriltag_block_grasp --event-handlers console_direct+
source install/setup.bash

ros2 run apriltag_block_grasp probe_arm_pose
```

探针默认读取 20 次位姿。官方服务的位置按米读取并乘 `1000` 转成毫米，姿态按弧度读取，
使用与现有项目一致的 `Rz(yaw) @ Ry(pitch) @ Rx(roll)` 构造 `T_base_eef`。
它不打开相机、不加载手眼矩阵，也不创建或发送任何运动命令。

请反馈完整 JSON，并检查：

1. `summary.valid=true`，20 次请求全部成功；
2. `pose_mm_rad` 的位置数量级符合当前机械臂实际位姿；
3. 静止时 `stability` 中 XYZ 和三个角度的 `peak_to_peak` 足够小；
4. 每个旋转矩阵的行列式接近 `1`，正交误差接近 `0`。

此项通过后，下一小步才会把已确认的 `T_eef_camera` 复制到新包并执行只读坐标链计算。

### 默认方案：直接读取 RoArm 串口状态

如果官方 `/get_pose_cmd` 服务未运行，项目默认采用独立 Python 串口封装。确保没有其他
节点或程序占用机械臂串口，让机械臂保持静止，然后运行：

```bash
ros2 run apriltag_block_grasp probe_arm_serial_state \
  --port /dev/ttyUSB0 \
  --sample-count 20
```

该探针只读取机械臂主动上报的 `T=1051` JSON。只读串口类没有发送方法，运行期间
`serial_bytes_transmitted=0`，不会发送 LED、关节、末端或夹爪命令，也不会打开相机。

探针会列出实际收到的全部字段。只有状态同时包含 `x/y/z/r/b` 和 `tit` 或 `t`，且数值
有限时，才按当前手眼标定所用约定生成候选 `T_base_eef`。请反馈完整 JSON，确认
`summary.valid=true`、20 帧均有效，并检查静止状态下 `pose_stability` 的波动。
连接后的启动阶段默认允许最多 5 次空读；空读不会作为状态样本，也不会复用旧状态。

## 阶段 3B：只读验证手眼坐标链

机械臂串口状态探针通过后，让机械臂和标签都保持静止，确保没有其他程序占用相机或
机械臂串口，然后运行：

```bash
colcon build --packages-select apriltag_block_grasp --event-handlers console_direct+
source install/setup.bash

ros2 run apriltag_block_grasp probe_handeye_chain \
  --port /dev/ttyUSB0 \
  --sample-count 20 \
  --wait-for-ready
```

RoArm-M3 的 ESP32 USB 串口在打开时可能通过 DTR/RTS 自动下载电路触发控制板复位；
已观察到 OLED 刷新，但常驻只读驱动实测没有观察到固定下降。此前固定下降与显式运动命令
强相关，不能再解释为串口打开后的必然初始回位。指定 `--wait-for-ready` 后，探针先打开且
持续占用串口，等待控制器稳定并提示按 Enter。此时再调整到观察姿态、固定标签和物块，最后按
Enter。探针会立即清空等待和调整期间积压的旧 `T=1051` 状态帧，再启动相机；后续
采样不会重新打开串口，也不会把调整前的机械臂状态与调整后的相机图像配对。

等待期间可以使用不占机械臂串口的控制方式调整姿态。按 Enter 前必须停止其他占用
Orbbec 相机的程序。不要同时启动另一个访问 `/dev/ttyUSB0` 的进程。

该探针使用新包内部独立安装的 `config/handeye_cam_to_eef.json` 和
`config/tag_to_object.json`，计算：

```text
T_base_tag = T_base_eef @ T_eef_camera @ T_camera_tag
T_base_object = T_base_tag @ T_tag_object
```

当前 `T_tag_object` 使用现场确认值：

```text
translation_mm = [0.0, -77.0, -25.0]
rotation_rpy_deg = [180.0, 0.0, -90.0]
```

探针同时输出标签中心 `base_tag_mm` 和物块几何中心 `base_object_mm`。它不应用
`base_position_correction_mm` 或旧包的 `base Z + 100 mm`。RGBD 深度关闭，机械臂
串口数据方向仍为只读且不发送命令；但如上所述，首次打开串口可能刷新/复位控制板。

请先反馈 `summary`、`failure_counts`、`per_id_stability` 和最后一条 `samples`。首轮只
判断静止状态下坐标链是否有限、矩阵方向是否合理，以及 `base_tag_mm`、
`base_object_mm` 的波动；不把这些坐标用于机械臂动作。

### 暂定抓取几何配置

`config/grasp_calibration.json` 记录首次人工最终夹取对准结果，但不会被当前只读探针用于
运动。固定关系为：

```text
P_base_final_grasp_tcp
= P_base_object
+ base_position_correction_mm
+ final_grasp_tcp_offset_base_mm
```

当前 `final_grasp_tcp_offset_base_mm = [-24.058, 33.017, 0.676]`，末端固定姿态
RPY为 `[0.012272, 1.713457, -0.062893] rad`。夹爪是左指固定、右指活动的非对称结构，
所以TCP不要求位于物块几何中心。夹爪 `g=2.408350 rad` 仅对应“能够夹住但不紧”的
暂定状态。上述参数仍需多位置复测和夹爪专项测试，不允许据此直接开放自动下降或闭爪。

### 受限 B 关节观察角移动工具

手眼链跨位姿验证仅使用 B 关节小角度运动。工具只允许发送一条绝对 B 关节命令，默认
范围为 `[-20°, +20°]`、单次最大变化 `10°`。不指定 `--enable-motion` 时只做演练，
不会发送命令。

先在机械臂周围清空人员和物品，执行演练：

```bash
ros2 run apriltag_block_grasp move_b_joint_safe \
  --port /dev/ttyUSB0 \
  --target-b-deg 5.0
```

确认输出中的当前角度、目标角度、变化量和 `planned_command` 正确后，才能显式启用运动：

```bash
ros2 run apriltag_block_grasp move_b_joint_safe \
  --port /dev/ttyUSB0 \
  --target-b-deg 5.0 \
  --enable-motion
```

默认命令为 `T=121, joint=1, spd=10, acc=10`。工具发送一条命令后只读取 `T=1051`
反馈，连续 3 帧进入 `±1°` 才报告到位；超时不会自动发送第二条命令或回退。相机、
夹爪、其他关节和补光灯均不受控制。
连接后的初始状态默认最多读取 5 次，每次等待 1 秒；空读不会触发任何状态请求命令。

### 单命令 B 关节反馈轨迹诊断

如果单次 B 关节命令出现欠位或随后回落，使用独立轨迹工具在同一个串口连接中记录
反馈。工具仍然默认只演练；显式启用后只发送一次 `T=121, joint=1`，持续记录 B 角，
不会重发、补偿或回退，也不会控制其他关节、夹爪、相机或补光灯。

先执行演练：

```bash
ros2 run apriltag_block_grasp trace_b_joint_motion \
  --port /dev/ttyUSB0 \
  --target-b-deg 5.0
```

确认当前角度和计划命令后，清空机械臂周围空间并显式启用一次运动：

```bash
ros2 run apriltag_block_grasp trace_b_joint_motion \
  --port /dev/ttyUSB0 \
  --target-b-deg 5.0 \
  --trace-duration-s 15.0 \
  --enable-motion
```

输出中的 `trace` 默认每 0.25 秒保留一个 B 角和 `tB`，同时汇总最接近目标的时刻、
观测到的角度范围、最终误差和所有关节的起止差值。诊断结束后保持最终实际姿态，
不会自动回到起点。

### 同一连接内的成对手眼验证

如果两个独立程序之间的 B 反馈发生变化，使用成对探针保持相机和机械臂串口连接，依次
完成基准采样、一次 B 命令和第二组采样。必须指定唯一标签 ID；基准组没有完整采到时，
工具不会发送运动命令。默认仍为演练，只采基准并显示计划命令：

```bash
ros2 run apriltag_block_grasp probe_handeye_pair_b \
  --port /dev/ttyUSB0 \
  --tag-id 1 \
  --target-b-deg 5.0 \
  --sample-count 20
```

确认基准、当前 B 和计划变化量后，才可显式启用：

```bash
ros2 run apriltag_block_grasp probe_handeye_pair_b \
  --port /dev/ttyUSB0 \
  --tag-id 1 \
  --target-b-deg 5.0 \
  --sample-count 20 \
  --enable-motion
```

显式启用后最多发送一条 `T=121, joint=1`，不会重试或恢复。输出直接给出两组
`base_tag` 中位数及三轴差值和差值范数；只用于验证手眼链，不用于抓取动作。

### 常驻只读机械臂驱动验证

`roarm_driver_node` 是新包内部独立实现的第一版常驻驱动。当前版本只用于区分“打开串口
导致的控制器/OLED复位”和“显式运动命令导致的机械臂动作”：

- 节点生命周期内只打开一次 `/dev/ttyUSB0`；
- 持续读取 `T=1051` 并发布 `/roarm_m3/state`；
- 订阅 `/roarm_m3/cmd`，但拒绝所有收到的命令；
- 串口类没有写接口，`serial_bytes_transmitted` 始终应为 `0`；
- 读取失败后不自动重连，避免再次触发控制器复位；
- 不发送初始姿态、LED、夹爪或任何运动命令。

启动前必须停止旧功能包驱动、所有串口探针和网页中可能占用 `/dev/ttyUSB0` 的连接。同一时刻
只允许本节点占用机械臂串口：

```bash
ros2 run apriltag_block_grasp roarm_driver_node \
  --ros-args -p port:=/dev/ttyUSB0
```

首次打开串口仍可能使 ESP32/OLED 复位，但首轮实测没有观察到固定下降。保持节点连续运行
至少 60 秒，不启动
其他机械臂程序，也不向命令话题发布消息。在另一个终端检查：

```bash
ros2 topic echo --once --full-length /roarm_m3/state std_msgs/msg/String
```

验收时应看到 `connected=true`、`state_valid=true`、`serial_open_count=1`、
`serial_bytes_transmitted=0`、`motion_commands_enabled=false`，且持续运行期间没有机械臂动作。
当前证据表明此前的固定下降与显式运动命令强相关，不是单纯打开串口造成。按 `Ctrl-C` 停止
节点会关闭串口；再次启动会重新打开串口并可能再次引起 ESP32/OLED 复位。

常驻只读驱动通过后，使用 `probe_roarm_model` 离线检查已安装的官方模型和 MoveIt 配置：

```bash
source /home/sunrise/dog/roarm_ws/install/setup.bash
source /home/sunrise/dog/ros2_red_block_ws/install/setup.bash
ros2 run apriltag_block_grasp probe_roarm_model
```

该工具只读取 `roarm_description` 和 `roarm_moveit` 的安装目录，列出 URDF/Xacro/SRDF 中可直接
解析的关节、父子连杆、关节限制及相关配置行。不创建 ROS 节点、不连接串口/相机、不调用
服务，也不启动任何官方 launch。若 Xacro 中仍含表达式，工具只原样报告，不为其猜测数值。

#### 常驻连接内的原位保持诊断（已停用）

实测已完成，不得重复。测试在同一常驻串口连接内，把稳定 `T=1051` 反馈中的
`x/y/z/tit/r/g` 原样复制为一条 `T=1041`，但机械臂在约 0.54 秒内移动到另一姿态：
`Δx=+5.864 mm`、`Δy=+0.054 mm`、`Δz=-20.310 mm`、`Δtit=+0.058291 rad`
（约 `+3.340°`），随后保持稳定。该结果证明在当前固件和接口路径中，`T=1051` 的
笛卡尔反馈不能作为 `T=1041` 的等价原位目标。驱动现已硬拒绝该诊断，即使设置
`enable_diagnostic_hold_test:=true` 也不会发送命令。

### 固定观察姿态单次验收

本工具独立采用旧功能包已实机成功的 `T=121` 关节控制路径。根据现场确认，观察动作只发送
`B=0°、S=0°、E=70°`，顺序为 `B→S→E`，速度和加速度均为 `35`。不发送 T、R、夹爪、
笛卡尔 XYZ、相机或补光灯命令，T/R 关节保持动作开始前的值。节点不会自动运动。

先关闭其他占用 `/dev/ttyUSB0` 的程序，移开物块并清空机械臂运动范围。先执行演练：

```bash
ros2 run apriltag_block_grasp move_observation_pose_safe \
  --port /dev/ttyUSB0
```

确认 JSON 中三条 `planned_commands`、`gripper_commanded=false`、
`cartesian_commanded=false` 和 `motion_command_sent=false` 后，才执行一次实机验收：

```bash
ros2 run apriltag_block_grasp move_observation_pose_safe \
  --port /dev/ttyUSB0 \
  --enable-motion \
  --confirmation I_ACCEPT_OBSERVATION_POSE_MOTION
```

工具沿用旧包的 `3.0 s` 定时等待，记录 B/S/E 最终误差，以及未控制的 T/R 和夹爪前后差值，
不擅自设定关节精度阈值。
请观察实际相机是否到达正式识别角度并反馈完整 JSON；本轮不继续发送 XYZ、夹爪或抓取命令。

### 单次笛卡尔 XYZ 小步安全工具

`move_cartesian_fixed_orientation_safe` 用一条 RoArm-M3 `T=1041` 命令验证 XYZ 小步运动。
固件命令字段为 `x/y/z/t/r/g`：它可以指定 TCP 的 XYZ、工具俯仰 `t`、工具滚转
`r` 和夹爪 `g`，但没有独立的 yaw/B 字段。B 角由固件逆运动学根据目标位置确定，因此该工具
不能承诺在不同 XYZ 下保持完整 RPY 不变。

工具默认只演练，不发送命令。默认 `--orientation-source current`：读取新鲜 `T=1051` 后，
将当前 `tit/r/g` 原样复制到命令，仅在当前 XYZ 上增加 `dx/dy/dz`。默认每轴和三维总位移
都不超过 5 mm，只允许发送一条命令，不重试、不恢复，也不控制相机和补光灯。
`--orientation-source calibration` 仍保留用于后续姿态复现，但当前阶段不要使用。

首次测试必须移开物块并清空机械臂周围空间。先编译并演练 `X + 5 mm`：

```bash
colcon build --packages-select apriltag_block_grasp --event-handlers console_direct+
source install/setup.bash

ros2 run apriltag_block_grasp move_cartesian_fixed_orientation_safe \
  --port /dev/ttyUSB0 \
  --dx-mm 5.0
```

确认 JSON 中：

1. `orientation_source=current`；
2. `requested_delta_xyz_mm` 为 `[5, 0, 0]`；
3. `target_xyz_mm.x = initial_state.x + 5`，Y/Z 不变；
4. `planned_command.T=1041`；
5. `planned_command.t/r/g` 分别等于当前反馈 `tit/r/g`；
6. `pitch_change_deg=0`、`roll_change_deg=0`；
7. `motion_command_sent=false`。

直接笛卡尔串口实机发送现已停用。旧 `T=104` 的两次 `X +/-5 mm` 测试均沿 Z 下降约
24--25 mm。改用当前官方 RoArm-M3 `T=1041` 后，`X +5 mm` 测试从
`[364.823, 3.358, 160.786] mm` 运动到 `[375.705, 3.458, 131.366] mm`，仍未保持目标
Z/pitch。该测试目标距 base 原点约 403.3 mm，而最终反馈距原点约 398.0 mm，存在目标
接近或超出当前姿态可达边界的可能；这只是待验证假设，不能据此判定 `T=1041` 协议错误。
在核对 RoArm-M3 几何、关节限制和逆解返回前，即使指定 `--enable-motion`，工具也会在打开
串口之前拒绝执行；不要继续使用以下测试命令：

```bash
ros2 run apriltag_block_grasp move_cartesian_fixed_orientation_safe \
  --port /dev/ttyUSB0 \
  --dx-mm 5.0 \
  --enable-motion
```

官方 ROS 2/MoveIt 接口探针已经确认三个服务类型均已安装，但当前 ROS 图中没有对应服务
提供者。官方 `command_control.launch.py` 启动时会主动将前臂伸展到水平姿态，因此不得在
当前现场姿态下直接启动。下一步先离线核对已安装的 RoArm-M3 模型、关节限制和目标可达性，
不连接机械臂：

```bash
ros2 run apriltag_block_grasp probe_official_motion_interfaces
```

该探针不打开串口、不创建 service client，也不发送 service request；它只列出已安装的
`GetPoseCmd`、`MoveJointCmd`、`MoveLineCmd` 请求/响应字段，并读取 ROS 图中对应服务是否已存在。
接口探针结果与离线可达性检查共同决定后续采用 `/move_line_cmd`、`/move_joint_cmd`，还是
经过可达性检查的 `T=1041` 直控。

在官方接口验证完成前，不再进行任何 XYZ 实机测试。

### PnP 多解与深度只读诊断

当改变视角后 PnP 距离与固定标签不一致时，可在当前静止视角比较 IPPE 的全部候选解、
ITERATIVE、SQPNP、标定畸变与零畸变诊断结果，同时记录四条标签像素边长和对齐后的
中心深度：

```bash
ros2 run apriltag_block_grasp probe_pnp_solutions \
  --tag-id 1 \
  --frame-count 30
```

该工具不连接机械臂，也不发送运动命令。零畸变和 RGBD 深度只用于诊断，不会替换正式
定位源或修改标定参数。

多解实测表明当前 Orbbec 848x530 彩色帧已经过镜头校正：使用 SDK 返回的强畸变系数
会把 ID 1 的 PnP Z 从约 388.5 mm 错算为约 215 mm，并把重投影误差从约 0.12 px
增大到约 2.28 px。因此正式 PnP 默认采用 `rectified_zero_distortion`，但仍保留
`sdk_calibrated_distortion` 作为显式诊断模式。节点输出中的 `pnp_distortion_mode` 和
`pnp_distortion_coefficients` 表示求解实际使用的值，不等同于 SDK 中仍可读取的镜头参数。
