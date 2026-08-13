# AprilTag 物块抓取功能包开发流程

## 1. 文档目的

本文档是新 ROS 2 功能包 `apriltag_block_grasp` 的开发、实测、标定和验收基线。

开发采用分阶段方式：Codex 负责按已确认的方案编写代码、静态检查、可在当前环境执行的测试以及现场测试说明；项目负责人负责在 RDK X5、Orbbec 相机和 RoArm-M3 实机上执行测试、测量参数并反馈完整结果。每个阶段只有通过对应验收门槛后，才进入下一阶段。

禁止根据主观判断填写未确认的机械臂参数。所有可能影响真实运动安全的未知参数必须保持为 `REQUIRED/待实测`，由现场测试确定后回填。

本文档当前只冻结 AprilTag 定位和抓取流程。放置方案及最终 ROS Noetic 机器狗通信协议尚未确定，开始实现前必须再次讨论。

## 2. 已确认的任务范围

### 2.1 物块与标签

- 目标为两个红色长方体，尺寸为 `100 × 50 × 50 mm`。
- 物块以 `100 × 50 mm` 平面为底，平放在箱子上表面。
- 箱子上表面不存在妨碍夹爪下探的箱沿或其他障碍。
- 场景中另有两个绿色物块，绿色物块不参与定位或抓取。
- 每个红色物块附近放置一个 AprilTag，安装方式与参考图片一致：标签水平放在物块短边外侧，标签平面与物块底面处于同一水平面。
- 标签与对应物块的空间关系固定；手工摆放误差在本项目中可忽略。
- 标签族为 `tag25h9`。
- 整张打印图片边长为 `50 mm`，用于姿态估计的有效 Tag 边长为 `38.9 mm`。
- 只使用 ID `0` 和 `1`，两个标签共用一套 `T_tag_object` 变换。
- 两个物块长边方向相同且固定。
- 夹爪从上方接近，从物块两条 `100 mm` 长边夹持，夹持跨度约为 `50 mm`。

### 2.2 硬件与系统

- 机械臂：RoArm-M3。
- 主控：RDK X5。
- 机械臂侧：ROS 2 Humble。
- 机器狗侧：ROS Noetic。
- 相机：沿用现有项目的 Orbbec RGBD 相机及眼在手上安装方式。
- 相机与末端之间未重新拆装，继续使用现有 `T_eef_camera` 手眼标定。
- 相机由新包通过 `pyorbbecsdk` 直接打开，不依赖 Orbbec ROS 2 驱动。
- 相机补光灯始终关闭。
- 运行新包时，不得同时运行其他占用 Orbbec 相机或机械臂串口的节点。
- 机器狗在抓取前停止并稳定站立，机身保持水平，抓取过程中不移动，并保证目标位于机械臂可达范围。

### 2.3 成功判据

- 闭爪和抬升动作按状态机完成，即认为抓取成功。
- 第一版不增加力、压力、电流、夹爪反馈或二次视觉抓取验证。

## 3. 明确不在当前阶段实现的内容

- 不使用红色阈值或 YOLO 识别目标。
- 不依赖 `red_block_grasp_ros2` 或 `red_block_grasp_mplus0` 的运行时模块。
- 不采用现有包的多次视觉伺服修正。
- 机械臂开始抓取运动后，不再更新或修正冻结的目标坐标。
- 不进行障碍物路径规划、MoveIt2 规划或完整场景工作空间建模。
- 暂不实现 A/B/C/D 放置。
- 暂不冻结最终 ROS1/ROS2 bridge、狗端 JSON 字段和 `reposition_required` 对外 topic。
- 暂不创建不可用的 `full_task.launch.py` 空壳。

## 4. 总体软件架构

新包必须完整包含自身运行所需的代码、配置、launch、手眼标定和文档，不得从两个旧功能包导入模块。

计划结构如下：

```text
src/apriltag_block_grasp/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/apriltag_block_grasp
├── README.md
├── config/
│   ├── apriltag_localization.yaml
│   ├── grasp_task.yaml
│   └── camera_calibration.yaml       # 可选覆盖，内容需实测
├── handeye/
│   └── handeye_cam_to_eef.json
├── launch/
│   ├── localization_only.launch.py
│   └── grasp_standalone.launch.py
└── apriltag_block_grasp/
    ├── core/
    │   ├── camera_rgbd_orbbec.py
    │   ├── apriltag_detector.py
    │   ├── pose_estimator.py
    │   ├── transforms.py
    │   └── stability_filter.py
    ├── nodes/
    │   ├── apriltag_localizer_node.py
    │   ├── manipulation_task_node.py
    │   ├── roarm_driver_node.py
    │   └── execution_logger_node.py
    └── roarm_m3/
        └── driver.py
```

后续放置方案和狗端协议确定后再增加：

```text
launch/full_task.launch.py
nodes/dog_arm_task_interface_node.py
```

抓取和放置最终由同一个 `manipulation_task_node` 管理，避免多个动作节点竞争 `/roarm_m3/cmd`。

## 5. 节点职责与内部接口

### 5.1 `apriltag_localizer_node`

职责：

- 独占 Orbbec 相机；
- 读取 RGBD、相机内参和畸变参数；
- 检测 tag25h9 的 ID 0、1；
- 估计 `T_camera_tag`；
- 执行 PnP、图像边缘、像素面积、重投影和可选深度一致性检查；
- 使用机械臂最新有效状态和手眼标定计算 `T_base_object`；
- 发布所有有效目标，不负责 ID 选择、任务周期或机械臂动作；
- 提供可关闭的调试窗口和默认关闭的图像保存功能。

定位输出使用 `std_msgs/String + JSON`。最终字段在实现时形成接口文档，至少应表达：

```json
{
  "stamp": 0.0,
  "frame_id": "camera_color",
  "detections": [
    {
      "tag_id": 0,
      "valid": true,
      "reason": "ok",
      "pixel_center": {"u": 0.0, "v": 0.0},
      "reprojection_error_px": 0.0,
      "pnp_depth_mm": 0.0,
      "rgbd_depth_mm": 0.0,
      "depth_check_enabled": true,
      "base_tag_mm": {"x": 0.0, "y": 0.0, "z": 0.0},
      "base_object_mm": {"x": 0.0, "y": 0.0, "z": 0.0}
    }
  ]
}
```

上述数值仅说明数据结构，不是运行参数。

### 5.2 `manipulation_task_node`

职责：

- 接收内部 `pick` 命令；
- 管理首次观察姿态、ID 选择、稳定采样、B 关节搜索和重定位等待；
- 冻结抓取坐标并执行开爪、预抓取、接近、下降、闭爪和抬升；
- 维护 `picked_ids`、`placed_ids` 和 `carrying_id`；
- 发布完整内部状态和高层结果；
- 后续在同一节点增加放置状态机。

内部命令暂定为：

```text
/apriltag_grasp/task_cmd
std_msgs/msg/String
```

测试命令结构：

```json
{"task_id": 101, "cmd": "pick"}
```

包内任务状态和结果接口确认为：

```text
/apriltag_grasp/task_state
/apriltag_grasp/task_result
std_msgs/msg/String + JSON
```

上述三个 `/apriltag_grasp/*` topic 仅用于新包内部编排和独立开发测试。后续机器狗使用的
`/dog_arm/*` topic 仍由独立 ROS1/ROS2 适配层处理，不直接写入定位或抓取核心。

不增加 `abort`、`reset_cycle`、`status` 或 `home` 命令。

### 5.3 `roarm_driver_node`

- 唯一占用机械臂串口；
- 订阅 `/roarm_m3/cmd`；
- 发布 `/roarm_m3/state`；
- 保留现有 `move_joint`、`move_pose`、`set_initial_pose` 和 LED/夹爪命令能力；
- 新包中保留自己的驱动副本，不从旧包导入；
- 连接后确保补光灯关闭。

### 5.4 `execution_logger_node`

每次运行生成 JSONL，至少记录：

- 标签检测摘要和拒绝原因；
- PnP、深度检查和最终物块坐标；
- 稳定样本统计；
- 机械臂状态、命令和到位判断；
- 状态机变化；
- `task_id`、选中 ID、`picked_ids`、`placed_ids`、`carrying_id`；
- 原始错误、恢复动作和恢复结果。

默认不保存相机视频或逐帧图片，避免占满存储。

## 6. 坐标系、定位和修正

### 6.1 坐标链

```text
T_base_object
= T_base_eef
× T_eef_camera
× T_camera_tag
× T_tag_object
```

- `T_base_eef`：继续沿用现有机械臂状态字段、单位及旋转顺序；
- `T_eef_camera`：沿用现有手眼标定；
- `T_camera_tag`：由 AprilTag 四角点、有效边长、相机内参和畸变参数通过 PnP 求得；
- `T_tag_object`：两个标签共用的完整刚体变换，终点为物块几何中心；其定义为
  `X_tag = T_tag_object × X_object`，即把物块坐标系中的点转换到 Tag 坐标系。

标签坐标约定：

```text
原点：Tag 有效黑色区域中心
+X：图案左侧指向右侧
+Y：图案上侧指向下侧
+Z：从标签正面指向标签背面（远离相机）
```

以上是用于 PnP 和齐次变换的右手坐标系。最终实现必须通过调试画面绘制坐标轴验证检测库的实际坐标方向。如果检测库返回约定不同，必须在 `pose_estimator` 内显式转换，不允许仅在配置文档中默认为一致。

“图案上下左右”以标签纸下方说明文字正常可读时的打印方向为准。标签正对相机时，OpenCV 调试图中的红色 X 轴应指向打印纸右侧，绿色 Y 轴应指向打印纸下方；整张图像显示旋转时，坐标轴应与标签纸一起旋转。

物块坐标约定：

```text
原点：物块几何中心
+X_object：沿物块 100 mm 长边
+Y_object：沿物块 50 mm 短边
+Z_object：从物块底面指向顶面
```

`tag_to_object.rotation_rpy_deg` 按 `[roll, pitch, yaw]` 保存，表示
`R_tag_object`，旋转组合顺序为：

```text
R_tag_object = Rz(yaw) × Ry(pitch) × Rx(roll)
```

其中 roll、pitch、yaw 分别是绕 X、Y、Z 轴的旋转角，单位为度。

现场已测得物块几何中心相对 Tag 有效黑色区域中心的位置：

```yaml
tag_to_object:
  translation_mm: [0.0, -77.0, -25.0]
  rotation_rpy_deg: [180.0, 0.0, -90.0]
```

- X 不变，因此 `tx = 0.0 mm`；
- 几何中心位于图案中心上方 `77 mm`，Tag 的 `+Y` 指向图案下方，因此 `ty = -77.0 mm`；
- 几何中心位于标签正面方向 `25 mm`，Tag 的 `+Z` 指向标签背面，因此 `tz = -25.0 mm`。
- 这组数值以物块几何中心为终点，不包含抓取高度或 `base_position_correction_mm`。
- 根据已确认的示例摆放方向，`rotation_rpy_deg = [180.0, 0.0, -90.0]`：
  `+X_object` 对应 `-Y_tag`，`+Y_object` 对应 `-X_tag`，`+Z_object` 对应
  `-Z_tag`。

### 6.2 PnP 顺序

1. 优先使用 OpenCV `cv2.aruco.DICT_APRILTAG_25h9` 检测；
2. 优先使用 `cv2.SOLVEPNP_IPPE_SQUARE`；
3. 当前 OpenCV 不支持 IPPE Square 时，回退普通 PnP；
4. RDK X5 上的 OpenCV 不支持该字典或实测效果不满足要求时，再切换专用 AprilTag 检测库。

### 6.3 相机参数

- 默认从 `pyorbbecsdk` 获取当前分辨率对应的相机内参和畸变参数；
- 允许通过独立 YAML 覆盖；
- 必须区分原始图像与已校正图像；
- 原始图像必须使用正确畸变系数；
- 已校正图像使用对应的新相机矩阵和零畸变；
- 不允许把原始图像当作无畸变图像处理。

### 6.4 RGBD 深度检查

- PnP 是唯一的位置来源；
- RGBD 深度只作为可关闭的有效性检查，不替换 PnP 平移；
- 开启检查时，无有效深度或 PnP 距离与深度差超过阈值均拒绝当前帧；
- 如果设备无法可靠完成彩色/深度对齐，自动禁用深度检查，继续使用 PnP，并在日志中记录 `depth_check_disabled` 及原因。

### 6.5 位置修正

不继承旧包中的硬编码 `base Z + 100 mm`。提供：

```yaml
base_position_correction_mm: [REQUIRED, REQUIRED, REQUIRED]
```

该项必须经定位验证后回填。若实测确认无需修正，明确填入 `[0.0, 0.0, 0.0]`，不能依赖隐含默认值。

## 7. 单帧有效性和稳定定位

### 7.1 单帧检查

每个检测结果至少检查：

- ID 是否为 0 或 1；
- 标签是否位于相机前方；
- 平移、旋转、矩阵和最终坐标是否为有限数；
- 标签距离是否在实测配置范围内；
- 四角点是否距离图像边缘过近；
- 标签像素面积是否达到阈值；
- PnP 重投影误差是否小于阈值；
- 开启深度校验时，深度是否有效且与 PnP 一致；
- 机械臂状态是否有效且新鲜。

### 7.2 稳定快照

只判断最终物块 `base XYZ` 的多帧波动，不判断标签旋转角的多帧稳定性。

```text
锁定一个 tag ID
→ 在限定时间内收集 N 帧有效 base_object XYZ
→ 采样期间机械臂保持静止
→ XYZ 波动满足配置阈值
→ 对 XYZ 取中位数
→ 生成不可变的抓取快照
```

以下情况均可导致稳定超时：

- 同一 ID 无法持续出现；
- 有效帧数不足；
- XYZ 波动过大；
- PnP 检查反复失败；
- 深度检查开启时反复无深度或不一致；
- 采样期间机械臂发生超阈值运动。

每次改变 B 关节观察角后，必须清空上一角度的全部样本。不同观察角度的结果不能混合计算中位数。

## 8. ID 选择、任务周期和命令规则

### 8.1 目标选择

- 选择顺序为 ID 0、ID 1；
- ID 0 不可见时可直接选择 ID 1；
- 若只发现一个未完成 ID，直接对该目标进行稳定采样；
- 每个搜索角度重新选择尚未完成的可见 ID；
- 一旦在当前角度开始采集某个 ID，该角度内不切换；
- 当前角度失败并进入下一角度后，才重新选择。

所有搜索角度结束后：

- 从未看到任何未完成标签：`target_not_found`；
- 曾看到至少一个未完成标签但始终无法稳定：`target_unstable`。

### 8.2 周期状态

任务节点在内存中维护：

```text
picked_ids
placed_ids
carrying_id
cycle_observation_b_deg
```

- 闭爪并成功抬升后，将 ID 加入 `picked_ids`，并设置 `carrying_id`；
- 放置成功后，将 ID 加入 `placed_ids` 并清除 `carrying_id`；
- ID 0、1 均进入 `placed_ids` 后，自动清空周期状态；
- `carrying_id` 非空时拒绝新的 `pick`，返回 `object_already_held`；
- 节点重启视为人工介入后的新周期，不恢复旧记录。

### 8.3 命令并发规则

- 正在执行 `pick` 时收到相同 `task_id` 的 `pick`：忽略；
- 正在执行 `pick` 时收到不同 `task_id`：返回 `arm_busy`；
- `REPOSITION_REQUIRED` 等待期间，相同 `task_id` 的 `pick` 恢复任务；
- `REPOSITION_REQUIRED` 等待期间，新 `task_id` 可以覆盖旧任务；
- 覆盖时保留周期完成记录，清除旧任务搜索/采样状态，并继续使用周期最初的观察 B 角；
- 恢复或覆盖后重新查看所有未完成目标，允许从原先不稳定的 ID 0 改抓当前可见的 ID 1。

## 9. B 关节搜索和重定位

首次正常 `pick`：

```text
移动到固定观察关节姿态
→ 记录周期最初 B 角 B0
→ 从偏移 0 开始检测
```

搜索目标是相对 B0 的绝对关节角，不是增量移动。例如：

```yaml
b_offsets_deg: [0, -5, 5, -10, 10]  # 仅为结构示例，正式数值必须实测
```

对应：

```text
B0 → B0-5 → B0+5 → B0-10 → B0+10
```

- 无目标和目标不稳定均使用该左右交替搜索；
- 定位成功后直接从当前 B 角开始抓取，不返回 B0；
- 搜索耗尽后必须先返回 B0；
- 返回成功后发布可恢复中间状态 `reposition_required`；
- 原因分别为 `target_not_found` 或 `target_unstable`；
- 机器狗调整并站稳后，重新发送 `pick`；
- 重定位后的重复 `pick` 不再次进入固定观察姿态，也不建立新的 B0；
- 新任务覆盖等待状态时同样沿用周期最初 B0。

现场已批准阶段 4 使用 `[0, -5, 5, -10, 10]°` 进行搜索标定。B-only 常驻驱动单次运动和
返回 B0 已通过：到位反馈采用连续 3 帧进入 `±1.5°`；驱动继续保持单次最大 `10°` 限制。
相邻搜索角跨度大于安全范围时经过 B0 及必要的 `±5°` 不采样过渡点，不提高驱动限制。

第二次正常抓取发生在前一物块已经成功放置、机器狗返回抓取区之后。该次 `pick` 需要重新进入固定观察姿态，然后检测剩余 ID。

## 10. 抓取几何与动作流程

### 10.1 配置表达

所有抓取位置以物块几何中心为基准。三个动作点保持相同 XY，只改变 Z：

```yaml
grasp_geometry:
  final_grasp_tcp_offset_base_mm: [-24.057833653, 33.016991758, 0.675554933]
  pre_grasp_z_offset_mm: REQUIRED
  approach_z_offset_mm: REQUIRED
  lift_distance_mm: REQUIRED
```

固定观察姿态和固定抓取末端姿态分别配置：

```yaml
observation_joint_pose_deg:
  b: REQUIRED
  s: REQUIRED
  e: REQUIRED
  t: REQUIRED
  r: PRESERVE
  g: PRESERVE

grasp_tool_orientation:
  roll_rad: 0.012271846
  pitch_rad: 1.713456540
  yaw_rad: -0.062893212
  reference_clamp_g_rad: 2.408349837
```

- `final_grasp_tcp_offset_base_mm` 的定义是
  `P_base_final_grasp_tcp - P_base_object`。当前数值来自标签、物块和机械臂底座均未移动时，
  视觉几何中心 `[272.380757453, -48.655443158, -109.909254433] mm` 与人工最终夹取
  TCP `[248.322923800, -15.638451400, -109.233699500] mm` 的差；
- 夹爪为左指固定、右指活动的非对称结构，因此最终TCP不要求与物块几何中心重合；
- 该偏移包含夹爪几何偏置，也可能包含本次定位中的系统误差，不得复制到
  `base_position_correction_mm`；
- 当前偏移仅完成单位置测量，状态为暂定，必须经过多个物块位置复测后才能开放自动下降；
- `reference_clamp_g_rad = 2.408349837` 对应能够夹住但不紧的状态，只作为后续夹爪专项
  标定基准，当前不视为最终闭爪参数；
- 观察阶段使用固定 B/S/E/T 关节姿态；R 不发送命令以保持相机当前物理朝向，G 不发送命令以保持夹爪状态；
- 相机使用原始传感器画面，不做软件旋转；
- 抓取阶段使用固定末端姿态；
- 不根据标签姿态调整夹爪朝向；
- `OPENING_GRIPPER` 在移动预抓取点之前执行；
- 最终夹持高度、开合角度、速度和等待时间均由实测确定。

### 10.2 主状态机

```text
IDLE
→ MOVE_TO_OBSERVATION
→ SEARCHING
→ STABILIZING
→ FREEZE_TARGET
→ OPENING_GRIPPER
→ MOVING_PRE_GRASP
→ MOVING_APPROACH
→ DESCENDING
→ CLOSING_GRIPPER
→ LIFTING
→ CARRYING
→ PICK_SUCCEEDED
```

动作路径：

```text
冻结物块坐标
→ 张开夹爪并等待
→ 移动到预抓取点
→ 同 XY 移动到接近点
→ 同 XY 垂直下降到最终夹持点
→ 闭爪并等待
→ 同 XY 垂直抬升 lift_distance_mm
→ 停在物块上方并保持夹爪闭合
→ 发布 pick_success
```

物块保持悬空，等待机器狗运输和后续放置命令。第一阶段尚未实现放置，因此实测后由人工处理物块并重启任务节点开始新周期。

## 11. 动作完成模式

运动到位判断必须可切换：

```yaml
motion_completion:
  mode: REQUIRED               # feedback 或 timed
  position_tolerance_mm: REQUIRED
  orientation_tolerance_deg: REQUIRED
  joint_tolerance_deg: REQUIRED
  stable_sample_count: REQUIRED
  motion_timeout_s: REQUIRED
  timed_wait_s: REQUIRED
```

- `feedback`：持续读取机械臂状态，位置/姿态或关节误差连续满足阈值后进入下一状态；超时报告 `motion_timeout`；
- `timed`：发出动作后等待配置时间；用于现场反馈不可靠时切换；
- 位姿移动支持反馈/定时；
- B 关节搜索支持关节反馈/定时；
- 夹爪开合使用固定等待时间。

不得在动作尚未确认完成时直接进入下一状态。

## 12. 基本安全检查与失败恢复

### 12.1 保留的基本检查

不设置常规 XYZ 工作空间上下限，但在发送任何运动命令前必须：

- 验证 JSON、坐标和矩阵字段完整；
- 拒绝 `NaN`、`Inf` 和非正 PnP 深度；
- 验证齐次矩阵最后一行和旋转矩阵基本有效性；
- 验证机械臂状态有效且新鲜；
- 验证预抓取、接近、最终夹持三点的 XY 完全来自同一冻结目标；
- 验证抬升目标只改变 Z；
- 动作超时后禁止继续执行后续动作。

承载平面/夹爪最低点检查保留在代码和配置中，但第一阶段默认关闭。完成 TCP 和夹爪几何标定后再开启：

```yaml
safety:
  enable_support_plane_check: false
  min_tool_center_above_support_mm: REQUIRED
```

### 12.2 失败恢复原则

运动开始前故障：

```text
停止后续动作
→ 尝试返回观察姿态
→ 发布 failed
```

下降前故障：

```text
不下降、不闭爪
→ 尝试返回预抓取安全高度
→ 发布 failed
```

下降后或闭爪后故障：

```text
不自动张开夹爪
→ 机械臂反馈可用时尝试同 XY 垂直抬升
→ 保持夹爪闭合
→ 发布 failed
→ 等待人工处理
```

状态和日志必须同时报告原始错误、是否尝试恢复及恢复结果。

## 13. 分阶段开发与实测门槛

### 阶段 0：环境能力检查

Codex：

- 提供 RDK X5 环境检查脚本或命令；
- 检查 Python、ROS 2、OpenCV、`cv2.aruco`、`DICT_APRILTAG_25h9`、`SOLVEPNP_IPPE_SQUARE`、`pyorbbecsdk` 和机械臂消息依赖；
- 输出明确的支持状态，不执行机械臂动作。

现场负责人：

- 在 RDK X5 执行检查；
- 提供完整输出、OpenCV 版本、相机型号/分辨率和失败信息。

通过条件：

- 相机 SDK 可用；
- 已确定使用 OpenCV 路径还是专用 AprilTag 回退路径；
- 相机内参和畸变参数可读取，或明确需要 YAML 覆盖。

### 阶段 1：功能包骨架与纯二维检测

Codex：

- 创建独立 `ament_python` 包；
- 实现 Orbbec 彩色取流和 ID 0、1 检测；
- 实现调试窗口、角点/ID/坐标轴显示和默认关闭的图像保存；
- 提供 `localization_only.launch.py` 的二维检查模式和测试说明。

现场负责人：

- 验证两个标签单独/同时出现时 ID 正确；
- 测试常用距离、视角、光照和图像边缘位置；
- 反馈漏检、误检、帧率和现场图像。

通过条件：

- 只识别 ID 0、1；
- 两个标签同时出现时均能稳定输出；
- 调试画面坐标方向已经人工核对。

### 阶段 2：PnP 与 RGBD 校验

Codex：

- 实现 IPPE Square/普通 PnP 回退；
- 实现重投影误差、距离、面积、边缘和有限数检查；
- 实现 RGBD 对齐能力检查及可关闭的一致性检查；
- 发布 `T_camera_tag` 摘要和拒绝原因。

现场负责人：

- 精确确认有效 Tag 边长 `38.9 mm`；
- 在多个已知距离下测量 PnP 距离误差；
- 验证深度对齐和 PnP/深度差；
- 提供阈值建议所需数据，不凭单帧填写阈值。

通过条件：

- PnP 平移方向和距离趋势正确；
- 重投影和深度检查能区分有效/异常帧；
- 若深度不可用，已明确自动禁用路径且 PnP 可独立运行。

### 阶段 3：手眼变换与 `tag_to_object` 标定

RoArm-M3 的 ESP32 USB 串口在 `open()` 时可能因 DTR/RTS 自动下载电路发生复位，
已观察到 OLED 刷新，但常驻只读驱动实测没有观察到固定下降。当前证据表明此前的固定下降
与显式运动命令强相关，不能再归因于单纯打开串口。`serial_bytes_transmitted=0` 只能证明程序
未发送串口命令，不能证明打开串口对控制器完全无副作用。标定探针应先建立并持续保持串口
连接，等待控制器稳定后再开始采样；同一轮不得关闭后重新打开串口。

常驻连接内的零位移对照试验也已完成：把稳定 `T=1051` 的 `x/y/z/tit/r/g` 原样写入一条
`T=1041` 后，末端在约 0.54 秒内产生 `(+5.864, +0.054, -20.310) mm` 位移及约
`+3.340°` 的 `tit` 变化，之后稳定而非持续振荡。因此该接口映射未通过验收，已停用并禁止
重复测试；后续不得用 `T=1051` 笛卡尔反馈直接构造 `T=1041` 抓取轨迹。

等待和人工调整期间主动上报的旧机械臂状态会积压在串口接收缓冲区。人工确认观察姿态
完成后，必须先清空输入缓冲区，再读取新状态并采集相机图像，禁止将调整前的机械臂
状态与调整后的图像组成同一坐标样本。

Codex：

- 集成现有 `T_eef_camera`；
- 复用现有机械臂状态字段和旋转约定；
- 实现 `T_base_object` 和可选 `base_position_correction_mm`；
- 在调试界面和 JSON 中同时显示 `base_tag`、`base_object`；
- 提供只定位、不下降、不闭爪的标定流程。

现场负责人：

- 测量并回填 `tag_to_object.translation_mm` 和 `rotation_rpy_deg`；
- 在多个机械臂观察姿态下验证同一物块的 base 坐标一致性；
- 测量并决定 `base_position_correction_mm`，包括明确的零修正结论；
- 使用安全高度验证计算出的物块 XY，不执行抓取。

通过条件：

- 标签、物块和 base 坐标方向正确；
- 多观察姿态下定位误差满足现场抓取要求；
- 所有位置修正都有实测记录。

### 阶段 4：稳定采样、ID 管理与 B 搜索（无抓取动作）

Codex：

- 实现 ID 选择、锁定、多帧中位数、XYZ 波动判断；
- 实现不同 B 角度样本隔离；
- 实现任务周期状态和 `reposition_required` 内部状态；
- 实现 B 搜索，但不开放下降、闭爪和抬升；
- 提供完整日志。

现场负责人：

- 标定观察关节姿态和 B 搜索偏移；
- 测试 ID 0/1 同时出现、仅一个出现、目标不稳定和完全不可见；
- 测试返回周期最初 B 角；
- 回填稳定帧数、XYZ 波动、超时、状态新鲜度和搜索等待参数。

通过条件：

- 选择、锁定、搜索和错误分类符合本文档；
- 不同观察角度样本没有混用；
- `target_not_found` 与 `target_unstable` 区分正确；
- 此阶段不得产生抓取动作。

### 阶段 5：分段运动与动作到位判断

按风险从低到高逐项开放，每个子阶段单独实测后才能继续：

1. 固定观察姿态；
2. 开爪；
3. 移动到预抓取点；
4. 移动到接近点；
5. 空载下降至保守高度；
6. 最终夹持高度；
7. 闭爪；
8. 垂直抬升。

Codex：

- 实现反馈/定时两种完成模式；
- 每次只开放当前获准动作；
- 实现动作超时和相应恢复；
- 提供每个动作的 launch 参数、命令和验收记录模板。

现场负责人：

- 先空载、低速测试，再逐步接近实物；
- 标定固定抓取末端姿态；
- 标定 XY 偏移、三个 Z 偏移、抬升距离、开合角度、速度、容差、超时和等待；
- 记录实际机械臂反馈和是否发生碰撞/擦碰；
- 明确同意后才开放下一动作。

通过条件：

- 每个动作均有实测参数；
- 到位模式和超时行为可靠；
- 失败时不会继续执行危险后续状态。

### 阶段 6：独立完整抓取

Codex：

- 完成 `grasp_standalone.launch.py`；
- 串联全部抓取状态；
- 实现命令并发、周期记录、携带状态和日志；
- 提供 ROS 2 手动命令和故障注入测试说明。

现场负责人：

- 依次测试 ID 0、ID 1、只见一个标签、先抓 ID 1、重定位恢复和第二次正常抓取；
- 验证 `pick_success` 后物块悬空、夹爪保持闭合；
- 第一阶段测试结束后人工处理物块并重启节点；
- 提供完整 JSONL 和终端日志。

通过条件：

- 抓取主流程、周期排除和恢复流程均符合本文档；
- 不会重复选择已经成功抓取的 ID；
- `carrying_id` 非空时拒绝新的抓取；
- 实测失败均能停止或进入约定恢复动作。

### 阶段 7：放置方案确认与实现

开始前必须重新讨论并确认：

- A/B/C/D 贴纸视觉定位，或 A/B/C/D 固定放置位姿；
- 放置区目标选择和狗端命令中的 `zone`；
- 放置点、接近点、开爪点和返回姿态；
- 放置成功及放置失败的判定与恢复；
- `picked_ids`、`placed_ids`、`carrying_id` 的最终联动。

在这些内容确认之前不编写放置动作逻辑。

### 阶段 8：ROS Noetic 机器狗适配

开始前读取并确认机器狗端代码，暂时沿用现有高层方向：

```text
/dog_arm/task_cmd
/dog_arm/task_result
/dog_arm/base_adjust_req
```

届时重新确定：

- ROS1/ROS2 bridge 的部署位置与启动方式；
- 最终 topic 名、JSON 字段和 `task_id` 生命周期；
- `reposition_required` 发布到结果还是调整请求 topic；
- 机器狗调整方向、距离、站稳确认和重发规则；
- 超时、重复消息、断线与幂等行为。

狗端适配必须作为独立接口层，不把 ROS Noetic 协议写入定位或抓取核心代码。

## 14. 每阶段协作和反馈格式

每完成一个代码阶段，Codex 应交付：

- 修改文件清单；
- 节点、topic、参数和状态说明；
- 编译/静态检查结果；
- 本阶段现场测试命令；
- 预期输出与通过条件；
- 已知风险和明确禁止执行的动作；
- 需要现场测量并反馈的参数表。

现场负责人反馈时尽量提供：

```text
当前 Git 分支和提交
完整构建命令与输出
完整启动命令
从启动到问题发生的终端日志
一条或多条实际 JSON 消息
对应 JSONL 日志
现场照片或调试窗口截图
机械臂、相机、标签和物块的相对位置
本轮测得的参数及测量方法
是否发生碰撞、擦碰、抖动、超时或误动作
```

如果实测结果与设计不一致，应先停止后续阶段，分析证据并修订流程或参数，不以猜测继续开放动作。

## 15. 待实测参数清单

以下参数在对应阶段完成前不得主观赋予可驱动真实机械臂的默认值：

- 相机分辨率、内参、畸变和 RGBD 对齐状态；
- PnP 距离范围、像素面积、边缘余量、重投影误差和深度差阈值；
- `tag_to_object` 六自由度变换；
- `base_position_correction_mm`；
- 固定观察关节姿态；
- 固定抓取末端姿态；
- B 关节搜索偏移和等待时间；
- 稳定帧数、采样超时和 XYZ 波动阈值；
- 机械臂状态新鲜度和静止判断阈值；
- 目标 XY 修正；
- 预抓取、接近和最终夹持 Z 偏移；
- 垂直抬升距离；
- 夹爪开合角度和等待时间；
- 位姿/关节速度、加速度、到位容差和超时；
- 承载平面和夹爪 TCP 几何安全参数。

## 16. 当前冻结项与待确认项

### 已冻结

- 包名、硬件、标签、相机接入方式；
- PnP/深度定位路线；
- 坐标链和标签坐标约定；
- ID 选择、稳定采样和 B 搜索；
- 三点同 XY 抓取路径和固定夹爪姿态；
- 周期记录、重定位和命令并发规则；
- 可切换动作完成判断；
- 基本安全检查、失败恢复和日志；
- 分阶段开发与实测门槛。

### 实现前必须再次确认

- 放置采用贴纸识别还是固定位置；
- A/B/C/D 命令、动作和验收规则；
- 机器狗端实际代码和最终 ROS1/ROS2 协议；
- 所有 `REQUIRED/待实测` 数值参数。
