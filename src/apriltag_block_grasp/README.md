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
