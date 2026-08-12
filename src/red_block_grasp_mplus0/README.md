# red_block_grasp_mplus0

基于 ROS2 Humble 的红色物料识别与 RoArm-M3 视觉伺服抓取工程，运行平台为 RDK X5。

本包从 `red_block_grasp_ros2` 建立独立开发基线，已包含相机接入、视觉识别、目标定位、模型、手眼标定、机械臂驱动和任务节点，不在运行时依赖原功能包。当前视觉识别算法保持原样，后续优化范围以抓取定位和抓取放置动作为主。

模型、手眼标定和配置通过 ROS2 包共享目录查找，不依赖功能包源码位于固定工作空间路径。默认运行日志和困难样本写入 `~/.ros/red_block_grasp_mplus0/`。

原抓取、放置、比赛通信和机械臂执行流程已集中备份到 `legacy/action_flow_v1/`。该目录不参与构建和安装；正式运行入口暂时保持不变，后续开发全新动作流程时逐步替换正式节点、launch 和动作配置。

工程使用 Orbbec RGBD 相机和 YOLO 模型识别红色物块，通过深度图计算目标三维坐标，再结合手眼标定转换到机械臂 base 坐标系，最后由视觉闭环状态机控制机械臂移动到预抓取点并执行安全下降测试。

## 当前状态

已完成：

- 使用 `red_block_yolo11n_v3_mix.pt` 识别红色物块
- RGBD 深度定位和手眼标定坐标转换
- 小步视觉闭环移动到预抓取点
- 预抓取点安全高度控制
- 下降测试流程
- 新增 center-first 视觉伺服策略：先居中再靠近
- 可选闭爪、抬升、移动到放置点、开爪流程
- 运行数据 JSONL 记录
- 轻量化自适应视觉伺服步长

仍在现场调优：

- 眼在手上相机视角变化过快时的目标保持能力
- 下降距离、速度、单步下降量和最低安全高度
- 闭爪角度、抬升距离和放置点

## 节点结构

主要节点：

- `roarm_driver_node`
  - 机械臂驱动节点，唯一占用 RoArm-M3 串口，通常为 `/dev/ttyUSB0`
  - 订阅：`/roarm_m3/cmd`
  - 发布：`/roarm_m3/state`

- `target_localizer_node`
  - 启动 Orbbec RGBD 相机
  - 运行 YOLO 检测
  - 使用深度和手眼标定发布目标 base 坐标
  - 订阅：`/roarm_m3/state`
  - 发布：`/red_block/target_base`

- `visual_servo_task_node`
  - 视觉闭环任务状态机
  - 发布机械臂运动命令
  - 保留工作空间、高度、ROI、目标新鲜度等安全检查
  - 订阅：`/roarm_m3/state`、`/red_block/target_base`
  - 发布：`/roarm_m3/cmd`、`/red_block/visual_servo_state`

- `execution_logger_node`
  - 记录现场运行数据，方便复盘和调参
  - 订阅：`/roarm_m3/state`、`/red_block/target_base`、`/red_block/visual_servo_state`、`/roarm_m3/cmd`
  - 写入：`/home/sunrise/dog/ros2_red_block_ws/run_records`

注意：运行本任务 launch 时不要同时运行其他占用 Orbbec 相机的节点，尤其不要同时运行 `yolo_camera_node`。

## X5 运行命令

```bash
cd /home/sunrise/dog/ros2_red_block_ws
git pull
colcon build --packages-select red_block_grasp_mplus0 --event-handlers console_direct+
source source_red_block.sh
ros2 launch red_block_grasp_mplus0 visual_servo_task.launch.py show_window:=true
```

不显示调试窗口：

```bash
ros2 launch red_block_grasp_mplus0 visual_servo_task.launch.py show_window:=false
```

高帧率现场调试建议先关闭窗口运行：

```bash
ros2 launch red_block_grasp_mplus0 visual_servo_task.launch.py show_window:=false infer_imgsz:=256 target_timer_period:=0.08
```

关闭运行日志记录：

```bash
ros2 launch red_block_grasp_mplus0 visual_servo_task.launch.py show_window:=true enable_execution_logger:=false
```

启用下降后的完整抓取放置流程：

```bash
ros2 launch red_block_grasp_mplus0 visual_servo_task.launch.py show_window:=true enable_pick_place_sequence:=true
```

## 默认文件

默认 YOLO 模型：

```text
/home/sunrise/dog/ros2_red_block_ws/src/red_block_grasp_mplus0/models/red_block_yolo11n_v3_mix.pt
```

默认手眼标定文件：

```text
/home/sunrise/dog/ros2_red_block_ws/src/red_block_grasp_mplus0/handeye/handeye_cam_to_eef.json
```

## 状态机流程

主流程：

```text
INIT
-> WAIT_INITIAL
-> WAIT_TARGET
-> CENTER_TARGET
-> WAIT_AFTER_CENTER_STEP
-> APPROACH_CENTERED
-> WAIT_AFTER_STEP
-> REACHED_PRE_GRASP
-> DESCEND_TEST
-> WAIT_AFTER_DESCEND_STEP
-> WAIT_AFTER_DESCEND
-> DONE
```

当 `enable_pick_place_sequence:=true` 时，下降后继续执行：

```text
CLOSE_GRIPPER
-> WAIT_AFTER_CLOSE_GRIPPER
-> LIFT_AFTER_GRASP
-> WAIT_AFTER_LIFT
-> MOVE_TO_PLACE
-> WAIT_AFTER_PLACE
-> OPEN_GRIPPER
-> WAIT_AFTER_OPEN_GRIPPER
-> DONE
```

## 视觉伺服策略

视觉闭环阶段不会一次性冲到最终目标，而是不断发送小步 `move_pose`。这是因为相机固定在机械臂上，机械臂每移动一步，相机视角都会变化；步长过大时，目标容易跑到画面边缘甚至丢失。

当前保护策略：

- 保留 `check_target_range` 工作空间保护
- 使用 `servo_min_z_mm` 限制视觉闭环阶段最低高度
- 当前高度过低时先执行恢复抬升
- 新增 center-first 视觉伺服策略：先把红块移动到图像中心，再执行高位小步靠近
- 修复 CENTER_TARGET 阶段 E 关节像素居中方向：新增 `center_e_pixel_sign`（默认 -1.0）和 `center_b_pixel_sign`（默认 1.0），用于校正 B/E 关节调整方向与像素偏移之间的符号关系。如果目标在图像中偏离中心但 E 关节调整后反而更远，优先翻转对应 sign 参数。
- 目标靠近安全 ROI 边缘时使用 `edge-safe-step`
- 根据目标像素点距离图像中心的程度动态调整本次步长
- 如果发生 `Target lost after step`，先恢复到上一次安全高位，再进入 WAIT_TARGET

自适应步长不是强化学习，也不会在线训练模型。它只是根据当前视觉反馈和运行记录做安全的运行时参数自适应。

关键参数：

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `infer_imgsz` | `256` | YOLO 推理输入尺寸，越小越快，过小会降低识别稳定性 |
| `target_timer_period` | `0.08` | 目标定位节点调度周期，0.08 约等于最高 12.5 FPS |
| `show_window` | `false` | 是否显示 OpenCV 调试窗口，开启后会降低 X5 帧率 |
| `adaptive_step_enabled` | `true` | 是否启用自适应步长 |
| `max_step_mm` | launch 配置 | 原普通最大步长 |
| `edge_step_mm` | launch 配置 | 原边缘保护步长 |
| `move_speed` | `0.12` | 视觉伺服移动速度 |
| `step_wait_s` | `0.8` | 每次视觉伺服移动后的等待时间 |
| `adaptive_step_min_mm` | `5.0` | 普通自适应最小步长 |
| `adaptive_step_max_mm` | `25.0` | 普通自适应最大步长 |
| `adaptive_edge_step_min_mm` | `5.0` | 边缘保护最小步长 |
| `adaptive_center_good_ratio` | `0.25` | 认为目标较居中的比例 |
| `adaptive_center_bad_ratio` | `0.55` | 认为目标偏离较大的比例 |

如果现场仍然只有 1-2 FPS，优先尝试：

```bash
ros2 launch red_block_grasp_mplus0 visual_servo_task.launch.py show_window:=false enable_execution_logger:=false infer_imgsz:=224 target_timer_period:=0.05
```

如果识别变快但动作仍然一顿一顿，优先降低等待时间：

```bash
ros2 launch red_block_grasp_mplus0 visual_servo_task.launch.py show_window:=false step_wait_s:=0.5 move_speed:=0.14
```

## 下降测试策略

下降阶段默认使用像素引导的保守下降。只有目标仍然可见、置信度足够、没有跑出安全 ROI，并且像素位置接近图像中心时，机械臂才允许下降。

关键行为：

- 目标丢失、过期、置信度过低或超出安全 ROI 时，不继续下降
- 目标可见但偏离中心时，只做小步 XY 修正，不下降 Z
- 目标回到中心死区内时，才下降一个小步
- 达到下降距离或触发最低安全高度后停止

关键参数：

| 参数 | 典型值 | 说明 |
| --- | ---: | --- |
| `descend_test_mm` | `30.0` | 测试总下降距离 |
| `descend_step_mm` | `5.0` | 单次下降步长 |
| `descend_control_mode` | `pixel` | 像素引导下降模式 |
| `descend_min_confidence` | `0.60` | 下降阶段最低目标置信度 |
| `descend_pixel_deadband` | `35.0` | 允许下降的像素误差死区 |
| `descend_pixel_kp_mm_per_px` | 现场调参 | 像素误差到 XY 修正的比例 |
| `descend_pixel_max_xy_step_mm` | 现场调参 | 单次 XY 修正上限 |
| `descend_min_z_mm` | `30.0` | 最低安全高度 |
| `descend_speed` | `0.06` | 下降速度 |

如果下降时机械臂明显向前伸，导致物块逐渐跑出相机画面，优先检查和调整：

- 降低 `descend_step_mm`
- 降低 `descend_pixel_max_xy_step_mm`
- 检查 `descend_pixel_x_sign` 和 `descend_pixel_y_sign` 方向是否反了
- 增大 `descend_pixel_deadband` 前先确认目标像素是否稳定

## 运行日志

默认启动 `execution_logger_node`，由 `enable_execution_logger:=true` 控制。

日志目录：

```text
/home/sunrise/dog/ros2_red_block_ws/run_records
```

文件名格式：

```text
run_YYYYMMDD_HHMMSS.jsonl
```

每行 JSONL 记录包含：

- `stamp`
- 最新机械臂状态
- 最新目标定位
- 最新视觉伺服状态
- 最新机械臂命令

查看日志：

```bash
ls -lh /home/sunrise/dog/ros2_red_block_ws/run_records | tail
tail -f /home/sunrise/dog/ros2_red_block_ws/run_records/<latest_run_file>.jsonl
```

运行日志用于现场复盘和参数调优，不要提交到 Git。

## 常用调试命令

```bash
ros2 topic echo /red_block/target_base
ros2 topic echo /red_block/visual_servo_state
ros2 topic echo /roarm_m3/state
ros2 topic echo /roarm_m3/cmd
```

查看 launch 参数：

```bash
ros2 launch red_block_grasp_mplus0 visual_servo_task.launch.py --show-args
```

## 推荐调参顺序

1. 机械臂静止时，先确认 `/red_block/target_base` 稳定。
2. 调 `max_step_mm`、`edge_step_mm` 和自适应步长参数，让视觉闭环移动时目标尽量留在画面中间。
3. 确认能稳定进入 `REACHED_PRE_GRASP`，且不会反复触发恢复抬升。
4. 用较小的 `descend_test_mm` 先验证下降方向。
5. 下降稳定后，再启用 `enable_pick_place_sequence:=true` 测试闭爪、抬升和放置。

## Git 注意事项

不要提交运行产物和训练产物：

- `datasets/`
- `runs/`
- `hard_samples/`
- `run_records/`
- `build/`
- `install/`
- `log/`
- `Log/`

## 比赛机械臂 V0 任务模式

比赛 V0 模式只定义机械臂侧接口。机械臂只负责两个动作：

- `pick`：识别红色长条/红色目标，执行视觉伺服靠近、下降、闭爪、抬升，并在抬升后悬空保持 `hold_after_pick_s`（默认 3.2 秒）以上。收到 `pick_success` 表示物料已经抓起、抬升并保持完成。
- `place_to_zone`：不识别 A/B/C/D，也不识别箱子。默认机械狗已经移动到正确放置箱前并站稳，机械臂执行固定放置点移动、开爪、等待、回安全/初始姿态。收到 `place_success` 表示已经开爪放置并回到安全姿态。

机械狗侧负责移动到抓取区/放置区并站稳；ROS1 bridge 暂时不做，等 ROS2 内部测试稳定后再接。

启动比赛机械臂侧：

```bash
cd /home/sunrise/dog/ros2_red_block_ws
colcon build --packages-select red_block_grasp_mplus0 --event-handlers console_direct+
source source_red_block.sh
ros2 launch red_block_grasp_mplus0 competition_arm_task.launch.py show_window:=false
```

狗端/测试端发送任务：

```bash
ros2 topic pub --once /dog_arm/task_cmd std_msgs/msg/String "{data: '{\"task_id\":1,\"cmd\":\"pick\"}'}"
ros2 topic pub --once /dog_arm/task_cmd std_msgs/msg/String "{data: '{\"task_id\":2,\"cmd\":\"place_to_zone\"}'}"
```

查看结果和左右微调请求：

```bash
ros2 topic echo /dog_arm/task_result
ros2 topic echo /dog_arm/base_adjust_req
```

V0 通信协议：

```json
{"task_id":1,"cmd":"pick"}
{"task_id":2,"cmd":"place_to_zone"}
{"task_id":1,"result":"pick_success"}
{"task_id":2,"result":"place_success"}
{"task_id":1,"result":"pick_failed","error":"target_not_found"}
{"task_id":2,"result":"place_failed","error":"place_motion_failed"}
```

当目标位于机械臂横向工作空间边缘时，机械臂会请求机械狗只做左右微调：

```json
{"task_id":1,"direction":"left","step_m":0.05,"reason":"target_too_right"}
{"task_id":1,"direction":"right","step_m":0.05,"reason":"target_too_left"}
```

`left/right` 表示机械狗自身坐标系左右横移，不按图像左右定义。V0 不发布前后、上下或其他方向。
