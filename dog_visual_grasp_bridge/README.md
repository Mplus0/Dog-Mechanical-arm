# dog_visual_grasp_bridge

这个目录负责把视觉结果桥接到官方 `RoArm-M3` 控制接口：

`/red_block/target_base` -> `visual_moveit_grasp.py` -> `/move_line_cmd` + `/gripper_cmd`

它不修改 handeye、相机内参、YOLO、官方 `roarm_ws` 或服务定义。

## Runtime

- `visual_moveit_grasp.py` 使用 `ReentrantCallbackGroup`
- main loop 使用 `MultiThreadedExecutor(num_threads=2)`
- `move_line_timeout_s` 默认 `120.0`
- `service_wait_timeout_s` 默认 `5.0`
- `DONE` 和 `RECOVER` 会提前返回，不再持续触发 snapshot 过期检查

## Grasp compensation

视觉发布的 `base_mm` 不是最终抓取点。自动抓取会先经过一层 “视觉坐标 -> 抓取坐标” 补偿。

当前现场成功参数：

- `grasp_bias_x_mm: 5.0`
- `grasp_bias_y_mm: 0.0`
- `grasp_bias_z_mm: -95.0`
- `fixed_grasp_pitch_rad: 1.3963`

调参建议：

- 抓取偏前或偏后，优先调 `grasp_bias_x_mm`
- 抓取偏左或偏右，优先调 `grasp_bias_y_mm`
- 夹爪太高，减小 `grasp_bias_z_mm` 或 `grasp_offset_z_mm`
- 夹爪太低，增大 `grasp_bias_z_mm` 或 `grasp_offset_z_mm`

## Overlay

`debug_overlay_level` 支持三档：

- `none`: 只显示框和中心点，不显示文字
- `compact`: 比赛调试用，显示精简状态文字
- `full`: 排查问题用，保留详细调试信息

## Run

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/dog/roarm_ws/install/setup.bash
source /home/sunrise/dog/ros2_red_block_ws/install/setup.bash
python3 /home/sunrise/dog/ros2_red_block_ws/dog_visual_grasp_bridge/visual_moveit_grasp.py
```
