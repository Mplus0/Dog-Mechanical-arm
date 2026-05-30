# ROS1/ROS2 bridge for the arm side

This setup keeps the bridge on the arm computer. The dog side stays on ROS1 and only publishes the high-level task command:

- `/dog_arm/task_cmd`

The arm side returns only these high-level messages through the bridge:

- `/dog_arm/task_result`
- `/dog_arm/base_adjust_req`

Do not bridge or expose the internal arm, perception, or camera topics:

- `/roarm_m3/cmd`
- `/roarm_m3/state`
- `/red_block/target_base`
- camera image topics

## Start the ROS2 arm task

On the arm computer:

```bash
ros2 launch red_block_grasp_ros2 competition_arm_ros2.launch.py show_window:=false
```

## Bridge only the competition topics

Run `ros1_bridge` on the arm computer and bridge only the three high-level `std_msgs/String` topics:

```bash
source /opt/ros/noetic/setup.bash
source /opt/ros/humble/setup.bash

export ROS_MASTER_URI=http://<狗端ROS1_IP>:11311
export ROS_IP=<机械臂X5_IP>

rosparam load /home/sunrise/dog/ros2_red_block_ws/src/red_block_grasp_ros2/docs/dog_arm_bridge.yaml
ros2 run ros1_bridge parameter_bridge
```

## Test from the ROS1 dog side

Publish a pick command:

```bash
rostopic pub -1 /dog_arm/task_cmd std_msgs/String '{"data":"{\"task_id\":1,\"cmd\":\"pick\"}"}'
```

Publish a place command:

```bash
rostopic pub -1 /dog_arm/task_cmd std_msgs/String '{"data":"{\"task_id\":2,\"cmd\":\"place_to_zone\"}"}'
```

Watch arm results:

```bash
rostopic echo /dog_arm/task_result
```

Watch base adjustment requests:

```bash
rostopic echo /dog_arm/base_adjust_req
```

## Test on the ROS2 arm side

Confirm the bridge delivers dog commands into ROS2:

```bash
ros2 topic echo /dog_arm/task_cmd
```

Watch results before they cross back to ROS1:

```bash
ros2 topic echo /dog_arm/task_result
```

Watch base adjustment requests before they cross back to ROS1:

```bash
ros2 topic echo /dog_arm/base_adjust_req
```
