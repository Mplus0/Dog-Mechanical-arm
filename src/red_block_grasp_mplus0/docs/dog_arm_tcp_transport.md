# 机器狗—机械臂 TCP 通信

`dog_arm_tcp_server_node` 在机械臂 ROS2 Humble 侧提供一条全双工 TCP 连接。机器狗虽然是 TCP 客户端，但连接后仍可接收机械臂主动发送的任务结果和底盘微调请求。

默认网络参数：

```text
机械臂绑定地址：192.168.31.56
允许的机器狗地址：192.168.31.192
端口：47001
```

## 密钥准备

在一台设备生成密钥：

```bash
mkdir -p ~/.ros
umask 077
openssl rand -hex 32 > ~/.ros/dog_arm_shared_secret
chmod 600 ~/.ros/dog_arm_shared_secret
```

通过可信方式把同一文件复制到另一台设备的 `~/.ros/dog_arm_shared_secret`。缺少密钥或两端密钥不同均无法建立连接。

## 单独启动传输层

```bash
source /opt/ros/humble/setup.bash
source ~/dog/ros2_red_block_ws/install/setup.bash
ros2 run red_block_grasp_mplus0 dog_arm_tcp_server_node --ros-args \
  --params-file ~/dog/ros2_red_block_ws/install/red_block_grasp_mplus0/share/red_block_grasp_mplus0/config/competition_arm.yaml
```

完整机械臂 launch 默认会同时启动该节点。IP 变化时通过 launch 参数覆盖：

```bash
ros2 launch red_block_grasp_mplus0 competition_arm_task.launch.py \
  tcp_bind_host:=192.168.31.56 \
  tcp_allowed_client_ip:=192.168.31.192 \
  tcp_port:=47001
```

## 状态话题

```bash
ros2 topic echo /dog_arm/transport_connected
ros2 topic echo /dog_arm/transport_status
```

只有完成来源 IP 检查和 HMAC 双向认证后，`transport_connected` 才为 `true`。所有协议帧都带有 HMAC 完整性签名；协议还包含心跳、断线检测、未确认消息重发和基于任务编号的重复执行抑制。

该连接不对业务内容加密，仅应在受控局域网中使用。
