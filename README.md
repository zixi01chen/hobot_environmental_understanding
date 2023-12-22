# hobot_environmental_understanding

## 功能介绍
  本项目用于机器人感知与环境高阶理解

## 模块说明

### server_objectdet2d_perception

功能：
  - 云端推理中继节点, 接收真实/仿真图像 sensor_msg/msg/Image 话题消息, 向云端发送推理请求, 接收并通过 ai_msgs/msg/PerceptionTarget 话题类型发布。

依赖包：
  - ai_msgs

运行：

```shell
  source /opt/ros/foxy/setup.bash
  # source 你的ROS工程包
  python3 main.py
```

### sense_position_node

功能:
  - 视觉场景理解节点, 订阅深度图 sensor_msg/msg/Image 话题消息, 订阅 ai_msgs/msg/PerceptionTarget 话题类型消息, 发布目标物体的tf坐标系信息。
  - 作为 server 服务节点, 支持服务端发送 hobot_autonomous_moving_msgs::srv::GetLocation 话题消息, 返回 response 消息。

依赖包：
  - ai_msgs
  - hobot_autonomous_moving_msgs

运行：
```shell
  source /opt/ros/foxy/setup.bash
  # source 你的ROS工程包
  ros2 run sense_position sense_position
```
