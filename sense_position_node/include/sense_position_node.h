// Copyright (c) 2023，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SENSE_POSITION_NODE_H_
#define SENSE_POSITION_NODE_H_

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/region_of_interest.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster_node.hpp"
#include "tf2_ros/transform_listener.h"

#include "ai_msgs/msg/perception_targets.hpp"
#include "hobot_autonomous_moving_msgs/srv/get_location.hpp"

using rclcpp::NodeOptions;

class echoListener
{
public:
  tf2_ros::Buffer buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;

  //constructor with name
  echoListener(rclcpp::Clock::SharedPtr clock) :
    buffer_(clock)
  {
    tfl_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
  };

  ~echoListener()
  {

  };
};

struct compare_frame {
  bool operator()(const sensor_msgs::msg::Image::SharedPtr f1,
                  const sensor_msgs::msg::Image::SharedPtr f2) {
    return ((f1->header.stamp.sec > f2->header.stamp.sec) ||
            ((f1->header.stamp.sec == f2->header.stamp.sec) &&
             (f1->header.stamp.nanosec > f2->header.stamp.nanosec)));
  }
};
struct compare_msg {
  bool operator()(const ai_msgs::msg::PerceptionTargets::SharedPtr m1,
                  const ai_msgs::msg::PerceptionTargets::SharedPtr m2) {
    return ((m1->header.stamp.sec > m2->header.stamp.sec) ||
            ((m1->header.stamp.sec == m2->header.stamp.sec) &&
             (m1->header.stamp.nanosec > m2->header.stamp.nanosec)));
  }
};
struct compare_tf {
  bool operator()(const geometry_msgs::msg::TransformStamped t1,
                  const geometry_msgs::msg::TransformStamped t2) {
    return ((t1.header.stamp.sec > t2.header.stamp.sec) ||
            ((t1.header.stamp.sec == t2.header.stamp.sec) &&
             (t1.header.stamp.nanosec > t2.header.stamp.nanosec)));
  }
};

class SensePositionNode : public rclcpp::Node {
 public:

  SensePositionNode(const std::string &node_name,
        const NodeOptions &options = NodeOptions());

  virtual ~SensePositionNode();

  // 监听TF话题消息
  int Listener();

  // 发布TF订阅消息
  int Broadcaster(geometry_msgs::msg::TransformStamped tf_msg);

  // 消息处理程序
  void MessageProcess();

  // 更新坐标系
  int UpdateTransform(sensor_msgs::msg::Image::SharedPtr frame_msg, 
                      ai_msgs::msg::PerceptionTargets::SharedPtr smart_msg,
                      geometry_msgs::msg::TransformStamped map_base_transform);

  // 通过2D结果与深度图计算目标姿态信息
  int CalculateTransform(int x, int y, float depth_value, geometry_msgs::msg::Transform& transform);

  // 更正目标的坐标系并记录
  int CorrectTransform(geometry_msgs::msg::TransformStamped& targetTF);

  // 判断两个坐标系是否重叠
  bool CheckSameTransform(geometry_msgs::msg::TransformStamped& tf1,
                        geometry_msgs::msg::TransformStamped& tf2);

  // 计算两个坐标系空间距离
  double CalculateDistance(geometry_msgs::msg::TransformStamped& tf1,
                        geometry_msgs::msg::TransformStamped& tf2);

  // 计算IOU
  bool CheckIOU(sensor_msgs::msg::RegionOfInterest& rect1, 
                    sensor_msgs::msg::RegionOfInterest& rect2);

 private:

  std::shared_ptr<std::thread> listener_thread_ = nullptr;
  std::shared_ptr<std::thread> worker_ = nullptr;

  rclcpp::Node::SharedPtr nh_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<hobot_autonomous_moving_msgs::srv::GetLocation>::SharedPtr server_;
  bool map_stop_ = false;
  std::condition_variable map_smart_condition_;
  std::mutex map_smart_mutex_;

  std::string server_name_ = "/get_target_and_position";
  std::string camera_link_name_ = "rgbd_link";

  std::string ai_msg_sub_topic_name_ = "/ai_msg_mono2d_trash_detection";
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr
      smart_subscription_ = nullptr;

  std::string depth_img_topic_name_ = "/rgbd/depth/image_raw";
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
      depth_img_subscription_ = nullptr;

  std::priority_queue<sensor_msgs::msg::Image::SharedPtr,
                      std::vector<sensor_msgs::msg::Image::SharedPtr>,
                      compare_frame> frames_;
  std::priority_queue<ai_msgs::msg::PerceptionTargets::SharedPtr,
                      std::vector<ai_msgs::msg::PerceptionTargets::SharedPtr>,
                      compare_msg> smart_msgs_;
  std::priority_queue<geometry_msgs::msg::TransformStamped,
                      std::vector<geometry_msgs::msg::TransformStamped>,
                      compare_tf> tf_msgs_;
  
  std::priority_queue<ai_msgs::msg::PerceptionTargets::SharedPtr,
                      std::vector<ai_msgs::msg::PerceptionTargets::SharedPtr>,
                      compare_msg> filter_smart_msgs_;

  geometry_msgs::msg::TransformStamped current_position_;
  std::vector<geometry_msgs::msg::TransformStamped> targetTFs_;

  void SmartTopicCallback(
      const ai_msgs::msg::PerceptionTargets::SharedPtr msg);

  void DepthImgTopicCallback(
      const sensor_msgs::msg::Image::SharedPtr msg);

  void ServiceRequest(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<hobot_autonomous_moving_msgs::srv::GetLocation::Request> request,
      const std::shared_ptr<hobot_autonomous_moving_msgs::srv::GetLocation::Response> response);

};

#endif  // SENSE_POSITION_NODE_H_