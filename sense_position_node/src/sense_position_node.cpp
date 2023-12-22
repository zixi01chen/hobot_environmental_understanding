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

#include "include/sense_position_node.h"

SensePositionNode::SensePositionNode(const std::string &node_name, const NodeOptions &options)
    : rclcpp::Node(node_name, options) {

    this->declare_parameter<std::string>("ai_msg_sub_topic_name", ai_msg_sub_topic_name_);
    this->declare_parameter<std::string>("depth_img_topic_name", depth_img_topic_name_);
    this->declare_parameter<std::string>("server_name", server_name_);
    this->declare_parameter<std::string>("camera_link_name", camera_link_name_);

    this->get_parameter<std::string>("ai_msg_sub_topic_name", ai_msg_sub_topic_name_);
    this->get_parameter<std::string>("depth_img_topic_name", depth_img_topic_name_);
    this->get_parameter<std::string>("server_name", server_name_);
    this->get_parameter<std::string>("camera_link_name", camera_link_name_);
  
    {
      std::stringstream ss;
      ss << "Parameter:"
        << "\n ai_msg_sub_topic_name: " << ai_msg_sub_topic_name_
        << "\n depth_img_topic_name: " << depth_img_topic_name_
        << "\n server_name: " << server_name_
        << "\n camera_link_name: " << camera_link_name_;
      RCLCPP_WARN(rclcpp::get_logger("sense_position"), "%s", ss.str().c_str());
    }

    smart_subscription_ =
      this->create_subscription<ai_msgs::msg::PerceptionTargets>(
          ai_msg_sub_topic_name_,
          10,
          std::bind(&SensePositionNode::SmartTopicCallback,
                    this,
                    std::placeholders::_1));

    depth_img_subscription_ =
      this->create_subscription<sensor_msgs::msg::Image>(
          depth_img_topic_name_,
          10,
          std::bind(&SensePositionNode::DepthImgTopicCallback,
                    this, 
                    std::placeholders::_1));

    listener_thread_ = std::make_shared<std::thread>(&SensePositionNode::Listener, this);

    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                            std::bind(&SensePositionNode::Main, this));

    // 创建一个服务并指定回调函数
    server_ = create_service<hobot_autonomous_moving_msgs::srv::GetLocation>(
      server_name_, std::bind(&SensePositionNode::ServiceRequest, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));                       
    
}

SensePositionNode::~SensePositionNode() {}

void SensePositionNode::ServiceRequest(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<hobot_autonomous_moving_msgs::srv::GetLocation::Request> request,
  const std::shared_ptr<hobot_autonomous_moving_msgs::srv::GetLocation::Response> response)
{
  // 在这里处理服务请求，生成响应
  // request 包含客户端发送的请求数据
  // response 用于设置服务端的响应数据
  // 可以通过设置 response->result = ... 来填充响应数据

  std::string request_type = request->type + "_frame";

  for (auto targetTF: targetTFs_) {
    if (targetTF.child_frame_id == request_type) {
      response->success = true;
      response->dis_isvalid = true;
      response->dis_x = targetTF.transform.translation.x;
      response->dis_y = targetTF.transform.translation.y;
      
      response->goal.pose.position.x = targetTF.transform.translation.x;
      response->goal.pose.position.y = targetTF.transform.translation.y;
      response->goal.pose.position.z = targetTF.transform.translation.z;
      response->goal.pose.orientation = targetTF.transform.rotation;
      
      server_->send_response(*request_header, *response);
      return;
    }
  }

  response->success = false;
  // 发送响应
  server_->send_response(*request_header, *response);
}

int SensePositionNode::Main() {

  while(!smart_msgs_.empty() && !frames_.empty() && !tf_msgs_.empty()) {
    auto msg = smart_msgs_.top();
    auto frame = frames_.top();
    if (msg->header.stamp == frame->header.stamp) {

      geometry_msgs::msg::TransformStamped tf_msg;
      while(!tf_msgs_.empty() && 
            (msg->header.stamp.sec > tf_msgs_.top().header.stamp.sec) ||
            ((msg->header.stamp.sec == tf_msgs_.top().header.stamp.sec) &&
            (msg->header.stamp.nanosec >
              tf_msgs_.top().header.stamp.nanosec))) {
        tf_msg = tf_msgs_.top();
        tf_msgs_.pop();
      } 

      int ret = UpdateTransform(frame, msg, tf_msg);
      if (ret != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("sense_position"), "transform update failed!");  
        return -1;
      }
      smart_msgs_.pop();
      frames_.pop();
      break;

    } else if ((msg->header.stamp.sec > frame->header.stamp.sec) ||
                ((msg->header.stamp.sec == frame->header.stamp.sec) &&
                (msg->header.stamp.nanosec >
                  frame->header.stamp.nanosec))) {
      frames_.pop();
      // RCLCPP_INFO(rclcpp::get_logger("sense_position"), "frames_.pop()");
    } else {
      smart_msgs_.pop();
      // RCLCPP_INFO(rclcpp::get_logger("sense_position"), "smart_msgs_.pop()");
    }
  }

}

int SensePositionNode::UpdateTransform(sensor_msgs::msg::Image::SharedPtr frame_msg, 
                  ai_msgs::msg::PerceptionTargets::SharedPtr smart_msg,
                  geometry_msgs::msg::TransformStamped map_base_transform) {
  
  std::stringstream ss;
  ss << "\n";

  for (const auto& tar : smart_msg->targets) {
    
    for (const auto& roi : tar.rois) {

      int x = (2 * roi.rect.x_offset + roi.rect.width) / 2;
      int y = roi.rect.y_offset + roi.rect.height;
      std::string type = roi.type;

      int image_width = frame_msg->width;
      int image_height = frame_msg->height;
      int pixel_index = y * image_width + x;
      int float_size = sizeof(float);

      // 计算像素值在数据数组中的偏移
      uint32_t data_offset = pixel_index * float_size;

      // 获取深度值（32位浮点数）
      float depth_value;
      memcpy(&depth_value, &frame_msg->data[data_offset], float_size);

      // 计算相对坐标变换 transform
      geometry_msgs::msg::Transform transform;
      CalculateTransform(x, y, depth_value, transform);

      auto stamp = this->now();
      std::string child_frame_id = type + "_frame";

      geometry_msgs::msg::TransformStamped base_target_transform;
      base_target_transform.header.stamp = stamp;
      base_target_transform.transform = transform;
      base_target_transform.header.frame_id = camera_link_name_;
      base_target_transform.child_frame_id = child_frame_id;

      geometry_msgs::msg::TransformStamped map_target_transform;
      map_target_transform.header.frame_id = "map";
      map_target_transform.child_frame_id = child_frame_id;

      try
      {
          tf2::doTransform(base_target_transform, map_target_transform, map_base_transform);
      }
      catch (tf2::TransformException &ex)
      {
          RCLCPP_ERROR(rclcpp::get_logger("sense_position"), "Transform map to target failed: %s", ex.what());
          return 0;
      }

      CorrectTransform(map_target_transform);

      auto translation = map_target_transform.transform.translation;
      auto rotation = map_target_transform.transform.rotation;

      ss << "- Target Frame: " << child_frame_id << "\n";
      ss << "- Header: [" << map_target_transform.header.stamp.sec << " " << map_target_transform.header.stamp.nanosec << "\n";
      ss << "- Translation: [" << translation.x << ", " << translation.y << ", " << translation.z << "]" << "\n";
      ss << "- Rotation: in Quaternion [" << rotation.x << ", " << rotation.y << ", " 
                << rotation.z << ", " << rotation.w << "]" << "\n";

      Broadcaster(map_target_transform);
    }

    ss << "\n";
  }
  RCLCPP_INFO(rclcpp::get_logger("SensePositionNode"), "%s", ss.str().c_str());
  return 0;             
}

int SensePositionNode::Listener() {
  double rate_hz = 1.0;

  rclcpp::Rate rate(rate_hz);

  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("tf2_echo");

  rclcpp::Clock::SharedPtr clock = nh->get_clock();
  //Instantiate a local listener
  echoListener echoListener(clock);

  std::string source_frameid = "map";
  std::string target_frameid = camera_link_name_;

  // Wait for the first transforms to become avaiable.
  std::string warning_msg;
  while (rclcpp::ok() && !echoListener.buffer_.canTransform(
    source_frameid, target_frameid, tf2::TimePoint(), &warning_msg))
  {
    RCLCPP_INFO_THROTTLE(nh->get_logger(), *clock, 1000, "Waiting for transform %s ->  %s: %s",
      source_frameid.c_str(), target_frameid.c_str(), warning_msg.c_str());
    rate.sleep();
  }

  //Nothing needs to be done except wait for a quit
  //The callbacks withing the listener class
  //will take care of everything
  while(rclcpp::ok()) {
    try {
      geometry_msgs::msg::TransformStamped echo_transform;
      echo_transform = echoListener.buffer_.lookupTransform(source_frameid, target_frameid, tf2::TimePoint());

      tf_msgs_.push(echo_transform);
      if (tf_msgs_.size() > 100) {
        tf_msgs_.pop();
        RCLCPP_WARN(rclcpp::get_logger("sense_position"),
                    "visual understanding node has cache tf message num > 100, drop the "
                    "oldest tf message");
      }

      // std::cout.precision(3);
      // std::cout.setf(std::ios::fixed,std::ios::floatfield);
      // std::cout << "At time " << echo_transform.header.stamp.sec << "." << echo_transform.header.stamp.nanosec << std::endl;

      // auto translation = echo_transform.transform.translation;
      // auto rotation = echo_transform.transform.rotation;
      // std::cout << "- Translation: [" << translation.x << ", " << translation.y << ", " << translation.z << "]" << std::endl;
      // std::cout << "- Rotation: in Quaternion [" << rotation.x << ", " << rotation.y << ", " 
      //           << rotation.z << ", " << rotation.w << "]" << std::endl;

    } catch(tf2::TransformException& ex) {
      std::stringstream ss;
      ss << "Failure at "<< clock->now().seconds();
      ss << " Exception thrown:" << ex.what();
      ss << " The current list of frames is: ";
      ss << echoListener.buffer_.allFramesAsString();
      RCLCPP_ERROR(rclcpp::get_logger("sense_position"), "%s", ss.str().c_str());
    }
    rate.sleep();
  }

  return 0;
}

int SensePositionNode::Broadcaster(geometry_msgs::msg::TransformStamped tf_msg) {

  // check frame_id != child_frame_id
  if (tf_msg.header.frame_id == tf_msg.child_frame_id) {
    RCLCPP_ERROR(this->get_logger(),
      "cannot publish static transform from '%s' to '%s', exiting",
      tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str());
    throw std::runtime_error("child_frame_id cannot equal frame_id");
  }

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

  broadcaster_->sendTransform(tf_msg);

  RCLCPP_INFO(rclcpp::get_logger("sense_position"), "update success");
  return 0;
}

void SensePositionNode::DepthImgTopicCallback(
  const sensor_msgs::msg::Image::SharedPtr msg) {

  frames_.push(msg);
  RCLCPP_INFO(rclcpp::get_logger("sense_position"),
              "frames_ size %d",
              frames_.size());
  if (frames_.size() > 100) {
    frames_.pop();
    RCLCPP_WARN(rclcpp::get_logger("sense_position"),
                "visual understanding node has cache image num > 100, drop the oldest "
                "image message");
  }
}

void SensePositionNode::SmartTopicCallback(
    const ai_msgs::msg::PerceptionTargets::SharedPtr msg) {

  if (smart_msgs_.empty()) {
    smart_msgs_.push(msg);
  }

  if (smart_msgs_.size() > 100) {
    smart_msgs_.pop();
    RCLCPP_WARN(rclcpp::get_logger("sense_position"),
                "visual understanding node has cache smart message num > 100, drop the "
                "oldest smart message");
  }
}


int SensePositionNode::CalculateTransform(int x, int y, float depth_value, geometry_msgs::msg::Transform& transform) {
  
  double fx = 554.254691191187;  // 焦距 x
  double fy = 554.254691191187;  // 焦距 y
  double cx = 320.5;             // 光心 x
  double cy = 240.5;             // 光心 y

  double pointx = (x - cx) * depth_value / fx;
  double pointy = (y - cy) * depth_value / fy;
  double pointz = depth_value;

  float targetX = static_cast<float>(pointz);
  float targetY = static_cast<float>(-pointx);
  float targetZ = static_cast<float>(pointy);

  targetX = static_cast<float>(static_cast<int>(targetX * 100)) / 100.0;
  targetY = static_cast<float>(static_cast<int>(targetY * 100)) / 100.0;

  transform.translation.x = targetX;
  transform.translation.y = targetY;
  transform.translation.z = 0.0;
  transform.rotation.x = 0.0;
  transform.rotation.y = 0.0;
  transform.rotation.z = 0.0;
  transform.rotation.w = 1.0;

  return 0;
}

int SensePositionNode::CorrectTransform(geometry_msgs::msg::TransformStamped& transform) {
  
  if (targetTFs_.empty()) {
    targetTFs_.push_back(transform);
    return 0;
  }

  for (auto targetTF: targetTFs_) {
    if (targetTF.child_frame_id == transform.child_frame_id) {
      return 0;
    }
  }
  targetTFs_.push_back(transform);

  return 0;
}
