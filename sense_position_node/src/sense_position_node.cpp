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

bool isSubset(const std::string& strA, const std::string& strB) {
    return strB.find(strA) != std::string::npos;
}

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

    // Test();

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
  
    worker_ = std::make_shared<std::thread>(std::bind(&SensePositionNode::MessageProcess, this));

    // 创建一个服务并指定回调函数
    server_ = create_service<hobot_autonomous_moving_msgs::srv::GetLocation>(
      server_name_, std::bind(&SensePositionNode::ServiceRequest, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));                       

}

SensePositionNode::~SensePositionNode() {
  {
    if (worker_ && worker_->joinable()) {
      map_stop_ = true;
      map_smart_condition_.notify_one();
      worker_->join();
      worker_ = nullptr;
      RCLCPP_INFO(nh_->get_logger(), "sense_position stop worker");
    }
    {
      std::unique_lock<std::mutex> lock(map_smart_mutex_);
      while (!frames_.empty()) {
        frames_.pop();
      }
    }
    {
      std::unique_lock<std::mutex> lock(map_smart_mutex_);
      while (!smart_msgs_.empty()) {
        smart_msgs_.pop();
      }
    }
    {
      std::unique_lock<std::mutex> lock(map_smart_mutex_);
      while (!tf_msgs_.empty()) {
        tf_msgs_.pop();
      }
    }
  }

}

void SensePositionNode::Test() {

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = this->now();
  tf.header.frame_id = "map";
  tf.child_frame_id = "apple_frame";

  tf.transform.translation.x = 2.0;
  tf.transform.translation.y = 1.0;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation.x = 0.0;
  tf.transform.rotation.y = 0.0;
  tf.transform.rotation.z = 0.0;
  tf.transform.rotation.w = 1.0;

  targetTFs_.push_back(tf);

  tf.header.stamp = this->now();
  tf.header.frame_id = "map";
  tf.child_frame_id = "banana_frame";

  tf.transform.translation.x = 2.0;
  tf.transform.translation.y = 2.0;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation.x = 0.0;
  tf.transform.rotation.y = 0.0;
  tf.transform.rotation.z = 0.0;
  tf.transform.rotation.w = 1.0;

  targetTFs_.push_back(tf);


  tf.header.stamp = this->now();
  tf.header.frame_id = "map";
  tf.child_frame_id = "peach_frame";

  tf.transform.translation.x = -2.0;
  tf.transform.translation.y = -1.0;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation.x = 0.0;
  tf.transform.rotation.y = 0.0;
  tf.transform.rotation.z = 0.0;
  tf.transform.rotation.w = 1.0;

  targetTFs_.push_back(tf);
}

/**
 * @brief 处理获取位置服务请求的函数
 * 
 * @param request_header 客户端请求头信息
 * @param request 客户端发送的请求数据
 * @param response 用于设置服务端的响应数据
 */
void SensePositionNode::ServiceRequest(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<hobot_autonomous_moving_msgs::srv::GetLocation::Request> request,
  const std::shared_ptr<hobot_autonomous_moving_msgs::srv::GetLocation::Response> response)
{
  // 在这里处理服务请求，生成响应
  // request 包含客户端发送的请求数据
  // response 用于设置服务端的响应数据
  // 可以通过设置 response->result = ... 来填充响应数据

  // 初始化请求类型和重叠距离
  std::string request_type;
  double overlap = 10000.0;

  // 遍历目标坐标系列表
  for (auto targetTF: targetTFs_) {
    // 判断请求类型是否是当前目标坐标系的子集
    if (isSubset(request->type, targetTF.child_frame_id)) {
      // 计算当前位置到目标坐标系的距离
      double distance = CalculateDistance(current_position_, targetTF);
      // 更新最小距离和对应的请求类型
      if (distance < overlap) {
        overlap = distance;
        request_type = targetTF.child_frame_id;
      }
    }
  }
  
  // 根据最小距离找到对应的目标坐标系
  for (auto targetTF: targetTFs_) {
    if (targetTF.child_frame_id == request_type) {
      // 填充响应数据
      response->success = true;
      response->dis_isvalid = true;
      response->dis_x = targetTF.transform.translation.x;
      response->dis_y = targetTF.transform.translation.y;
      
      response->goal.pose.position.x = targetTF.transform.translation.x;
      response->goal.pose.position.y = targetTF.transform.translation.y;
      response->goal.pose.position.z = targetTF.transform.translation.z;
      response->goal.pose.orientation = targetTF.transform.rotation;
      
      // 发送成功的响应
      server_->send_response(*request_header, *response);
      return;
    }
  }

  // 若未找到匹配的目标坐标系，发送失败的响应
  response->success = false;
  server_->send_response(*request_header, *response);
}


/**
 * @brief 处理消息的函数
 * 
 * @details 该函数用于处理智能消息、帧和坐标变换消息的同步，确保它们在时间上的匹配。
 *          在消息队列非空的情况下，通过等待条件变量触发消息处理。
 * 
 * @note 该函数在一个循环中运行，直到收到停止信号（map_stop_为true）。
 */
void SensePositionNode::MessageProcess() {
  while (!map_stop_) {
    // 使用互斥锁等待条件变量触发
    std::unique_lock<std::mutex> lock(map_smart_mutex_);
    map_smart_condition_.wait(lock);
    
    // 若收到停止信号，则退出循环
    if (map_stop_) {
      break;
    }

    // 处理智能消息、帧和坐标变换消息的同步
    while (!smart_msgs_.empty() && !frames_.empty() && !tf_msgs_.empty()) {
      auto msg = smart_msgs_.top();
      auto frame = frames_.top();

      // 判断智能消息和帧的时间戳是否匹配
      if (msg->header.stamp == frame->header.stamp) {
        geometry_msgs::msg::TransformStamped tf_msg;

        // 处理坐标变换消息，确保其时间戳不早于当前智能消息的时间戳
        while (!tf_msgs_.empty() && 
                tf_msgs_.top().header.stamp.sec != 0 && 
               ((msg->header.stamp.sec > tf_msgs_.top().header.stamp.sec) ||
                ((msg->header.stamp.sec == tf_msgs_.top().header.stamp.sec) &&
                 (msg->header.stamp.nanosec > tf_msgs_.top().header.stamp.nanosec)))) {
          tf_msg = tf_msgs_.top();
          tf_msgs_.pop();
        } 

        // 更新坐标变换并处理完成的消息
        lock.unlock();
        int ret = UpdateTransform(frame, msg, tf_msg);

        if (ret != 0) {
          RCLCPP_ERROR(rclcpp::get_logger("sense_position"), "transform update failed!");
        }
        lock.lock();
        smart_msgs_.pop();
        frames_.pop();
      } else if ((msg->header.stamp.sec > frame->header.stamp.sec) ||
                 ((msg->header.stamp.sec == frame->header.stamp.sec) &&
                  (msg->header.stamp.nanosec > frame->header.stamp.nanosec))) {
        // 当前智能消息较新，丢弃帧
        frames_.pop();
      } else {
        // 当前帧较新，丢弃智能消息
        smart_msgs_.pop();
      }
    }
  }
}

/**
 * @brief 更新坐标变换并广播目标框架之间的变换关系
 *
 * 此函数基于传感器数据和目标检测结果，计算并更新每个目标框架相对于参考框架的坐标变换。
 * 然后通过tf2库将这些变换关系广播到ROS 2的变换树中。
 *
 * @param frame_msg 传感器数据，Image消息类型
 * @param smart_msg 智能感知消息，PerceptionTargets消息类型
 * @param map_base_transform 传感器坐标系到参考坐标系的变换
 * @return 返回0表示成功，非0表示失败
 */
int SensePositionNode::UpdateTransform(sensor_msgs::msg::Image::SharedPtr frame_msg, 
                  ai_msgs::msg::PerceptionTargets::SharedPtr smart_msg,
                  geometry_msgs::msg::TransformStamped map_base_transform) {
  
  std::stringstream ss;
  ss << "\n";

  // 1. 根据当前智能结果, 转换为TF坐标系结果
  std::vector<geometry_msgs::msg::TransformStamped> currentTFs;
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

      if(map_target_transform.header.stamp.sec == 0) {
        break;
      }
      currentTFs.push_back(map_target_transform);
      
    }
  }

  // 2. 根据当前视野, 当前智能结果, 删除坐标系中消失目标
  std::vector<geometry_msgs::msg::TransformStamped> newtargetTFs;
  for (auto targetTF: targetTFs_) {
    if (!InView(targetTF, map_base_transform)) {
      newtargetTFs.push_back(targetTF);
    } else {
      for (auto& currentTF: currentTFs) {
      // 判断修正后的坐标变换是否与已知坐标系匹配
        if (isSubset(currentTF.child_frame_id, currentTF.child_frame_id) && CheckSameTransform(currentTF, targetTF)) {
          newtargetTFs.push_back(targetTF);
          break;
        }
      }
    }
  }
  swap(newtargetTFs, targetTFs_);

  // 3. 根据当前智能结果, 新增坐标系中目标
  for (auto currentTF: currentTFs) {
    CorrectTransform(currentTF);
  }

  // 4. 发布更新的TF坐标系
  Broadcaster();
  
  RCLCPP_INFO(rclcpp::get_logger("SensePositionNode"), "%s", ss.str().c_str());
  return 0;             
}

/**
 * @brief 监听坐标变换的函数
 * 
 * @return 成功返回0，否则返回错误码
 */
int SensePositionNode::Listener() {
  // 设置监听频率
  double rate_hz = 1.0;
  rclcpp::Rate rate(rate_hz);

  // 创建ROS 2节点
  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("tf2_echo");
  rclcpp::Clock::SharedPtr clock = nh->get_clock();

  // 实例化本地监听器
  echoListener echoListener(clock);

  // 设置源坐标系和目标坐标系
  std::string source_frameid = "map";
  std::string target_frameid = camera_link_name_;

  // 等待第一个坐标变换可用
  std::string warning_msg;
  while (rclcpp::ok() && !echoListener.buffer_.canTransform(
    source_frameid, target_frameid, tf2::TimePoint(), &warning_msg))
  {
    RCLCPP_INFO_THROTTLE(nh->get_logger(), *clock, 1000, "Waiting for transform %s ->  %s: %s",
      source_frameid.c_str(), target_frameid.c_str(), warning_msg.c_str());
    rate.sleep();
  }

  // 监听坐标变换并将其加入队列
  while(rclcpp::ok()) {
    try {
      current_position_ = echoListener.buffer_.lookupTransform(source_frameid, target_frameid, tf2::TimePoint());

       // 使用互斥锁确保对消息队列的安全访问
      {
        std::unique_lock<std::mutex> lock(map_smart_mutex_);
        tf_msgs_.push(current_position_);
        if (tf_msgs_.size() > 100) {
          tf_msgs_.pop();
          RCLCPP_WARN(rclcpp::get_logger("sense_position"),
                      "visual understanding node has cache tf message num > 100, drop the "
                      "oldest tf message");
        }
        // 通知处理线程有新消息可处理
        map_smart_condition_.notify_one();
      }

    } catch(tf2::TransformException& ex) {
      std::stringstream ss;
      ss << "Failure at "<< clock->now().seconds();
      ss << " Exception thrown:" << ex.what();
      ss << " The current list of frames is: ";
      ss << echoListener.buffer_.allFramesAsString();
      RCLCPP_ERROR(rclcpp::get_logger("sense_position"), "%s", ss.str().c_str());
    }
    // 控制监听频率
    rate.sleep();
  }

  return 0;
}


/**
 * @brief 广播坐标变换消息的函数
 * 
 * @return 成功返回0，否则返回错误码
 */
int SensePositionNode::Broadcaster() {

  // 创建静态坐标变换广播器
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
  
  std::stringstream ss;
  ss << "\n";
  for (auto& tf_msg: targetTFs_) {
    // 检查 frame_id 和 child_frame_id 是否相同
    if (tf_msg.header.frame_id == tf_msg.child_frame_id) {
      RCLCPP_ERROR(this->get_logger(),
        "cannot publish static transform from '%s' to '%s', exiting",
        tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str());
      throw std::runtime_error("child_frame_id cannot equal frame_id");
    }
    
    auto translation = tf_msg.transform.translation;
    auto rotation = tf_msg.transform.rotation;
    ss << "- Target Frame: " << tf_msg.child_frame_id << "\n";
    ss << "- Header: [" << tf_msg.header.stamp.sec << "_" << tf_msg.header.stamp.nanosec << "]" <<"\n";
    ss << "- Translation: [" << translation.x << ", " << translation.y << ", " << translation.z << "]" << "\n";
    ss << "- Rotation: in Quaternion [" << rotation.x << ", " << rotation.y << ", " 
              << rotation.z << ", " << rotation.w << "]" << "\n\n";

    // 发送坐标变换消息
    broadcaster_->sendTransform(tf_msg);
  }
  RCLCPP_INFO(rclcpp::get_logger("SensePositionNode"), "%s", ss.str().c_str());

  // 打印更新成功的日志信息
  RCLCPP_INFO(rclcpp::get_logger("sense_position"), "update success");
  return 0;
}

/**
 * @brief 深度图像消息的回调函数
 * 
 * @param msg 深度图像消息
 */
void SensePositionNode::DepthImgTopicCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {

  // 使用互斥锁确保对消息队列的安全访问
  std::unique_lock<std::mutex> lock(map_smart_mutex_);

  // 将接收到的深度图像消息加入队列
  frames_.push(msg);

  // 控制消息队列长度，保持不超过100条，超过则丢弃最旧的消息
  if (frames_.size() > 100) {
    frames_.pop();
    RCLCPP_WARN(rclcpp::get_logger("sense_position"),
                "visual understanding node has cache image num > 100, drop the oldest "
                "image message");
  }
  // 通知处理线程有新消息可处理
  map_smart_condition_.notify_one();
}

/**
 * @brief 智能感知消息的回调函数
 * 
 * @param msg 智能感知消息
 */
void SensePositionNode::SmartTopicCallback(
    const ai_msgs::msg::PerceptionTargets::SharedPtr msg) {
  
  ai_msgs::msg::PerceptionTargets::SharedPtr avg_msg = std::make_shared<ai_msgs::msg::PerceptionTargets>();

  if (isFilter_) {
    filter_smart_msgs_.push(msg);
    if (filter_smart_msgs_.size() < 2) {
      return;
    }

    std::queue<ai_msgs::msg::PerceptionTargets::SharedPtr> tmp_smart_msgs;

    for (const auto& tar : msg->targets) {
      auto rect1 = tar.rois[0].rect;
      std::string type1 = tar.rois[0].type;
      float confidence1 = tar.rois[0].confidence;
      int count = 0;

      while (!filter_smart_msgs_.empty()) {
        auto smart_msg = filter_smart_msgs_.top();
        tmp_smart_msgs.push(smart_msg);
        filter_smart_msgs_.pop();
        for (const auto& tmp_tar : smart_msg->targets) {
          auto rect2 = tmp_tar.rois[0].rect;
          std::string type2 = tmp_tar.rois[0].type;
          if (type1 == type2 && CheckIOU(rect1, rect2)) {
            count++;
            break;
          }
        }
      }

      if (count >= 1) {
        avg_msg->targets.emplace_back(std::move(tar));
      }

      while (!tmp_smart_msgs.empty()) {
        auto smart_msg = tmp_smart_msgs.front();
        filter_smart_msgs_.push(smart_msg);
        tmp_smart_msgs.pop();
      }
    }
    filter_smart_msgs_.pop();
  }


  avg_msg->header = msg->header;
  avg_msg->targets = msg->targets;

  {
    // 使用互斥锁确保对消息队列的安全访问
    std::unique_lock<std::mutex> lock(map_smart_mutex_);

    // 将接收到的智能感知消息加入队列
    if (smart_msgs_.empty()) {
      // smart_msgs_.push(msg);
      smart_msgs_.push(avg_msg);
    }

    // 控制消息队列长度，保持不超过100条，超过则丢弃最旧的消息
    if (smart_msgs_.size() > 100) {
      smart_msgs_.pop();
      RCLCPP_WARN(rclcpp::get_logger("sense_position"),
                  "visual understanding node has cache smart message num > 100, drop the "
                  "oldest smart message");
    }
    
    // 通知处理线程有新消息可处理
    map_smart_condition_.notify_one();
  }
}


/**
 * @brief 根据像素坐标和深度值计算坐标变换的函数
 * 
 * @param x 横向像素坐标
 * @param y 纵向像素坐标
 * @param depth_value 深度值
 * @param transform 计算得到的坐标变换结果
 * @return 成功返回0，否则返回错误码
 */
int SensePositionNode::CalculateTransform(int x, int y, float depth_value, geometry_msgs::msg::Transform& transform) {
  
  // 相机参数
  double fx = 554.254691191187;  // 焦距 x
  double fy = 554.254691191187;  // 焦距 y
  double cx = 320.5;             // 光心 x
  double cy = 240.5;             // 光心 y

  // 像素坐标转换为相机坐标
  double pointx = (x - cx) * depth_value / fx;
  double pointy = (y - cy) * depth_value / fy;
  double pointz = depth_value;

  // 相机坐标转换为目标坐标系坐标
  float targetX = static_cast<float>(pointz);
  float targetY = static_cast<float>(-pointx);
  float targetZ = static_cast<float>(pointy);

  // 保留两位小数
  targetX = static_cast<float>(static_cast<int>(targetX * 100)) / 100.0;
  targetY = static_cast<float>(static_cast<int>(targetY * 100)) / 100.0;

  // 填充坐标变换结果
  transform.translation.x = targetX;
  transform.translation.y = targetY;
  transform.translation.z = 0.0;
  transform.rotation.x = 0.0;
  transform.rotation.y = 0.0;

  // 更新目标物体的朝向角，使其与机器人视角朝向相同
  double Xa = current_position_.transform.translation.x;
  double Ya = current_position_.transform.translation.y;
  double Xb = transform.translation.x;
  double Yb = transform.translation.y;
  double theta = atan2((Yb - Ya), (Xb - Xa)); // 计算朝向角
  transform.rotation.z = sin(theta / 2.0);
  transform.rotation.w = cos(theta / 2.0);

  return 0;
}


/**
 * @brief 修正坐标变换的函数
 * 
 * @param tempTF 待修正的坐标变换消息（传入引用，将被修改）
 * @return 成功返回0，否则返回错误码
 */
int SensePositionNode::CorrectTransform(geometry_msgs::msg::TransformStamped& tempTF) {

  // 遍历目标坐标系列表
  for (auto& targetTF : targetTFs_) {
    // 判断修正后的坐标变换是否与已知坐标系匹配
    if (isSubset(tempTF.child_frame_id, targetTF.child_frame_id) && CheckSameTransform(tempTF, targetTF)) {
      // 若匹配，则更新坐标系标识和位置信息（取平均）
      tempTF.child_frame_id = targetTF.child_frame_id;
      tempTF.transform.translation.x = (tempTF.transform.translation.x + targetTF.transform.translation.x) / 2;
      tempTF.transform.translation.y = (tempTF.transform.translation.y + targetTF.transform.translation.y) / 2;
      tempTF.transform.translation.z = (tempTF.transform.translation.z + targetTF.transform.translation.z) / 2;
      return 0;
    }
  }

  // 若未找到匹配的已知坐标系，生成新的坐标系标识
  int count = 0;
  for (auto& targetTF : targetTFs_) {
    if (isSubset(tempTF.child_frame_id, targetTF.child_frame_id)) {
      if (count != 0) {
        targetTF.child_frame_id = tempTF.child_frame_id + "_" + std::to_string(count);
      }
      count++;
    }
  }
  if (count != 0) {
    tempTF.child_frame_id = tempTF.child_frame_id + "_" + std::to_string(count);
  }

  // 将修正后的坐标变换添加到目标坐标系列表
  targetTFs_.push_back(tempTF);
  return 0;
}


/**
 * @brief 检查两个坐标变换是否相似的函数
 * 
 * @param tf1 第一个坐标变换
 * @param tf2 第二个坐标变换
 * @return 如果两坐标变换相似，返回true；否则返回false
 */
bool SensePositionNode::CheckSameTransform(geometry_msgs::msg::TransformStamped& tf1,
                                          geometry_msgs::msg::TransformStamped& tf2) {
  
  // 初始化重叠阈值，默认为0.2
  double overlap_threshold = 1.0;

  // 根据坐标变换类型调整重叠阈值
  std::string type = tf1.child_frame_id;
  if (type == "trashcan_frame") {
    overlap_threshold = 1.0;
  } else if (type == "round_frame") {
    overlap_threshold = 0.2;
  }

  // 计算两坐标变换之间的欧几里得距离
  double distance = CalculateDistance(tf1, tf2);

  // 判断距离是否小于重叠阈值，若是，则认为相似
  if (distance < overlap_threshold) {
    return true;
  }

  // 若距离大于等于重叠阈值，则认为不相似
  return false;
}


/**
 * @brief 计算两个坐标变换之间的欧几里得距离
 * 
 * @param tf1 第一个坐标变换
 * @param tf2 第二个坐标变换
 * @return 返回两坐标变换之间的欧几里得距离
 */
double SensePositionNode::CalculateDistance(geometry_msgs::msg::TransformStamped& tf1,
                                          geometry_msgs::msg::TransformStamped& tf2) {
  double x1 = tf1.transform.translation.x;
  double y1 = tf1.transform.translation.y;
  double x2 = tf2.transform.translation.x;
  double y2 = tf2.transform.translation.y;
  double distance = std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
  return distance;
}


/**
 * @brief 检查两个矩形区域的交并比（Intersection over Union，简称IOU）是否显著
 * 
 * 交并比（IOU）是衡量两个矩形区域重叠程度的指标。本函数计算给定两个矩形区域的IOU值，
 * 并通过阈值判断它们是否有显著的重叠。
 * 
 * @param rect1 第一个矩形区域
 * @param rect2 第二个矩形区域
 * @return 如果IOU大于0.7，则返回true，表示显著的重叠；否则返回false
 */
bool SensePositionNode::CheckIOU(sensor_msgs::msg::RegionOfInterest& rect1, 
                                    sensor_msgs::msg::RegionOfInterest& rect2) {
    // 计算两个矩形区域的交集坐标
    uint32_t x1 = std::max(rect1.x_offset, rect2.x_offset);
    uint32_t y1 = std::max(rect1.y_offset, rect2.y_offset);
    uint32_t x2 = std::min(rect1.x_offset + rect1.width, rect2.x_offset + rect2.width);
    uint32_t y2 = std::min(rect1.y_offset + rect1.height, rect2.y_offset + rect2.height);

    // 计算交集和并集的面积
    double intersection = std::max(0.0, static_cast<double>(x2 - x1)) * std::max(0.0, static_cast<double>(y2 - y1));
    double union_area = rect1.width * rect1.height + rect2.width * rect2.height - intersection;

    // 计算交并比（IOU
    double iou = intersection / union_area;
    
    return iou > 0.7;
}

/**
 * @brief 检查相机视野中目标是否可见
 * 
 * 该函数判断相机视野中是否可见目标，通过坐标变换计算目标在相机坐标系下的位置，并检查其是否在可见范围内。
 * 
 * @param tf 目标在地图坐标系下的变换信息
 * @return 如果目标在相机视野内，返回true；否则返回false
 */
bool SensePositionNode::InView(geometry_msgs::msg::TransformStamped& map_targrt_transform,
                          geometry_msgs::msg::TransformStamped& map_base_transform) {
  
  // 获取当前机器人在地图坐标系下的反转变换
  Eigen::Isometry3d M = tf2::transformToEigen(map_base_transform);
  geometry_msgs::msg::TransformStamped base_map_transform = tf2::eigenToTransform(M.inverse());
  base_map_transform.header.stamp = map_targrt_transform.header.stamp;
  base_map_transform.header.frame_id = camera_link_name_;
  base_map_transform.child_frame_id = "map";

  // 创建目标在相机坐标系下的变换信息
  geometry_msgs::msg::TransformStamped base_target_transform;
  base_target_transform.header.stamp = map_targrt_transform.header.stamp;
  base_target_transform.header.frame_id = camera_link_name_;
  base_target_transform.child_frame_id = map_targrt_transform.child_frame_id;

  try
  {
      tf2::doTransform(map_targrt_transform, base_target_transform, base_map_transform);
  }
  catch (tf2::TransformException &ex)
  {
      RCLCPP_ERROR(rclcpp::get_logger("sense_position"), "Transform map to target failed: %s", ex.what());
  }

  // 获取目标在相机坐标系下的平移和旋转信息
  auto translation = base_target_transform.transform.translation;
  auto rotation = base_target_transform.transform.rotation;

  // 检查目标是否在相机视野内，满足条件则返回true，否则返回false
  if (translation.x < 2.0 && translation.x >= abs(translation.y)) {
    return true;
  }

  return false;
}