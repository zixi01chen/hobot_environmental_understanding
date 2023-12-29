import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2 as cv
import os
import numpy as np
import datetime
import time
from sensor_msgs.msg import Image

import requests
import json
import base64

from ai_msgs.msg import PerceptionTargets, Roi, Target


class ImageSubscriber(Node):

  def __init__(self):
    super().__init__('ImageSubscriber')

    # self.topic_name = '/camera/image_raw'
    self.topic_name = '/rgb/image_raw'

    # 创建sub节点，订阅image_raw话题
    self.subscription = self.create_subscription(
      Image,
      self.topic_name,
      self.listener_callback,
      1)

    # 创建sub节点，订阅depth话题
    self.subscription = self.create_subscription(
      Image,
      '/rgbd/depth/image_raw',
      self.depth_callback,
      1)

    self.declare_parameter("fps",2.0)
    self.declare_parameter("detection_result_folder","detection_result")
    self.declare_parameter("detection_image_folder","detection_image")
    # 创建CvBridge实例
    self.bridge = CvBridge()
    self.detection_image = np.zeros((640, 480, 3))
    self.start_time = time.time()
    self.time_diff_threshold = 1 / self.get_parameter('fps').get_parameter_value().double_value
    self.name = ""

    os.system("rm -rf detection_result")
    os.system("rm -rf detection_image")
    os.system("rm -rf detection_depth")

    self.detection_result_folder = self.get_parameter('detection_result_folder').get_parameter_value().string_value
    self.detection_image_folder = self.get_parameter('detection_image_folder').get_parameter_value().string_value
    self.detection_depth_folder = "detection_depth"
    # 检查用户存放标定数据的image_dataset文件夹是否存在，如果不存在则创建
    if not os.path.exists(self.detection_result_folder):
      os.makedirs(self.detection_result_folder)
    if not os.path.exists(self.detection_image_folder):
      os.makedirs(self.detection_image_folder)
    if not os.path.exists(self.detection_depth_folder):
      os.makedirs(self.detection_depth_folder)
    # 设置opecv展示窗属性

    self.subscription

    self.text_prompts = ["trashcan", "round"]
    self.last_time = time.time()
    self.publisher = self.create_publisher(PerceptionTargets, '/ai_msg_mono2d_trash_detection', 10)

  def plot_boxes_to_image_cv(self, image, res_msg):
      # 获取检测框信息
      boxes = res_msg['boxes']
      labels = res_msg['labels']
      H, W = res_msg['size']

      # 在图像上画框
      for box, label in zip(boxes, labels):

        box = box * np.array([W, H, W, H])
        # from xywh to xyxy
        box[:2] -= box[2:] / 2
        box[2:] += box[:2]
        # random color
        color = tuple(np.random.randint(0, 255, size=3).tolist())
        # draw
        x0, y0, x1, y1 = box
        x0, y0, x1, y1 = int(x0), int(y0), int(x1), int(y1)

        # 画矩形框
        thickness = 2
        cv.rectangle(image, (x0, y0), (x1, y1), color, thickness)

        # 添加标签
        text = f'{label}'
        cv.putText(image, text, (x0, y0 - 5), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness)

      return image

  def merge_dicts(self, dict1, dict2):
      """
      递归合并两个字典，相同键值合并
      """
      merged = dict1.copy()
      for key, value in dict2.items():
        if key == "boxes":
          merged[key].extend(value)
        elif key == "labels":
          merged[key].extend(value)
      return merged
    
  # sub回调函数
  def listener_callback(self, msg):
    end_time = time.time()
    time_diff = end_time - self.start_time

    self.name = str(msg.header.stamp.sec) + "_" + str(msg.header.stamp.nanosec)
    
    if time_diff > self.time_diff_threshold:
      self.start_time = end_time
      self.detection_image = self.bridge.imgmsg_to_cv2(msg)

      self.detection_image = cv.cvtColor(self.detection_image, cv.COLOR_BGR2RGB)

      _, image_data = cv.imencode('.jpg', self.detection_image)
      # 将字节数据转换为 Base64 字符串
      img_byte = base64.b64encode(image_data).decode('utf-8')

      res_msgs = []
      for text_prompt in self.text_prompts:

        request_msg={'text_prompt':text_prompt, 'image':img_byte, 'box_threshold':0.7}
        data = json.dumps(request_msg)  #字典数据结构变json(所有程序语言都认识的字符串)

        res = requests.post('http://10.64.29.52:8647', data=data)
        res_msg = dict(res.json())
        res_msgs.append(res_msg)
        print(res_msg)

      res_msg = self.merge_dicts(res_msgs[0], res_msgs[1])

      print("merged: ", res_msg)

      boxes = res_msg['boxes']
      labels = res_msg['labels']
      H, W = res_msg['size']

      # draw pic
      pic_name = str()
      # 保存上一次标定的结果
      cv.imwrite(os.path.join(self.detection_image_folder, self.name + '.jpg'), self.detection_image)

      image_with_box = self.plot_boxes_to_image_cv(self.detection_image, res_msg)
      cv.imwrite(os.path.join(self.detection_result_folder, self.name + '.jpg'), image_with_box)
      
      print("Save "+ self.name + ".jpg")
      # 载入新的图像

      if boxes != []:
        box = boxes[0] * np.array([W, H, W, H])
        # from xywh to xyxy
        box[:2] -= box[2:] / 2
        box[2:] += box[:2]
        # draw
        x0, y0, x1, y1 = box
        x0, y0, x1, y1 = int(x0), int(y0), int(x1), int(y1)
        
        print("box: ", x0, " ", y0, " ", x1, " ", y1)

      num = len(boxes)

      # 创建PerceptionTargets消息
      pub_data = PerceptionTargets()
      pub_data.header = msg.header

      for i in range(num):

        # 创建Roi消息
        roi = Roi()
        roi.type = labels[i].split('(')[0]
        roi.rect.x_offset = int((boxes[i][0] - boxes[i][2] / 2) * W)
        roi.rect.y_offset = int((boxes[i][1] - boxes[i][3] / 2) * H)
        roi.rect.width = int(boxes[i][2] * W)
        roi.rect.height = int(boxes[i][3] * H)
        roi.confidence = float(labels[i].split('(')[1].split(')')[0])

        # 创建Target消息
        target = Target()
        target.type = labels[i].split('(')[0]
        target.rois.append(roi)
        pub_data.targets.append(target)


      current_time = time.time()
      cost_time = current_time - self.last_time
      self.last_time = current_time

      self.get_logger().info(f"Publishing: {pub_data} ")
      self.get_logger().info(f"cost_time: {int(cost_time * 1000)} ms")
      self.get_logger().info(f"fps: {1 / (cost_time)}")

      self.publisher.publish(pub_data)

  def depth_callback(self, msg):

    # print(msg.header)
    detection_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    
    if self.name != "":
      # 保存上一次深度图
      cv.imwrite(os.path.join(self.detection_depth_folder, self.name + '.png'), detection_image)


def main(args=None):
  rclpy.init(args=args)
  image_subscriber = ImageSubscriber()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()

