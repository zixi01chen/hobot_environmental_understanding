import rclpy

from hobot_autonomous_moving_msgs.srv import GetLocation
from rclpy.node import Node
import time

class objectClient(Node):
    def __init__(self, name):
        super().__init__(name)                          # ROS2节点父类初始化
        print("create client")
        self.client = self.create_client(GetLocation, 'get_target_and_position')
        print("wait for server",end='')
        while not self.client.wait_for_service(timeout_sec=1.0):
            # self.get_logger().info('service not available, waiting again...')
            print(".", end='')
        print(".")
        self.req = GetLocation.Request()

    def send_request(self, text_prompt="cat"):
        self.req.type = text_prompt
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

if __name__ == "__main__":
    rclpy.init()
    object_client = objectClient("test_client")
    # last_text_prompt = "waste"
    last_text_prompt = "trash"
    while True:
        text_prompt = input(f"input text prompt, default is '{last_text_prompt}':")
        if text_prompt=='':
            text_prompt = last_text_prompt
        last_text_prompt = text_prompt
        response = object_client.send_request(text_prompt)


        object_client.get_logger().info(
            f'Result of get_target_and_position: {response.success}  {response.goal.pose.position.x}, {response.goal.pose.position.y}'  )
        # time.sleep(0)
    object_client.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()
