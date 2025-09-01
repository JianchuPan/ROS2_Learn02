import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge # ROS与OpenCV图像转换库
from sensor_msgs.msg import Image

import os


class FaceDetectClientNode(Node):
    def __init__(self):
        super().__init__('face_detect_client_node')
        self.client = self.create_client(FaceDetector, '/face_detect')
        self.bridge = CvBridge()
        self.test1_img_path = os.path.join(get_package_share_directory('demo_python_service'),
                                             'resource','test1.jpg')
        self.image = cv2.imread(self.test1_img_path)

    def send_request(self):
        # 1.判断服务是否可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        # 2.创建请求对象
        request = FaceDetector.Request()
        # 3.填充请求数据
        request.image = self.bridge.cv2_to_imgmsg(self.image)
        # 4.发送异步请求并spin等待服务处理完成
        # # 方式1：执行spin的同时等待future完成
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        # #方式2：回调函数处理响应,进行结果处理
        # future = self.client.call_async(request)
        # def request_callback(future):
        #     response = future.result()
        #     if response is not None:
        #         self.get_logger().info(f"Number of faces detected: {response.number}")
        #         self.get_logger().info(f"Time taken for detection: {response.use_time:.4f} seconds")
        #         self.show_face_locations(response)
        #     else:
        #         self.get_logger().error('Service call failed %r' % (future.exception(),))
        # future.add_done_callback(request_callback)

        # 5.处理响应结果
        response = future.result()
        if response is not None:
            self.get_logger().info(f"Number of faces detected: {response.number}")
            self.get_logger().info(f"Time taken for detection: {response.use_time:.4f} seconds")
            self.show_face_locations(response)
        else:
            self.get_logger().error('Service call failed %r' % (future.exception(),))

    def show_face_locations(self, face_locations):
        for i in range(len(face_locations.top)):
            top = face_locations.top[i]
            right = face_locations.right[i]
            bottom = face_locations.bottom[i]
            left = face_locations.left[i]
            cv2.rectangle(self.image, (left, top), (right, bottom), (0, 255, 0), 2)
        cv2.imshow("Detected Faces", self.image)
        cv2.waitKey(0)

def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectClientNode()
    node.send_request()
    rclpy.spin(node)
    rclpy.shutdown()
