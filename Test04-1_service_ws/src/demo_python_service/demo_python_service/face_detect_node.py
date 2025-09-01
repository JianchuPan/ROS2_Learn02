import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge # ROS与OpenCV图像转换库
import face_recognition
import time
import os

class FaceDetectorionNode(Node):
    def __init__(self):
        super().__init__('face_detection_node')
        self.bridge = CvBridge()
        self.service = self.create_service(FaceDetector, '/face_detect', self.detect_face_callback)
        self.get_logger().info("Face Detection Service is Ready.")
        self.default_img_path = os.path.join(get_package_share_directory('demo_python_service'),
                                             'resource','default.jpg')
        self.upsample_times = 1
        self.model = "hog"  # 可选 "hog" 或 "cnn"

    def detect_face_callback(self, request, response):
        self.get_logger().info("Received image for face detection.")
        if request.image.data:
            cv_image = self.bridge.imgmsg_to_cv2(request.image)  # 将ROS图像消息转换为OpenCV图像
        else:
            self.get_logger().info("No image data received, using default image.")
            cv_image = cv2.imread(self.default_img_path)
        start_time = time.time()
        face_locations = face_recognition.face_locations(cv_image, number_of_times_to_upsample=self.upsample_times, 
                                                         model=self.model)
        end_time = time.time()

        num_faces = len(face_locations)
        self.get_logger().info(f"Detected {num_faces} face(s) in {(end_time - start_time):.4f} seconds.")
        response.number = num_faces
        response.use_time = float(end_time - start_time)
        for top, right, bottom, left in face_locations:
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)

        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectorionNode()
    rclpy.spin(node)
    rclpy.shutdown()

