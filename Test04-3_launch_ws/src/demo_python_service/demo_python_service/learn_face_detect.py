import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory
import os

def main():
    print("Face Detection using face_recognition library")
    # 读取图像文件
    default_img_path = os.path.join(get_package_share_directory('demo_python_service'),
                                    'resource','default.jpg')
    # 使用 OpenCV 读取图像
    img = cv2.imread(default_img_path) 
    # 使用 face_recognition 库检测人脸位置
    print("Detecting faces...")
    face_locations = face_recognition.face_locations(img,number_of_times_to_upsample=2,model="hog")
    print(f"Found {len(face_locations)} face(s) in the image.")
    # 在图像上绘制检测到的人脸边框
    for top, right, bottom, left in face_locations:
        # 在图像上绘制矩形框
        cv2.rectangle(img, (left, top), (right, bottom), (255, 0, 0), 4)

    # 显示图像
    cv2.imshow('Face Detection', img)
    cv2.waitKey(0)