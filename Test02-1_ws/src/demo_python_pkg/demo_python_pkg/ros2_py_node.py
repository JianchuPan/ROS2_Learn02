#!/home/pjc/miniconda3/envs/ros2_humble/bin/python3
# 代表默认Python解释器路径
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = Node("hello_world_pynode") # 创建节点
    node.get_logger().info("Hello, ROS2 World!") # 打印日志信息
    node.get_logger().warn("warn Hello, ROS2 World!") # 打印日志信息
    rclpy.spin(node) # 保持节点运行
    node.destroy_node() # 销毁节点
    rclpy.shutdown() # 关闭rclpy