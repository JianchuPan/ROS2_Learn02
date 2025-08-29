#!/home/pjc/miniconda3/envs/ros2_humble/bin/python3
# 代表默认Python解释器路径
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

class PersonNode(Node):
    def __init__(self, node_name:str,name:str, age:int):
        print("Person init方法被调用")
        super().__init__(node_name)
        self.name = name
        self.age = age

    def eat(self,foodName:str):
        # return f"Hello, my name is {self.name} and I am {self.age} years old,I like {foodName}."
        self.get_logger().info(f"Hello, my name is {self.name} and I am {self.age} years old,I like eat {foodName}.")

def main():
    rclpy.init()
    node = PersonNode("person_node","Alice", 30)
    node.eat("pizza")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    