import espeakng
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from queue import Queue
import threading
import time

class NovelSubNode(Node):
    def __init__(self,node_name='novel_sub_node'):
        super().__init__(node_name)
        self.get_logger().info(f'subNode {node_name} has been started.')    
        self.novels_queue_ = Queue() # 创建一个队列，应该放在subscriber前面,避免subscriber的回调函数调用时队列还没有创建
        self.novel_subscriber_ = self.create_subscription(String, 'novel_topic', self.novel_callback, 10)
        self.speech_thread_ = threading.Thread(target=self.speech_thread_function)
        self.speech_thread_.start() # 启动线程

    def novel_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')
        self.novels_queue_.put(msg.data)
    
    def speech_thread_function(self):
        speaker = espeakng.Speaker()
        speaker.voice = 'zh'

        while rclpy.ok(): # 检测ROS rclpy是否还在运行
            if self.novels_queue_.qsize() > 0:
                novel_line = self.novels_queue_.get()
                self.get_logger().info(f'朗读: "{novel_line}"')
                speaker.say(novel_line) # 朗读文本
                speaker.wait() # 等待朗读完成
            else:
                time.sleep(1) #让当前线程休眠1000ms，避免空转占用CPU

def main():
    rclpy.init()
    node = NovelSubNode('novel_sub_node')
    rclpy.spin(node)
    rclpy.shutdown()
