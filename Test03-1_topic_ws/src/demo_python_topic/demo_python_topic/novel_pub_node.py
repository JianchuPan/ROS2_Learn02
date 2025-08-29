import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from queue import Queue

import requests

class NovelPubNode(Node):
    def __init__(self,node_name='novel_pub_node'):
        super().__init__(node_name)
        self.get_logger().info(f'Node {node_name} has been started.')
        self.novel_queue_ = Queue() # 创建一个队列，应该放在timer前面,避免timer的回调函数调用时队列还没有创建
        self.novel_publisher_ = self.create_publisher(String, 'novel_topic', 10)
        self.timers_ = self.create_timer(5.0, self.timer_callback)
        
    def timer_callback(self):
        if self.novel_queue_.qsize() > 0:
            novel_line = self.novel_queue_.get()
            msg = String() # 创建消息对象
            msg.data = novel_line
            self.novel_publisher_.publish(msg)
            self.get_logger().info(f'Published: "{msg.data}"')

    def download(self, url):
        try:
            response = requests.get(url)
            response.raise_for_status()  # Check for HTTP errors
            response.encoding = 'utf-8'
            text = response.text
            self.get_logger().info(f"下载{url}:,{len(text)}字节")
            for line in text.splitlines():
                self.novel_queue_.put(line)
            
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Error downloading the novel: {e}")
            return None

def main():
    rclpy.init()
    node = NovelPubNode('novel_pub_node')
    # 需要县先开启一个本地的HTTP服务器，例如使用Python的http.server模块
    # 运行命令: python3 -m http.server 8081 # 在哪个目录运行就会以哪个目录为根目录
    node.download('http://0.0.0.0:8081/novels/novel1.txt')
    # node.download('https://www.ihuaben.com/book/12699044/77334174.html')
    
    rclpy.spin(node)
    rclpy.shutdown()