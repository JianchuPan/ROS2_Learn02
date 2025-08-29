import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NovelPubNode(Node):
    def __init__(self,node_name='novel_pub_node'):
        super().__init__(node_name)
        self.get_logger().info(f'Node {node_name} has been started.')
        self.publisher_ = self.create_publisher(String, 'novel_topic', 10)
        timer_period = 1.0

def main():
    rclpy.init()
    node = NovelPubNode('novel_pub_node')
    rclpy.spin(node)
    rclpy.shutdown()