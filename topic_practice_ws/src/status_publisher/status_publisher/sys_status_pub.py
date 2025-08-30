import rclpy
from rclpy.node import Node
from status_interfaces.msg import SystemStatus
import psutil
import platform

class SysStatusPub(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f'Publisher Node {node_name} has been started.')
        self.status_publisher_ = self.create_publisher(SystemStatus, 'sys_status', 10)
        self.timer_ = self.create_timer(2.0, self.timer_callback)  # 每2秒发布一次系统状态

    def timer_callback(self):
        cpu_percent = psutil.cpu_percent()
        memory_info = psutil.virtual_memory()
        net_io_counters = psutil.net_io_counters()

        msg = SystemStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.host_name = platform.node()
        msg.cpu_percent = cpu_percent
        msg.memory_percent = memory_info.percent
        msg.memory_total = memory_info.total / (1024 * 1024)  # 转换为MB
        msg.memory_available = memory_info.available / (1024 * 1024)  # 转换为MB
        msg.net_sent = net_io_counters.bytes_sent / (1024 * 1024)  # 转换为MB
        msg.net_recv = net_io_counters.bytes_recv / (1024 * 1024)  # 转换为MB

        self.status_publisher_.publish(msg)
        self.get_logger().info(f'Published System Status: {msg}')
        
def main():
    rclpy.init()
    node = SysStatusPub('sys_status_pub')
    rclpy.spin(node)
    rclpy.shutdown()