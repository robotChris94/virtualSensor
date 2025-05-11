import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import time

class VirtualImuNode(Node):
    def __init__(self):
        super().__init__('virtual_imu_node')
        self.publisher = self.create_publisher(Imu, 'imu/data', 10)
        self.start_time = time.time()
        self.timer = self.create_timer(0.5, self.publish_imu)

    def publish_imu(self):
        t = time.time() - self.start_time
        msg = Imu()
        msg.linear_acceleration.x = math.sin(t)
        msg.angular_velocity.z = math.cos(t)
        self.publisher.publish(msg)
        self.get_logger().info(f'Published IMU at t={t:.2f}s')

def main(args=None):
    rclpy.init(args=args)
    node = VirtualImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
