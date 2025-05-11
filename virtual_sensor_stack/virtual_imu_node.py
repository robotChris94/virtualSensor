
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import time

class VirtualIMUNode(Node):
    def __init__(self):
        super().__init__('virtual_imu_node')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.start_time = time.time()
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Imu()
        t = time.time() - self.start_time
        msg.linear_acceleration.x = math.sin(t)
        msg.angular_velocity.z = math.cos(t)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published IMU data: t={t:.2f}')