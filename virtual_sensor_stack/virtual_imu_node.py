import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import time

class VirtualImuNode(Node):
    def __init__(self):
        super().__init__('virtual_imu_node')
        self.publisher = self.create_publisher(Imu, '/imu/data', 10)
        self.start_time = time.time()
        self.timer = self.create_timer(0.5, self.publish_imu)

    def publish_imu(self):
        t = time.time() - self.start_time
        msg = Imu()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # Orientation (set to 0, but must include covariance)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = -1.0  # means "orientation not provided"

        # Angular velocity
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = math.cos(t)
        msg.angular_velocity_covariance[0] = 0.01  # non-zero covariance

        # Linear acceleration
        msg.linear_acceleration.x = math.sin(t)
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 0.0
        msg.linear_acceleration_covariance[0] = 0.01

        self.publisher.publish(msg)
        self.get_logger().info(f'Published IMU at t={t:.2f}s')

def main(args=None):
    rclpy.init(args=args)
    node = VirtualImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
