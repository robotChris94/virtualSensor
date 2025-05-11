import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import time

class TFBroadcasterNode(Node):
    def __init__(self):
        super().__init__('tf_broadcaster_node')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.start_time = time.time()

    def timer_callback(self):
        t = time.time() - self.start_time
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'base_link'

        # Simulate position: 
        # x moves slowly forward, 
        # y swings in a sinusoidal manner
        transform.transform.translation.x = 0.1 * t
        transform.transform.translation.y = math.sin(t)
        transform.transform.translation.z = 0.0

        # Simulated angle: Rotate around the z axis
        transform.transform.rotation.w = math.cos(t/2)
        transform.transform.rotation.z = math.sin(t/2)

        self.br.sendTransform(transform)
        self.get_logger().info(f'Published TF at t={t:.2f}s')

def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcasterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
