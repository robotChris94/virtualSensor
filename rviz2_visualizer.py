import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class ImuVisualizer(Node):
    def __init__(self):
        super().__init__('imu_visualizer_node')
        self.subscriber = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

    def imu_callback(self, msg):
        # acceleration data
        acc = msg.linear_acceleration

        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "imu"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # arrow shaft
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # Arrow direction from origin (0,0,0) to acc vector
        marker.points.append(Point(x=0.0, y=0.0, z=0.0))
        marker.points.append(Point(x=acc.x, y=acc.y, z=acc.z))

        self.marker_pub.publish(marker)
        self.get_logger().info(f'Visualizing IMU acc: x={acc.x:.2f}, y={acc.y:.2f}, z={acc.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = ImuVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
