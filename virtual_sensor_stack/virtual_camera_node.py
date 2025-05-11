import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import os
import json

class VirtualCameraNode(Node):
    def __init__(self):
        super().__init__('virtual_camera_node')

        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.meta_pub = self.create_publisher(String, 'camera/dummy_imu_meta', 10)

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()
        self.image_dir = os.path.join(os.path.dirname(__file__), 'test_image.jpg')

        # Get parameter of use_imu 
        self.declare_parameter('use_imu', True)
        self.use_dummy_imu = self.get_parameter('use_imu').get_parameter_value().bool_value

        # Inactivate IMU, fetch dummy metadata 
        if not self.use_dummy_imu:
            self.get_logger().info('Dummy IMU metadata will be published.')
            # Read metadata json
            data_path = os.path.join(os.path.dirname(__file__), 'dummy_imu_data.json')
            with open(data_path, 'r') as f:
                self.dummy_imu_list = json.load(f)
            self.imu_index = 0
        else:
            self.get_logger().info('IMU node will run separately. Dummy IMU disabled.')

    def timer_callback(self):
        image = cv2.imread(self.image_dir)
        if image is not None:
            msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            self.publisher.publish(msg)

        if not self.use_dummy_imu:
            imu_data = self.dummy_imu_list[self.imu_index]
            meta_msg = String()
            meta_msg.data = json.dumps(imu_data)
            self.meta_pub.publish(meta_msg)

            self.imu_index = (self.imu_index + 1) % len(self.dummy_imu_list)  

def main(args=None):
    rclpy.init(args=args)
    node = VirtualCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()