'''VIO_estimator_node, Optical Flow Track, IMU Buffer, Pose Estimator'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time

'''
Optical Flow:           Tracks the pixel displacement of feature points to estimate movement in (x, y)
IMU Z Angular Velocity: Integrates to obtain the rotation angle (theta)
Pose:                   Composes a PoseStamped message and publishes it to /estimated_pose
Path:                   Continuously stacks poses and publishes to /estimated_path (can be visualized in RViz)
'''


class VIOEstimatorNode(Node):
    def __init__(self):
        super().__init__('vio_estimator_node')

        # ROS2 pub & sub
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        self.pose_pub = self.create_publisher(PoseStamped, '/estimated_pose', 10)
        self.path_pub = self.create_publisher(Path, '/estimated_path', 10)

        self.bridge = CvBridge()
        self.prev_gray = None
        self.prev_pts = None
        self.last_time = time.time()

        self.current_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.path_msg = Path()
        self.path_msg.header.frame_id = "world"

        # IMU buffer
        self.last_imu_time = None
        self.last_angular_velocity_z = 0.0

    def imu_callback(self, msg):
        self.last_angular_velocity_z = msg.angular_velocity.z
        self.last_imu_time = self.get_clock().now().nanoseconds / 1e9

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            if self.prev_gray is None:
                self.prev_gray = gray
                self.prev_pts = cv2.goodFeaturesToTrack(gray, maxCorners=100, qualityLevel=0.3, minDistance=7)
                return

            # Computing Optical Flow
            next_pts, status, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, self.prev_pts, None)

            # valid tracking point
            good_prev = self.prev_pts[status == 1]
            good_next = next_pts[status == 1]

            if len(good_prev) >= 5:
                flow = good_next - good_prev
                # Pose Estimator
                dx = np.mean(flow[:, 0])
                dy = np.mean(flow[:, 1])

                scale = 0.005  # Unit conversion ratio (pixels-wise)
                self.current_pose[0] += dx * scale
                self.current_pose[1] += dy * scale

            # Integrate angular velocity to calculate angle via IMU
            now = time.time()
            dt = now - self.last_time
            self.last_time = now
            dtheta = self.last_angular_velocity_z * dt
            self.current_pose[2] += dtheta  # theta

            # PoseStamped
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "world"
            pose.pose.position.x = self.current_pose[0]
            pose.pose.position.y = self.current_pose[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = math.sin(self.current_pose[2] / 2.0)
            pose.pose.orientation.w = math.cos(self.current_pose[2] / 2.0)

            self.pose_pub.publish(pose)

            # Path
            self.path_msg.header.stamp = pose.header.stamp
            self.path_msg.poses.append(pose)
            self.path_pub.publish(self.path_msg)

            # Update previous frame data
            self.prev_gray = gray
            self.prev_pts = cv2.goodFeaturesToTrack(gray, maxCorners=100, qualityLevel=0.3, minDistance=7)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VIOEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
