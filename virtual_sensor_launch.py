from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='virtual_sensors',
            executable='virtual_camera_node',
            name='virtual_camera_node',
            output='screen'
        ),
        Node(
            package='virtual_sensors',
            executable='virtual_imu_node',
            name='virtual_imu_node',
            output='screen'
        ),
    ])