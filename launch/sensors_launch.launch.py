from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    use_imu = LaunchConfiguration('use_imu')

    return LaunchDescription([
        DeclareLaunchArgument('use_imu', default_value='true'),

        Node(
            package='virtual_sensor_stack',
            executable='virtual_camera_node',
            name='virtual_camera'
        ),

        Node(
            condition=IfCondition(use_imu),
            package='virtual_sensor_stack',
            executable='virtual_imu_node',
            name='virtual_imu'
        ),

        Node(
            condition=IfCondition(use_imu),
            package='virtual_sensor_stack',
            executable='rviz2_visualizer',
            name='imu_visualizer'
        ),

        Node(
            package='virtual_sensor_stack',
            executable='tf_broadcaster_node',
            name='static_tf_broadcaster'
        ),
        
        # VIO mode, after all neccessary nodes on
        Node(
            package='virtual_sensor_stack',
            executable='vio_estimator_node',
            name='vio_estimator',
            output='screen'
        ),
    ])
