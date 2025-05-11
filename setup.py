from setuptools import setup

package_name = 'virtual_sensor_stack'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YourName',
    maintainer_email='your@email.com',
    description='ROS2 package for virtual camera and IMU simulation.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'virtual_camera_node = virtual_sensor_stack.virtual_camera_node:main',
            'virtual_imu_node = virtual_sensor_stack.virtual_imu_node:main',
            'rviz2_visualizer = virtual_sensor_stack.rviz2_visualizer:main',
        ],
    },
)
