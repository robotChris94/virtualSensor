# ROS2 Virtual Sensor Stack
This project aims to:
- Simulate real-time camera and IMU data publishing
- Enable RViz2-based sensor visualization without Gazebo or physical devices.
- Provide a clean ROS2 Python codebase for educational or extension purposes.

## Features
- Virtual Camera Node: Publishes .jpg images to /camera/image_raw
- Virtual IMU Node(Optional): Simulates sinusoidal acceleration and angular velocity to /imu/data
- RViz2 IMU Visualizer: Converts IMU vector data into RViz arrow markers
- Unified Launch File: Launches all three nodes together
- Modular Design: Each node is self-contained, readable, and easily replaceable

## File Structure

``` bash
ros2_virtual_sensor_stack/
├── virtual_sensor_stack/
│   ├── virtual_camera_node.py    # Publish image from disk every 0.5s
│   ├── virtual_imu_node.py       
│   ├── rviz2_visualizer.py       # Convert IMU data to arrow Markers in RViz
├── launch/
│   └── sensors_launch.py         # Launch all nodes together
├── setup.py                      # Declare Python entry points
├── package.xml                   # ROS2 dependencies
├── setup.cfg                     # config
├── README.md                    
├── resource/
│   └── ros2_virtual_sensor_stack # Required by colcon
└── test_image.jpg                # Sample image to simulate a camera frame
```

## Installation

```bash
sudo apt install ros-humble-cv-bridge ros-humble-image-transport
cd ~/ros2_ws/src
cd ..
colcon build
source install/setup.bash
```

## Run(in Ubuntu 22.04 + ROS2 Humble)
``` bash
# Active IMU(default) and carmera
- ros2 launch virtual_sensor_stack sensors_launch.py
# Inactive IMU, only camera active
- ros2 launch virtual_sensor_stack sensors_launch.py use_imu:=false
```


