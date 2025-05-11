# ROS2 Virtual Sensor Stack + Visual-Inertial Odometry (VIO) Toolkit
This project aims to:
- Simulate real-time camera and IMU data publishing
- Enable RViz2-based sensor visualization without Gazebo or physical devices.
- Provide a clean ROS2 Python codebase for educational or extension purposes.

## Features
- Simulate camera and IMU sensor publishing using ROS2 nodes
- Visualize data in RViz2 (IMU markers, pose, path, TF)
- Provide an educational foundation for VIO-based localization
- Support testing of perception/localization pipelines without hardware

## File Structure

```bash
ros2_virtual_sensor_stack/
├── virtual_sensor_stack/
│   ├── virtual_camera_node.py       # Camera simulator
│   ├── virtual_imu_node.py          # Sinusoidal IMU simulator
│   ├── rviz2_visualizer.py          # Marker visualizer for IMU
│   ├── tf_broadcaster_node.py       # TF simulation
│   ├── vio_estimator_node.py        # VIO estimator
│   ├── dummy_imu_data.json          # Dummy IMU metadata fallback (when IMU off)
├── launch/
│   └── sensors_launch.py            
├── rviz_config/
│   └── default_vio_view.rviz        # RViz preset
├── test_image.jpg                   
├── setup.py                         # config
├── package.xml
├── setup.cfg
├── README.md    
```

### VIO Estimator Node (20250511 Update)
- Estimates robot pose based on:
  - Optical flow (OpenCV LK tracker)
  - Angular velocity (IMU z-axis integration)
- Publishes:
  - `geometry_msgs/PoseStamped` → `/estimated_pose`
  - `nav_msgs/Path` → `/estimated_path`

## Installation
```bash
# Install dependencies
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
# RViz2 Visualization
- rviz2 -d rviz_config/default_vio_view.rviz
```


