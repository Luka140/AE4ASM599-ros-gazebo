## Launch file
This package only contains a launch file that launches nodes from
- **pointcloud_to_laserscan**
- **pointcloud_to_occupancy_grid**

This launches a full pipeline that listens to a lidar LaserScan topic and publishes a dynamically growing gridmap.
To launch the pipeline use:
```
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lidar2map lidar2map.launch.py
```
