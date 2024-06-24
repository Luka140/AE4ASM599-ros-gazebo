
This package only contains a launch file that launches nodes from:
- **pointcloud_to_laserscan**
- **pointcloud_to_occupancy_grid**
This launches a full pipeline that listens to a lidar LaserScan topic and publishes a dynamically growing grid map.

## Launch file
To launch the pipeline use:
```
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lidar2map lidar2map.launch.py
```

## Overview
An overview of the nodes launched by this package is shown below:

![lidar2map_nb drawio(1)](https://github.com/Luka140/AE4ASM599-ros-gazebo/assets/92033464/a4afee9b-1cf2-4acf-890f-7b1d214c5273)
