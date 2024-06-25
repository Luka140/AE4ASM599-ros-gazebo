
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

## Potential issues
This package uses the [pointcloud_to_laserscan](https://github.com/ros-perception/pointcloud_to_laserscan/tree/59bf996fb3ee7db0026a5cd3ce0d2a39d2e602ea) package as a submodule. When cloning or pulling this repo, the submodule may not be loaded. If the `pointcloud_to_laserscan` directory is empty, use the following command to clone the subdirectory into it:
```
`git submodule update --init --recursive`
```
Then rebuild this package, source `install/setup.bash` and retry to launch the package. 

## Overview
An overview of the nodes launched by this package is shown below:

![lidar2map_nb drawio(1)](https://github.com/Luka140/AE4ASM599-ros-gazebo/assets/92033464/a4afee9b-1cf2-4acf-890f-7b1d214c5273)
