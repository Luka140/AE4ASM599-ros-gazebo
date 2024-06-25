# ROS2 and Gazebo
This project documents how to use Ros2 Humble in conjunction with Gazebo Ignition.

## Stetting up the Development Environment.
A standardised development environment is achieved through the use of devcontainers with vscode. For ubuntu based systems, the following system dependencies are required.
* **vscode**, setup for [dev containers](https://code.visualstudio.com/docs/devcontainers/containers)
* [**docker engine**](https://docs.docker.com/engine/install/ubuntu/)

Clone the git repositry and open it in vscode, a prompt will appear asking to open the project in the dev container. Alternatively, The project can be opened in the dev container by running ```Dev Containers: rebuild and reopen in container``` in the vscode command palette.

## image2map
The image2map package contains nodes that generate an occupancygrid based on images from a stereo camera setup. Further documentation is available in the README.md inside the package. 
To launch the nodes use:
```
source opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch image2map image2map.launch.py
```
Launch the simulation using
```
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$HOME/ros2_ws/gazebo_files/
ign gazebo worlds/tugbot_depot.sdf
```
The vehicle can be driven around using the arrow keys. Pressing "S" stops the vehicle. For this to work, the 'key publisher' in Gazebo needs to be enabled. 
To see the published gridmaps and pointclouds can be seen in RViz. An RViz configuration file can be found in `ros2_ws/rviz_configs`.


## 3D Reconstruction
The environment_reconstruction package contains code that takes in two images from a stereo camera and reconstructs the 3D environment as a point cloud. 

In one window, start up the reconstruction node using
```
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch environment_reconstruction launch_3d_reconstruction.launch.py
```

Open the simulation in another window using
```
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$HOME/ros2_ws/gazebo_files/
ign gazebo worlds/tugbot_depot.sdf
```

In a new window open RViz2:
```
source /opt/ros/humble/setup.bash
rviz2 
```
In this window go to 'file' -> 'open config' and open '3d_reconstruction_test.rviz'


The vehicle can be driven around using the arrow keys. Pressing "S" stops the vehicle. For this to work, the 'key publisher' in Gazebo needs to be enabled. 

- To request a 3D reconstruction press "Enter"
- To request clustering press "C"
- To request filtering press "F"


The point cloud and images should show up in the RViz2 window. 


## Pathfollowing
This contains a node that performs simple waypoint-following missions while avoiding obstacles. The algorithm is not robust and the waypoints may not be accurate to the simulated world due to error buildup in odometry. 

To use it open the gazebo file using:
```
ign gazebo gazebo_files/worlds/pathfollowing.sdf
```

In a new window open the node:
```
colcon build --packages-select pathfollowing --merge-install --symlink-install
```

```
source install/setup.bash
ros2 launch pathfollowing launch_pathfinding.launch.py
```

Then in a third window, a waypoint can be set using the following command:
```
ros2 topic pub --once waypoint geometry_msgs/msg/Pose "{position: {x: 10, y: 0.0, z: 0.0}}"
```



