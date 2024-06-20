# image2map
This package was written based on nodes from the **environment_reconstruction** package. The package is a pipeline that takes images from a stereo camera setup, and uses it to produce and update a gridmap. A flow chart overview of the package is shown at the bottom of this page.

It contains four nodes:
- Coordinator
- GridMapper
- Reconstructor
- Locator

## Nodes
### Coordinator
File: `timer.py`

The Coordinator node coordinates various tasks for 3D reconstruction, point cloud filtering, and occupancy map creation using stereo camera inputs.
It periodically collects images from stereo cameras, performs 3D reconstruction, filters the resulting point clouds, and updates the occupancy map, by sending service requests to the other nodes in the package.

#### Parameters

The node uses the following parameters:
- world_frame: The global frame ID. Default is world.
- baseline: The baseline distance between the stereo cameras. Default is 1.0.
- period: The period in seconds for the timer callback. Default is 3.5.
- camera_l_topic: Topic name for the left camera image. Default is /camera_l.
- camera_r_topic: Topic name for the right camera image. Default is /camera_r.
- camera_info_topic: Topic name for the camera info. Default is /camera_info.
- reconstruct_service: Service name for 3D reconstruction. Default is reconstruct_3d_view.
- map_service: Service name for occupancy map creation. Default is create_occupancy_map.
- filtered_reconstruction_topic: Topic name for filtered point clouds. Default is filtered_reconstruction.
- occupancy_map_topic: Topic name for the occupancy map. Default is occupancy_map.

#### Subscriptions

- /camera_l (sensor_msgs/Image): Subscription to the left camera image.
- /camera_r (sensor_msgs/Image): Subscription to the right camera image.
- /camera_info (sensor_msgs/CameraInfo): Subscription to the camera information.

#### Publishers

- filtered_reconstruction (sensor_msgs/PointCloud2): Publishes the filtered point cloud.
- occupancy_map (nav_msgs/OccupancyGrid): Publishes the occupancy map.

----

### Reconstructor

The Reconstructor node is a ROS2 node designed for reconstructing 3D point clouds from stereo images using disparity mapping. It utilizes the ROS2 framework along with OpenCV and Open3D libraries to process stereo image pairs and generate a 3D representation of the environment.

#### Parameters

The node declares and uses the following parameters:

- depth_lim: Maximum depth limit for camera reconstruction in meters. Default is 30.
- height_lim: Maximum height threshold in meters for filtering out points. Default is 1.5.
- voxel_downsample_size: Voxel size for downsampling the point cloud in meters. Default is 0.1.
- service_name: Name of the ROS service for reconstructing 3D views. Default is reconstruct_3d_view.

#### Services

- reconstruct_3d_view (interfaces/srv/ReconstructImage): Service for receiving stereo image pairs and reconstructing a 3D point cloud.

----
### GridMapper
The GridMapper node is a ROS2 node designed to generate and maintain an occupancy grid map based on incoming point cloud data and the position of a vehicle. This node uses the ROS2 framework and various libraries to process and visualize the occupancy grid.

#### Parameters

The node declares and uses the following parameters:

  - world_frame: The reference frame for the world. Default is world.
  - horizontal_fov: The horizontal field of view of the camera, in radians. Default is π/3.
  - camera_depth_lim: The maximum depth for camera data reconstruction in meters. Default is 30.0.
  - point_distance_tolerance: Tolerance for point distance when evaluating the proximity of a point to the camera. Default is 1.2.
    - This is used in the algorithm that evaluates what squares on the gridmap have been observed. 
  - voxel_connection_tolerance: Tolerance for voxel connection. Default is 1.95.
    - This is used in the algorithm that evaluates what squares on the gridmap have been observed. 
    - Increasing this will prevent 'observed' spikes in between pointcloud points.
  - angular_tolerance: Tolerance for angular position proximity. Default is 0.005.
    - This is used in the algorithm that evaluates what squares on the gridmap have been observed
    - Increasing this will prevent 'observed' spikes in between pointcloud points.

#### Services

- create_occupancy_map (interfaces/srv/CreateOccupancyMap): Service for creating and updating the occupancy grid based on incoming point cloud data. This is used to interact with **Coordinator**.

----

### Locator
File: `positioning.py`

The locator node listens to bridged pose messages from Gazebo, and then broadcasts them as TF2 transforms. This is simply a 'hack' to get the ground thruth position from Gazebo.

#### Parameters

The node uses the following parameter:

- vehicle_path: The ROS parameter that specifies the path to the vehicle model. Default is /model/Test_car.

#### Subscriptions
- <vehicle_path>/pose (tf2_msgs/TFMessage): Subscribes to the ground truth pose topic of the specified vehicle model. The actual topic name is formed by concatenating the vehicle_path parameter with /pose.

----
## Launching the nodes
The parameters described above can be set in the launch file `image2map.launch.py`. The nodes can be launched using the following command:

```bash
source opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch image2map image2map.launch.py
```

To test this package, open the following gazebo file:
```
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$HOME/ros2_ws/gazebo_files/
ign gazebo worlds/tugbot_depot.sdf
```
Alongside this, open the RViz configuration in `ros2_ws/rviz_configs`.

----
## Overview
Below is an overview of how the nodes in this package communicate.
![image2map drawio(2)](https://github.com/Luka140/AE4ASM599-ros-gazebo/assets/92033464/ddbb3cc1-ef14-4dd9-9c69-9364ebcca554)


