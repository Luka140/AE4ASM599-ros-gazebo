# image2map
This package was written based on nodes from the **environment_reconstruction** package. The package is a pipeline that takes images from a stereo camera setup and uses it to produce and update a grid map. A flow chart overview of the package is shown at the bottom of this page. 

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
  - This determines the rate at which the map update calls will be requested. Setting the value too low may cause a deadlock which prevents the map from updating. 
- camera_l_topic: Topic name for the left camera image. Default is /camera_l.
- camera_r_topic: Topic name for the right camera image. Default is /camera_r.
- camera_info_topic: Topic name for the camera info. Default is /camera_info.
- reconstruct_service: Service name for 3D reconstruction. Default is reconstruct_3d_view.
- map_service: Service name for occupancy map creation. Default is create_occupancy_map.
- filtered_reconstruction_topic: Topic name for filtered point clouds. Default is filtered_reconstruction.
- occupancy_map_topic: Topic name for the occupancy map. Default is occupancy_map.

#### Subscriptions

- Left Camera Image Subscription
  
        Topic: Defined by the camera_l_topic parameter, e.g., /camera_l
        Message Type: sensor_msgs.msg.Image
        Callback: update_img_l

- Right Camera Image Subscription
  
        Topic: Defined by the camera_r_topic parameter, e.g., /camera_r
        Message Type: sensor_msgs.msg.Image
        Callback: update_img_r

- Camera Info Subscription
  
        Topic: Defined by the camera_info_topic parameter, e.g., /camera_info
        Message Type: sensor_msgs.msg.CameraInfo
        Callback: update_camera_info

#### Services

- Reconstruct Image Service Client
        
        Service Name: set by the 'reconstruct_service' parameter (dafault is 'reconstruct_3d_view').
        Type: interfaces.srv.ReconstructImage
        Function: Invokes the reconstruction service to generate a 3D point cloud.
        Callback: recon_done_callback handles the response and publishes the filtered point cloud.

- Create Occupancy Map Service Client
        
        Service Name: set by the 'map_service' parameter (default is 'create_occupancy_map').
        Type: interfaces.srv.CreateOccupancyMap
        Function: Invokes the service to update an occupancy grid map based on the reconstructed point cloud.
        Callback: map_done_callback handles the response and publishes the updated occupancy grid.

#### Publishers

- Filtered Reconstruction Publisher
  
        Topic: Defined by the filtered_reconstruction_topic parameter, e.g., filtered_reconstruction
        Message Type: sensor_msgs.msg.PointCloud2
        Function: Publishes the filtered point cloud received from the reconstruction service.

- Occupancy Map Publisher
  
        Topic: Defined by the occupancy_map_topic parameter, e.g., occupancy_map
        Message Type: nav_msgs.msg.OccupancyGrid
        Function: Publishes the updated occupancy grid map generated from the reconstructed data.

#### TF2 Usage

- TF2 Buffer and Listener: Manages transforms using tf2_ros.Buffer and tf2_ros.TransformListener to obtain the current pose relative to the world frame (world_frame parameter).

#### Timer Callback

- Timer Callback: Periodically triggers the reconstruction pipeline:
  
        Collects the latest images and camera info.
        Obtains the current pose using TF2 transforms.
        Invokes the reconstruction service (reconstruction_client).
        Publishes the filtered point cloud and requests an occupancy map update using the respective services.
        
----

### Reconstructor
File: `stereo_reconstruction.py`

The Reconstructor node is a ROS2 node designed for reconstructing 3D point clouds from stereo images using disparity mapping. It utilizes the ROS2 framework along with OpenCV and Open3D libraries to process stereo image pairs and generate a 3D representation of the environment.

#### Parameters

The node declares and uses the following parameters:

- depth_lim: Maximum depth limit for camera reconstruction in meters. Default is 30.
- height_lim: Maximum height threshold in meters for filtering out points. Default is 1.5.
  - Any points above this height limit are ignored in the occupancy map, with the assumption that the vehicle can drive underneath it.
- voxel_downsample_size: Voxel size for downsampling the point cloud in meters. Default is 0.1.
- service_name: Name of the ROS service for reconstructing 3D views. Default is reconstruct_3d_view.

#### Services
- ReconstructImage Service Server
  
        Service Name: Determined by the parameter service_name (default is 'reconstruct_3d_view').
        Message Type: interfaces.srv.ReconstructImage
        Function: Handles requests to reconstruct a 3D point cloud from stereo images.
        Callback: reconstruct_view

----
### GridMapper
File: `grid_map.py`

The GridMapper node is a ROS2 node designed to generate and maintain an occupancy grid map based on incoming point cloud data and the position of a vehicle. This node uses the ROS2 framework and various libraries to process and visualize the occupancy grid. The methodology for creating the grid map is illustrated by the diagram below. 
- The yellow cone is the field of view of the cameras
- The red circles are point cloud points
- The green polygon is then drawn by linking adjacent point cloud points together and projecting them onto the back of the view cone. This polygon then encloses all the space that **should** be observed by the camera and excludes the grey dashed area that should be obstructed by obstacles.
- The orange triangles indicate the space between the vehicles and the point cloud points. This is the area that is **very likely** to have been observed. 

Any grid square inside the green polygon is then marked as potentially seen (30). Any grid square inside the orange triangles is marked as observed and empty (1). Any grid square with a point cloud point inside it is marked as an obstacle (100). 

This is a somewhat unconventional approach. The more obvious solution would be to trace the ray between a point and the position and set every square that the ray intersects with empty. This approach was not taken because it requires a while loop inside a loop over each point cloud point. This may be quite slow in Python. The polygon approach was an attempt to prevent doing this based on Python but rather based on NumPy code. However, it still requires a loop over the point cloud points, and may not scale that well with grid map size. 


![view_cone_v2](https://github.com/Luka140/AE4ASM599-ros-gazebo/assets/92033464/af7adaa1-2a7d-468f-b79a-9cc930d2c055)



#### Parameters

The node declares and uses the following parameters:

  - world_frame: The reference frame for the world. Default is world.
  - horizontal_fov: The horizontal field of view of the camera, in radians. Default is π/3.
  - camera_depth_lim: The maximum depth for camera data reconstruction in meters. Default is 30.0.
    - This is used to determine the 'view cone' of the camera.
  - point_distance_tolerance: Tolerance for point distance when evaluating the proximity of a point to the camera. Default is 1.2.
    - This is used in the algorithm that evaluates what squares on the grid map have been observed. 
  - voxel_connection_tolerance: Tolerance for voxel connection. Default is 1.95.
    - This is used in the algorithm that evaluates what squares on the grid map have been observed. 
    - Increasing this will prevent 'observed' spikes in between point cloud points.
  - angular_tolerance: Tolerance for angular position proximity. Default is 0.005.
    - This is used in the algorithm that evaluates what squares on the grid map have been observed
    - Increasing this will prevent 'observed' spikes in between point cloud points.

#### Services

- CreateOccupancyMap Service Server
  
        Service Name: 'create_occupancy_map'
        Type: interfaces.srv.CreateOccupancyMap
        Function: Handles requests to update an occupancy grid map based on point cloud data.
        Callback: update_map
        Purpose: Accepts requests containing a pose and a point cloud, processes the point cloud data to update an occupancy grid map, and returns the updated map in the response.
----

### Locator
File: `positioning.py`

The locator node listens to bridged pose messages from Gazebo and then broadcasts them as TF2 transforms. This is simply a 'hack' to get the ground truth position from Gazebo.

#### Parameters

The node uses the following parameter:

- vehicle_path: The ROS parameter that specifies the path to the vehicle model. Default is /model/Test_car.

#### Subscriptions

- Ground Truth Position Subscription
  
        Topic: Specified by the vehicle_name parameter, formatted as /model/{vehicle_name}/pose.
        Message Type: tf2_msgs.msg.TFMessage
        Callback: broadcast_transform

#### Publishers

- Transform Broadcaster
  
        Type: tf2_ros.TransformBroadcaster

- Static Transform Broadcaster
  
        Type: tf2_ros.StaticTransformBroadcaster

----
## Launching the nodes
The parameters described above can be set in the launch file `image2map.launch.py`. The nodes can be launched using the following command:

```bash
source opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch image2map image2map.launch.py
```

To test this package, open the following gazebo file:
```bash 
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$HOME/ros2_ws/gazebo_files/
ign gazebo worlds/tugbot_depot.sdf
```
Alongside this, open the RViz configuration in `ros2_ws/rviz_configs`.

----
## Overview
Below is an overview of how the nodes in this package communicate.
![image2map drawio(2)](https://github.com/Luka140/AE4ASM599-ros-gazebo/assets/92033464/ddbb3cc1-ef14-4dd9-9c69-9364ebcca554)


