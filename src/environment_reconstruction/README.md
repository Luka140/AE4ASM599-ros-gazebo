# Environment Reconstruction
This package contains various nodes to reconstruct a point cloud from a stereo image setup and manipulate it. 
It is not a streamlined product but rather the result of experimentation for the sake of learning. The **image2map** package
shows a more direct pipeline, based on some of the nodes in this package. 


## Nodes

### Cluster Client

File: `cluster_client.py`

This node acts as a client for point cloud clustering and filtering services. It listens to specific topics for triggers to call these services and publishes the processed point clouds.

#### Parameters:
- cluster_service: Service name for clustering.
- filter_service: Service name for filtering.
- cluster_trigger_topic: Topic name for cluster trigger.
- filter_trigger_topic: Topic name for filter trigger.
- pointcloud_topic: Topic name for incoming point cloud.
- clustered_pcl_topic: Topic name for publishing clustered point cloud.
- filtered_pcl_topic: Topic name for publishing filtered point cloud.

### Clustering

File: `clustering.py`

This node provides a service for clustering a point cloud using the DBSCAN algorithm. It receives a point cloud, processes it, and returns the clustered point cloud.

#### Parameters:
- cluster_service: Service name for clustering.

### Pointcloud Filtering

File: `filter_pcl.py`

This node provides a service for filtering a point cloud using statistical outlier removal and voxel down-sampling.

#### Parameters:
- filter_service: Service name for filtering.

### Keyboard Capture

File: `keyboard_capture.py`

This node listens for keyboard inputs to trigger different actions such as reconstruction, clustering, filtering, and stopping the vehicle.

#### Parameters:
- keypress_topic: Topic name for keyboard keypress.
- reconstruct_service: Service name for reconstruction.
- cluster_trigger_topic: Topic name for cluster trigger.
- filter_trigger_topic: Topic name for filter trigger.
- cmd_vel_topic: Topic name for vehicle command velocity.

### Pointcloud Aggregation

File: `pointcloud_aggregation.py`

This node aggregates multiple point clouds into a single point cloud. It listens to a topic for incoming point clouds, transforms them to the global frame, and publishes the aggregated point cloud.

#### Parameters:
- global_frame_id: The global frame to transform pointclouds into.
- pointcloud_topic: Topic name for incoming pointclouds.
- pointcloud_publish_topic: Topic name for the aggregated pointcloud.

### Positioning
File: `positioning.py`

The locator node listens to bridged pose messages from Gazebo and then broadcasts them as TF2 transforms. This is simply a 'hack' to get the ground truth position from Gazebo.

#### Parameters

The node uses the following parameter:

- vehicle_path: The ROS parameter that specifies the path to the vehicle model. Default is /model/Test_car.

#### Subscriptions
- <vehicle_path>/pose (tf2_msgs/TFMessage): Subscribes to the ground truth pose topic of the specified vehicle model. The actual topic name is formed by concatenating the vehicle_path parameter with /pose.


#### How it works
The node listens to the pointclouds published by the Stereo Reconstruction node. It then finds the TF2 transform from the world frame to the relative frame the point cloud is in. The points are then set to the world frame using this transform and added to the collection of previously transformed points. All these points are colleced into a large point cloud which is then published. 


### Stereo Reconstruction

File: `stereo_reconstruction.py`

The Reconstructor node is designed to create 3D reconstructions from stereo camera images. It subscribes to image and camera info topics, processes the images to generate a disparity map, and computes the corresponding 3D point cloud. This point cloud is then published for further use by other nodes. 

#### Parameters

The node allows configuration through several parameters:

- distance_limit: Maximum distance to consider for points in the point cloud (default: 30.0).
- left_camera_topic: Topic for the left camera images (default: /camera_l).
- right_camera_topic: Topic for the right camera images (default: /camera_r).
- camera_info_topic: Topic for the camera info (default: /camera_info).
- published_topic: Topic to publish the reconstructed point cloud (default: reconstruction).
- recon_service_tag: Service tag for the reconstruction service (default: reconstruct_3d_view).

#### Subscriptions

- Left Camera Images: Subscribed to the topic specified by left_camera_topic.
- Right Camera Images: Subscribed to the topic specified by right_camera_topic.
- Camera Info: Subscribed to the topic specified by camera_info_topic.

#### Publisher

- Reconstructed Point Cloud: Publishes the reconstructed 3D point cloud to the topic specified by published_topic.

#### Service

- Reconstruct 3D View: A service that triggers the reconstruction of the 3D view from the stereo images. The service tag is specified by the recon_service_tag parameter.

#### How It Works

- Image Reception: The node listens to the left and right camera image topics and updates the stored images.
- Camera Info Reception: The node listens to the camera info topic and updates the camera calibration data.
- Disparity Map Computation: Using OpenCV, the node computes a disparity map from the stereo images.
- 3D Point Cloud Computation: The node calculates the real-world coordinates from the disparity map and camera calibration data.
- Point Cloud Publishing: The reconstructed 3D point cloud is published to the specified topic.
- Reconstruction Service: The service can be called to perform the reconstruction process and obtain the 3D point cloud.


## Other files
### `utils.py`
This Python module provides utility functions for working with ROS2 PointCloud2 messages, specifically for creating unstructured pointclouds from numpy arrays and converting PointCloud2 messages back to numpy arrays.
Features
- create_unstructured_pointcloud: Converts a numpy array of points to a ROS2 PointCloud2 message.
- pcl2array: Converts a ROS2 PointCloud2 message to a numpy array.


#### Function Descriptions
`create_unstructured_pointcloud(points: np.ndarray, frame_id='world', time=None) -> PointCloud2`

Creates an unstructured pointcloud from a numpy array of points.

Parameters:
- points: A numpy array of shape [N, 3] or [N, 4] containing the points. If the array has shape [N, 3], it should contain [x, y, z] coordinates. If it has shape [N, 4], it should contain [x, y, z, c] where c is the cluster label.
- frame_id: The frame id of the pointcloud. Defaults to 'world'.
- time: ROS2 time at which the pointcloud is created. If None, the current time is used.

Returns:
- PointCloud2: The created PointCloud2 message.

`pcl2array(pcl_msg: PointCloud2, flatten=False) -> np.ndarray`

Converts a PointCloud2 message to a numpy array.

Parameters:
-pcl_msg: The PointCloud2 message to be converted.
- flatten: If True, returns an array in the unstructured format, regardless of the pointcloud.

Returns:
- np.ndarray: The converted numpy array.


# Overview 
Note that this nodal architecture is not optimal. It is more complex than necessary and not always consistent. This is a result of the fact that this package is simply the result of experimentation without a specific goal. 
![environment_reconstruction drawio(2)](https://github.com/Luka140/AE4ASM599-ros-gazebo/assets/92033464/b1c7ff24-f9ad-4bb6-a0df-cd670b1b43ab)


