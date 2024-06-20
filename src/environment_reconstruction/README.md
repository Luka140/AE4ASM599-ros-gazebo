# Environment Reconstruction
This package contains various nodes to reconstruct a pointcloud from a stereo image setup and manipulate it. 
It is not a streamlined product but rather the result of experimentation for the sake of learning. The **image2map** package
shows a more direct pipeline, based on some of the nodes in this package. 

WORK IN PROGRESS - NODES MISSING AND UNFORMATTED

## Nodes

### Cluster Client

File: cluster_client.py

This node acts as a client for pointcloud clustering and filtering services. It listens to specific topics for triggers to call these services and publishes the processed pointclouds.

    Parameters:
        cluster_service: Service name for clustering.
        filter_service: Service name for filtering.
        cluster_trigger_topic: Topic name for cluster trigger.
        filter_trigger_topic: Topic name for filter trigger.
        pointcloud_topic: Topic name for incoming point cloud.
        clustered_pcl_topic: Topic name for clustered point cloud.
        filtered_pcl_topic: Topic name for filtered point cloud.

### Clustering

File: clustering.py

This node provides a service for clustering a pointcloud using the DBSCAN algorithm. It receives a pointcloud, processes it, and returns the clustered pointcloud.

    Parameters:
        cluster_service: Service name for clustering.

### Pointcloud Filtering

File: filter_pcl.py

This node provides a service for filtering a pointcloud using statistical outlier removal and voxel down-sampling.

    Parameters:
        filter_service: Service name for filtering.

### Keyboard Capture

File: keyboard_capture.py

This node listens for keyboard inputs to trigger different actions such as reconstruction, clustering, filtering, and stopping the vehicle.

    Parameters:
        keypress_topic: Topic name for keyboard keypress.
        reconstruct_service: Service name for reconstruction.
        cluster_trigger_topic: Topic name for cluster trigger.
        filter_trigger_topic: Topic name for filter trigger.
        cmd_vel_topic: Topic name for vehicle command velocity.

### Pointcloud Aggregation

File: pointcloud_aggregation.py

This node aggregates multiple pointclouds into a single pointcloud. It listens to a topic for incoming pointclouds, transforms them to the global frame, and publishes the aggregated pointcloud.

    Parameters:
        global_frame_id: The global frame to transform pointclouds into.
        pointcloud_topic: Topic name for incoming pointclouds.
        pointcloud_publish_topic: Topic name for the aggregated pointcloud.

### Positioning


### Stereo Reconstruction


## Other files
### utils.py 
Utility functions ...


# ADD BLOCK DIAGRAM