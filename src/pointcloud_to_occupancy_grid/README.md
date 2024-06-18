This ROS2 package provides a node for converting PointCloud2 messages into an occupancy grid. The node subscribes to a PointCloud2 topic, transforms the point cloud to the world frame, and populates an occupancy grid which it publishes.

## Features
- Subscribes to a PointCloud2 topic.
- Transforms the points to the world frame using TF2.
- Creates and publishes an occupancy grid.
- Dynamically expands the occupancy grid when points exceed current boundaries.
- Filters out points beyond the lidar range limit.

## Parameters
All the parameters can be set in a launch file using a string input
- qos_best_effort:  ('qos_best_effort', 'True')     Adjust QoS reliability policy to best effort.
- world_frame_id:   (world_frame_id, f'{frame_id}') The frame ID of the world coordinate system.
- grid_topic:       ('grid_topic', 'topic')         The topic name for the occupancy grid.
- cloud_in_topic:   ('cloud_topic', 'topic')        The topic name for the input PointCloud2 messages.
- expansion_size:   ('expansion_size', f'{size}')   The expansion size for the grid when points go out of bounds in meters.
- lidar_range_lim:  ('range', f'{range}')           The range limit of the lidar in meters.