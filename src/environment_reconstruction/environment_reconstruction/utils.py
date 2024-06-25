import rclpy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


def create_unstructured_pointcloud(points: np.ndarray, frame_id='world', time=None) -> PointCloud2:
    """
    Creates an unstructured pointcloud from a numpy array of points.
    
    Args:
        points: [N, 3/4] numpy array containing the points. 
                If [N, 3], this should be [x, y, z]. If [N, 4], this is [x, y, z, c] where c is the cluster label.
        frame_id: The frame id of the pointcloud.
        time: ROS2 time at which the pointcloud is created. If None, current time is used.
    
    Returns:
        PointCloud2: The created PointCloud2 message.
    """
    
    pointcloud = PointCloud2()

    # Use the current time if no time is provided
    if time is None:
        time = rclpy.node.Node().get_clock().now().to_msg()
    
    pointcloud.header.stamp = time
    pointcloud.header.frame_id = frame_id

    # Define the base dimensions of the pointcloud
    base_dims = ['x', 'y', 'z']
    bytes_per_point = 4  # Number of bytes per point field (float32)

    # Create PointField for each dimension
    fields = [PointField(name=direction, offset=i * bytes_per_point, datatype=PointField.FLOAT32, count=1) for i, direction in enumerate(base_dims)]
    
    # If points have a cluster label, add an additional field
    if points.shape[1] == 4:
        # Note: The input datatype is np.int8 but using FLOAT32 for compatibility
        fields.append(PointField(name='c', offset=3 * bytes_per_point, datatype=PointField.FLOAT32, count=1))

    pointcloud.fields = fields
    pointcloud.point_step = len(fields) * bytes_per_point
    total_points = points.shape[0]
    pointcloud.is_dense = True
    pointcloud.height = 1  # Unstructured pointcloud has height 1
    pointcloud.width = total_points
    pointcloud.row_step = total_points * pointcloud.point_step
    
    pointcloud.is_bigendian = False
    pointcloud.data = points.flatten().tobytes()

    return pointcloud


def pcl2array(pcl_msg: PointCloud2, flatten=False) -> np.ndarray:
    """
    Converts a PointCloud2 message to a numpy array.
    
    Args:
        pcl_msg: The PointCloud2 message to be converted.
        flatten: If True, returns an array in the unstructured format, regardless of the pointcloud.
    
    Returns:
        np.ndarray: The converted numpy array.
    """
    field_count = len(pcl_msg.fields)
    
    if pcl_msg.height == 1 or flatten:
        # If the pointcloud is unstructured or flatten is True
        array = np.frombuffer(pcl_msg.data, dtype=np.float32).reshape(-1, field_count)
    else:
        # If the pointcloud is structured
        array = np.frombuffer(pcl_msg.data, dtype=np.float32).reshape(pcl_msg.height, pcl_msg.width, field_count)
    
    return array
