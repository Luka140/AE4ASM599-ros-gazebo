import rclpy
import numpy as np 
from sensor_msgs.msg import PointCloud2, PointField



def create_unstructured_pointcloud(points:np.ndarray, frame_id='world', time=None):
    """
    Creates an unstructured pointcloud from a numpy array of points
    inputs:
        points: [N,3/4] numpy array containing the points. 
                If [N,3] this should be [x y z], if [N,4] this is [x y z c] where c is the cluster label
        frame_id: the frame id of the pointcloud
    """
    
    pointcloud = PointCloud2()

    if time is None:
        time  = rclpy.node.Node().get_clock().now().to_msg()
    
    pointcloud.header.stamp = time
    pointcloud.header.frame_id = frame_id

    base_dims = ['x', 'y', 'z']
    bytes_per_point = 4
    fields = [PointField(name=direction, offset=i * bytes_per_point, datatype=PointField.FLOAT32, count=1) for i, direction in enumerate(base_dims)]
    
    if points.shape[1] == 4:
        # The input datatype is np.int8 but for some reason inserting that here breaks everything :)
        fields.append(PointField(name='c', offset=3 * bytes_per_point, datatype=PointField.FLOAT32, count=1))

    pointcloud.fields = fields
    pointcloud.point_step = len(fields) * bytes_per_point
    total_points = points.shape[0]  
    pointcloud.is_dense = True
    pointcloud.height = 1
    pointcloud.width = total_points
    pointcloud.row_step = total_points * pointcloud.point_step
    
    pointcloud.is_bigendian = False
    pointcloud.data = points.flatten().tobytes()

    return pointcloud


    