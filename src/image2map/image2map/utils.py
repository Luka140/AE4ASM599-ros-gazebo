import rclpy
import numpy as np 
from sensor_msgs.msg import PointCloud2, PointField



def create_unstructured_pointcloud(points:np.ndarray, frame_id='world', time=None) -> PointCloud2:
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


def pcl2array(pcl_msg: PointCloud2, flatten=False) -> np.ndarray:
    """
    Takes pointcloud2 message and returns a numpy array.

    If the pointcloud is unstructured, the returned size will be:
        [N, D] where N is the number of points, and D the number of dimensions.
        If the pointcloud only contains coordinates the columns are ['x', 'y', 'z']

    If the pointcloud is structured the returned array will be of format [width, height, D].
    If the argument 'flatten' is set to true, it will return an array in the unstructured format, 
    regardless of the pointcloud.
    """
    field_count = len(pcl_msg.fields)
    if pcl_msg.height == 1 or flatten:
        # If the pointcloud is unstructured
        array = np.frombuffer(pcl_msg.data, dtype=np.float32).reshape(-1, field_count)
    else:
        array = np.frombuffer(pcl_msg.data, dtype=np.float32).reshape(pcl_msg.height, pcl_msg.width, field_count)
    return array 


def euler_from_quaternion(x, y, z, w):
            """
            Convert a quaternion into euler angles (roll, pitch, yaw)
            roll is rotation around x in radians (counterclockwise)
            pitch is rotation around y in radians (counterclockwise)
            yaw is rotation around z in radians (counterclockwise)
            """
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = np.arctan2(t0, t1)
        
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = np.arcsin(t2)
        
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = np.arctan2(t3, t4)
        
            return roll_x, pitch_y, yaw_z # in radians