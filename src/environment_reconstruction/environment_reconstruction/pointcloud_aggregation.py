import rclpy 
import rclpy.node as node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from tf2_ros import TransformListener, LookupException, ExtrapolationException
from tf2_ros.buffer import Buffer
import open3d as o3d


class PointcloudAggregator(node.Node):
    
    def __init__(self):
        super().__init__('pointcloud_aggregator')
        """
        This node listens for pointclouds to be published to the 'reconstruction' topic, transforms them to 
        world coordinates and adds them to an aggregated pointcloud which is published to the 'total_pointcloud' topic
        """
        
        self.global_frame_id = 'world_demo'
        self.cluster = False

        # This is where the total pointcloud is stored
        # Initially 0 points
        self.pointcloud = o3d.geometry.PointCloud()

        # Listens to the reconstruction node. These pointclouds are with respect to the camera reference frame.
        # To add them to the total pointcloud, they first have to be transformed to the world coordinate frame. 
        # Once this publishes a new reconstruction, self.reconstruction_callback() is called to perform the 
        # coordinate transformations and add it to the total pointcloud.
        self.reconstruction_sub = self.create_subscription(PointCloud2,
                                                           'reconstruction', 
                                                           self.reconstruction_callback, 
                                                           10)
        
        # The topic to which the aggregated pointcloud is published
        self.pointcloud_pub = self.create_publisher(PointCloud2, 'total_pointcloud', 10)

        # Track coordinate transformations
        self.tf_buffer = Buffer(cache_time=rclpy.time.Time(seconds=20))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("Pointcloud aggregator node created")


    def reconstruction_callback(self, msg):     
        self.get_logger().info("Received a pointcloud message - adding to the total pointcloud")
        time = msg.header.stamp
        cam_frame = msg.header.frame_id
        target_frame = self.global_frame_id

        # Lookup the transformation at the time the pointcloud (picture) was created
        try:
            tf_trans = self.tf_buffer.lookup_transform(target_frame, cam_frame, time, timeout=rclpy.duration.Duration(seconds=3))
        except (LookupException, ExtrapolationException):
            self.get_logger().info("Could not lookup transform")
            return

        # Create a transformation matrix from the transformation message obtained from tf_trans
        transform = np.eye(4)
        rot = tf_trans.transform.rotation
        trans = tf_trans.transform.translation
        # If this is altered - be careful of the order of the quaternion (w,x,y,z) vs (x,y,z,w)
        # Some packages use one, other use the other and they often don't specify which is used.  
        transform[:3,:3] = _get_mat_from_quat(np.array([rot.w, rot.x, rot.y, rot.z]))
        transform[:3,3] = [trans.x, trans.y, trans.z]
        
        # Loads the pointcloud message into Open3D format to allows for transformation
        pcl_o3d = o3d.geometry.PointCloud()
        loaded_array = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 3)
        pcl_o3d.points = o3d.utility.Vector3dVector(loaded_array)
        
        # Remove all NaN points (points for which the depth could not be computed) and transform them
        pcl_o3d.remove_non_finite_points()
        pcl_o3d, _ = pcl_o3d.remove_statistical_outlier(
            nb_neighbors=20,
            std_ratio=1.0
        )
        pcl_o3d.transform(transform)

        # Add to the total pointcloud 
        self.pointcloud = self.pointcloud + pcl_o3d 

        labels = None
        if self.cluster:
            labels = self.pointcloud.cluster_dbscan(eps=0.05, min_points=10)

        pcl2_msg = self.create_pcl_msg(labels)
        self.pointcloud_pub.publish(pcl2_msg)
        self.get_logger().info(f"Added to the pointcloud")


    def create_pcl_msg(self, labels):
        pointcloud = PointCloud2()
        pointcloud.header.frame_id = self.global_frame_id

        datapoints = np.asarray(self.pointcloud.points, dtype=np.float32)

        if self.cluster and labels is not None:
            dims = ['x', 'y', 'z', 'c']
            lbls = np.expand_dims(np.asarray(labels, dtype=np.int8), 1)
            datapoints = np.hstack((datapoints, lbls))
        else:
            dims = ['x', 'y', 'z']

        bytes_per_point = 4
        fields = [PointField(name=direction, offset=i * bytes_per_point, datatype=PointField.FLOAT32, count=1) for i, direction in enumerate(dims)]
        pointcloud.fields = fields
        pointcloud.point_step = len(fields) * bytes_per_point
        total_points = datapoints.shape[0]
        pointcloud.is_dense = True
        pointcloud.height = 1
        pointcloud.width = total_points

        self.is_bigendian = False
        pointcloud.data = datapoints.flatten().tobytes()

        return pointcloud






def _get_mat_from_quat(quaternion: np.ndarray) -> np.ndarray:
    """
    TAKEN STRAIGHT FROM THE ROS2 REPO: 
    https://github.com/ros2/geometry2/blob/rolling/tf2_geometry_msgs/src/tf2_geometry_msgs/tf2_geometry_msgs.py
    simply importing it led to some issues because it is not in the setup file. 


    Convert a quaternion to a rotation matrix.

    This method is based on quat2mat from https://github.com
    f185e866ecccb66c545559bc9f2e19cb5025e0ab/transforms3d/quaternions.py#L101 ,
    since that library is not available via rosdep.

    :param quaternion: A numpy array containing the w, x, y, and z components of the quaternion
    :returns: The rotation matrix
    """
    Nq = np.sum(np.square(quaternion))
    if Nq < np.finfo(np.float64).eps:
        return np.eye(3)

    XYZ = quaternion[1:] * 2.0 / Nq
    wXYZ = XYZ * quaternion[0]
    xXYZ = XYZ * quaternion[1]
    yYZ = XYZ[1:] * quaternion[2]
    zZ = XYZ[2] * quaternion[3]

    return np.array(
        [[1.0-(yYZ[0]+zZ), xXYZ[1]-wXYZ[2], xXYZ[2]+wXYZ[1]],
        [xXYZ[1]+wXYZ[2], 1.0-(xXYZ[0]+zZ), yYZ[1]-wXYZ[0]],
        [xXYZ[2]-wXYZ[1], yYZ[1]+wXYZ[0], 1.0-(xXYZ[0]+yYZ[0])]])



def main(args=None):
    rclpy.init(args=args)
    
    aggregator = PointcloudAggregator()

    rclpy.spin(aggregator)

    aggregator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
