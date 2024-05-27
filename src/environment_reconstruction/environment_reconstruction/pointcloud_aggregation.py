import rclpy 
import rclpy.node as node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Transform
import numpy as np
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer
import open3d as o3d
import ros2_numpy
import copy


class PointcloudAggregator(node.Node):
    
    def __init__(self):
        super().__init__('pointcloud_aggregator')
        self.get_logger().info("Pointcloud aggregator node created")
        
        self.pointcloud = None
        self.reconstruction_sub = self.create_subscription(PointCloud2,
                                                           'reconstruction', 
                                                           self.reconstruction_callback, 
                                                           10)
        
        self.pointcloud_pub = self.create_publisher(PointCloud2, 'total_pointcloud', 10)

        self.tf_buffer = Buffer(cache_time=rclpy.time.Time(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def reconstruction_callback(self, msg):
        self.get_logger().info("Received a pointcloud message - adding to the total pointcloud")
        time = msg.header.stamp
        cam_frame = msg.header.frame_id
        # cam_frame = 'Test_car/chassis'
        target_frame = "world_demo"
        # target_frame = "Test_car/chassis"

        # tf_trans = self.tf_buffer.lookup_transform(cam_frame, target_frame, time)
        tf_trans = self.tf_buffer.lookup_transform(target_frame, cam_frame, time)
        # self.get_logger().info(f"{tf_trans}")
        # self.get_logger().info(f"tf transform dir {tf_trans.transform.__doc__}")
        # transform = ros2_numpy.numpify(tf_trans.transform) # THIS FUNCTION DOESNT WORK CORRECTLY 
        transform = np.eye(4)
        rot = tf_trans.transform.rotation
        trans = tf_trans.transform.translation
        # transform[:3,:3] = o3d.geometry.get_rotation_matrix_from_quaternion(np.array([[rot.x],
        #                                                                               [rot.y],
        #                                                                               [rot.z],
        #                                                                               [rot.w]]))

        # transform[:3,:3] = o3d.geometry.get_rotation_matrix_from_quaternion(np.array([[rot.w],
        #                                                                               [rot.x],
        #                                                                               [rot.y],
        #                                                                               [rot.z]]))

        # roll, pitch, yaw = self.euler_from_quaternion(rot.x, rot.y, rot.z, rot.w)
        # transform[:3,:3] = o3d.geometry.get_rotation_matrix_from_xyz(np.array([roll, pitch, yaw]))
        transform[:3,:3] = _get_mat_from_quat(np.array([rot.w, rot.x, rot.y, rot.z]))
        transform[:3,3] = [trans.x, trans.y, trans.z]


        # msg_transformed = self.tf_listener.transformPointCloud('world', msg)
        # msg_transformed = do_transform_cloud(msg, transform)
        # msg_transformed = self.tf_buffer.transform(object_stamped=msg, 
                                                #    target_frame='world')
        
        self.get_logger().info(f"\n\n{transform}")
        # coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame()
        # o3d.visualization.draw_geometries([coord_frame, copy.deepcopy(coord_frame).transform(transform)])


        
        pcl_o3d = o3d.geometry.PointCloud()
        loaded_array = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 3)
        pcl_o3d.points = o3d.utility.Vector3dVector(loaded_array)
        
        # self.get_logger().info(f"nr of nans before: {np.isnan(np.asarray(pcl_o3d.points)).sum()}")
        pcl_o3d.remove_non_finite_points().transform(transform)
        # self.get_logger().info(f"nr of nans after: {np.isnan(np.asarray(pcl_o3d.points)).sum()}")

        # This transform is a bandaid to so RVIZ shows the result as I think it should be
        # This transform should not be necessary, the previous one should take care of everything
        # transform_mat2 = np.zeros((4, 4))
        # transform_mat2[:3,:3] = o3d.geometry.get_rotation_matrix_from_xyz(np.array([0, 0, np.pi]))
        # transform_mat2[3,3] = 1
        # The line below seems to indicate that the height of the transform previously is the wrong way around
        # It may have to do with gazebo automatically inserting the height of the camera into the extrinsic matrix?
        # transform_mat2[3,2] = -2*transform[3,2]
        # pcl_o3d.transform(transform_mat2)

        msg_transformed = msg # TODO This may cause aliasing issues, not sure yet
        msg_transformed.data = np.asarray(pcl_o3d.points, dtype=np.float32).flatten().tobytes()
        msg_transformed.row_step = len(msg_transformed.data)
        msg_transformed.width = msg_transformed.row_step // msg.point_step
        
        msg_transformed.height = 1
        msg_transformed.header.frame_id = target_frame
        if self.pointcloud is None:
            self.pointcloud = msg_transformed
        else:
            self.pointcloud.header.stamp = msg.header.stamp
            self.pointcloud.width += msg_transformed.width
            total_num_of_points = self.pointcloud.height * self.pointcloud.width
            self.pointcloud.row_step = self.pointcloud.point_step * total_num_of_points
            self.pointcloud.data.extend(msg_transformed.data)
            # self.get_logger().info(f"new pointcloud data shape {self.pointcloud.data.shape}")
            # self.pointcloud.data = new_data

        # self.get_logger().info(f"total pointcloud data shape {len(self.pointcloud.data)} {loaded_array.shape} {pcl_o3d.points}")
        self.pointcloud_pub.publish(self.pointcloud)
        self.get_logger().info("Added to the pointcloud")


    # def euler_from_quaternion(self, x, y, z, w):
    #     """
    #     Convert a quaternion into euler angles (roll, pitch, yaw)
    #     roll is rotation around x in radians (counterclockwise)
    #     pitch is rotation around y in radians (counterclockwise)
    #     yaw is rotation around z in radians (counterclockwise)
    #     """
    #     t0 = +2.0 * (w * x + y * z)
    #     t1 = +1.0 - 2.0 * (x * x + y * y)
    #     roll_x = np.arctan2(t0, t1)
    
    #     t2 = +2.0 * (w * y - z * x)
    #     t2 = +1.0 if t2 > +1.0 else t2
    #     t2 = -1.0 if t2 < -1.0 else t2
    #     pitch_y = np.arcsin(t2)
    
    #     t3 = +2.0 * (w * z + x * y)
    #     t4 = +1.0 - 2.0 * (y * y + z * z)
    #     yaw_z = np.arctan2(t3, t4)
    
    #     return roll_x, pitch_y, yaw_z # in radians
    

def _get_mat_from_quat(quaternion: np.ndarray) -> np.ndarray:
    """
    TAKEN STRAIGHT FROM: https://github.com/ros2/geometry2/blob/rolling/tf2_geometry_msgs/src/tf2_geometry_msgs/tf2_geometry_msgs.py
    
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
