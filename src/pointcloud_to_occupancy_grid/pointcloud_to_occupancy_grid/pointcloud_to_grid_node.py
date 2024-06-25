import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformListener, LookupException, ExtrapolationException
from tf2_ros.buffer import Buffer
from rclpy.qos import DurabilityPolicy, qos_profile_system_default, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
import numpy as np

class PointcloudToGridNode(Node):
    def __init__(self):
        super().__init__("pointcloud_to_grid_node")

        self.grid = None

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters = [("qos_best_effort", ""),      
                          ("world_frame_id", "odom"),   
                          ("grid_topic", "/grid"),   
                          ("cloud_in_topic", "/cloud"), # PointCloud2 topic to listen to
                          ("expansion_size", 50.),      # When a point goes out of bounds, expand the map in this direction by this number of meters
                          ("lidar_range_lim", 30.)      # The range limit of the lidar in meters - used to filter out its endpoints that do not hit anything
            ]
        )

        # Get parameters
        self.qos_best_effort        = bool(self.get_parameter("qos_best_effort").get_parameter_value().string_value)
        self.world_frame_id         = self.get_parameter("world_frame_id").get_parameter_value().string_value
        self.expansion_size         = self.get_parameter("expansion_size").get_parameter_value().double_value
        self.cloud_topic            = self.get_parameter("cloud_in_topic").get_parameter_value().string_value 
        self.lidar_max_range        = self.get_parameter("lidar_range_lim").get_parameter_value().double_value
        self.grid_topic_name        = self.get_parameter("grid_topic").get_parameter_value().string_value

        # Initialize variables
        self.position = np.array([0.0, 0.0], dtype=float)

        # Initialize variables for occupancy grid creation
        self.resolution = .05   # Resolution of occupancy grid (meters per cell)
        self.x_width    = 75.0    # Width of occupancy grid (meters)
        self.y_width    = 75.0    # Height of occupancy grid (meters)
        self.grid       = None
        
        # Adjust QoS policy
        adjusted_policy = qos_profile_system_default
        if self.qos_best_effort:
            adjusted_policy.reliability = ReliabilityPolicy.BEST_EFFORT
        adjusted_policy.durability = DurabilityPolicy.VOLATILE

        self.get_logger().info(f"Adjusted policty to: {adjusted_policy}")

        # Set up Intensity base Grid Publisher
        self.publish_grid = self.create_publisher(
            OccupancyGrid, self.grid_topic_name, qos_profile=adjusted_policy
        )

        # Set up PointCloud2 Subscriber
        self.sub_pc2 = self.create_subscription(
            PointCloud2, self.cloud_topic, self.pointcloud_callback, qos_profile=adjusted_policy
        )

        # Initialize TF listener
        self.tf_buffer   = Buffer(cache_time=rclpy.time.Time(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)        

        self.get_logger().info(f"Pointcloud to grid node initiated, listening to the '{self.cloud_topic}' topic and publishing on '{self.grid_topic_name}'")

    def pointcloud_callback(self, msg: PointCloud2):
        # Try to get the transform from the cloud frame to the world frame
        try:
<<<<<<< HEAD
            tf_trans = self.tf_buffer.lookup_transform(self.world_frame_id, msg.header.frame_id, rclpy.time.Time(seconds=0), timeout=rclpy.duration.Duration(seconds=3))
        except (LookupException, ExtrapolationException) as error:
            self.get_logger().info(f"Could not lookup transform for time: {msg.header.stamp} \n {error}")
=======
            tf_time = msg.header.stamp
            # tf_trans = self.tf_buffer.lookup_transform(self.world_frame_id, msg.header.frame_id, rclpy.time.Time(seconds=0), timeout=rclpy.duration.Duration(seconds=3))
            tf_trans = self.tf_buffer.lookup_transform(self.world_frame_id, msg.header.frame_id, tf_time, timeout=rclpy.duration.Duration(seconds=5))
        except (LookupException, ExtrapolationException) as error:
            self.get_logger().info(f"Could not lookup transform for time: {tf_time} \n {error}")
>>>>>>> 0127beee20e4f4ff9ca6c5719f208bed7bf6b035
            return
        
        # Get the translation and rotation from the transform
        transl = tf_trans.transform.translation
        quat = tf_trans.transform.rotation
        
        if self.grid is None:
            # Initialize occupancy grid if it's not already initialized
            self.grid = OccupancyGrid()
            self.grid.header.frame_id           = self.world_frame_id
            self.grid.info.resolution           = self.resolution
            self.grid.info.width                = int(self.x_width/self.resolution)
            self.grid.info.height               = int(self.y_width/self.resolution)
            self.grid.info.origin.position.x    = transl.x - self.x_width / 2 
            self.grid.info.origin.position.y    = transl.y - self.y_width / 2

            # Initialize every cell as unknown
            self.grid.data = [-1] * self.grid.info.width * self.grid.info.height
        
        # Convert PointCloud2 message to numpy array
        points = pcl2array(msg)

        # Filter out endpoints of the lidar that do not hit anything
        points = points[np.where((points[:,0]**2 + points[:,1]**2) < 0.95 * (self.lidar_max_range / 2) ** 2)]
        
        # Transform points to global coordinates
        rotation_matrix = _get_mat_from_quat(np.array([quat.w, quat.x, quat.y, quat.z]))
        translation = np.array([transl.x, transl.y, transl.z])
        global_pcl = np.einsum("bi, ij -> bj", points[:,:3], rotation_matrix.T) + translation

        origin = np.array([self.grid.info.origin.position.x, self.grid.info.origin.position.y])
        pcl_wrt_origin = global_pcl[:,:2] - origin

        # Check if grid expansion is needed and expand if necessary
        if (pcl_wrt_origin[:,0] < 0).any():
            self.get_logger().info("\n\n Expanding the grid in  '-x' direction \n\n")
            self.expand_gridmap('-x')
        
        if (pcl_wrt_origin[:,0] > self.grid.info.width * self.resolution).any():
            self.get_logger().info("\n\n Expanding the grid in  'x' direction \n\n")
            self.expand_gridmap('x')

        if (pcl_wrt_origin[:,1] < 0).any():
            self.get_logger().info("\n\n Expanding the grid in  '-y' direction \n\n")
            self.expand_gridmap('-y')
        
        if (pcl_wrt_origin[:,1] > self.grid.info.height * self.resolution).any():
            self.get_logger().info("\n\n Expanding the grid in  'y' direction \n\n")
            self.expand_gridmap('y')

        # Convert points to grid indices
        pcl_2dim_indices = np.floor(pcl_wrt_origin / self.resolution).astype(int)
        pcl_flat_indices = self.grid.info.width * pcl_2dim_indices[:,1] + pcl_2dim_indices[:,0]

        # Update occupancy grid cells to occupied
        for idx in pcl_flat_indices:
            self.grid.data[idx] = 100

        # Publish the updated occupancy grid
        self.publish_grid.publish(self.grid)

    def expand_gridmap(self, direction: str) -> None:
        """
        Expand the occupancy grid map in the specified direction.
        possible direction inputs:
            '-x', 'x', '-y', 'y'
        """
        dirs = ['-x', 'x', '-y', 'y']
        assert direction in dirs, f"input 'direction' should be one of {dirs}, not {direction}"
        
        height, width = self.grid.info.height, self.grid.info.width
        grid = np.array(self.grid.data).reshape(height, width)

        if direction == '-x':
            # Expand grid backward
            grid = np.hstack((-1 * np.ones((height, int(self.expansion_size / self.resolution))), grid))
            self.grid.info.origin.position.x -= self.expansion_size
            self.x_width += self.expansion_size
            self.grid.info.width = int(self.x_width / self.resolution)
        elif direction == 'x':
            # Expand grid forward
            grid = np.hstack((grid, -1 * np.ones((height, int(self.expansion_size / self.resolution)))))
            self.x_width += self.expansion_size
            self.grid.info.width = int(self.x_width / self.resolution)
        elif direction == '-y':
            # Expand grid to the global right
            grid = np.vstack((-1 * np.ones((int(self.expansion_size / self.resolution), width)), grid))
            self.grid.info.origin.position.y -= self.expansion_size
            self.y_width += self.expansion_size
            self.grid.info.height = int(self.y_width / self.resolution)
        elif direction == 'y':
            # Expand grid to the global left
            grid = np.vstack((grid, -1 * np.ones((int(self.expansion_size / self.resolution), width))))
            self.y_width += self.expansion_size
            self.grid.info.height = int(self.y_width / self.resolution)

        # Update grid data
        expanded_data = [int(nr) for nr in grid.flatten()]
        self.grid.data = expanded_data


def _get_mat_from_quat(quaternion: np.ndarray) -> np.ndarray:
    """
    Taken from the ROS2 repo: 
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


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = PointcloudToGridNode()
    try:
        executor.add_node(node)
        executor.spin()
        # rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
