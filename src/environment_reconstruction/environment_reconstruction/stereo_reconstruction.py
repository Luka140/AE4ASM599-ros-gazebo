import numpy as np
import rclpy 
import rclpy.node as node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, PointField
from std_msgs.msg import Header
from interfaces.srv import Reconstruct
from cv_bridge import CvBridge
import cv2 as cv
import open3d as o3d


class Reconstructor(node.Node):
    def __init__(self):
        super().__init__('reconstructor')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ("distance_limit", 30.0),
                ("left_camera_topic", '/camera_l'), 
                ("right_camera_topic", '/camera_r'), 
                ("camera_info_topic", '/camera_info'),       
                ("published_topic", 'reconstruction'),
                ("recon_service_tag", "reconstruct_3d_view")
            ]
        )

        # Get parameters
        self.depth_lim = self.get_parameter("distance_limit").get_parameter_value().double_value
        self.cam_l_topic = self.get_parameter("left_camera_topic").get_parameter_value().string_value
        self.cam_r_topic = self.get_parameter("right_camera_topic").get_parameter_value().string_value
        self.cam_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.published_topic = self.get_parameter("published_topic").get_parameter_value().string_value
        self.service_topic = self.get_parameter("recon_service_tag").get_parameter_value().string_value

        
        # Initialize variables to store images and camera info
        self.img_l, self.img_r, self.cam_info_l, self.cam_info_r = None, None, None, None
        self.cv_bridge = CvBridge()
        
        # Create subscriptions for left and right camera images
        self.cam_sub_l = self.create_subscription(
            Image,
            self.cam_l_topic,
            self.update_img_l,
            10
        )
        self.cam_sub_r = self.create_subscription(
            Image,
            self.cam_r_topic,
            self.update_img_r,
            10
        )

        # Create subscription for camera info
        self.cam_sub_info = self.create_subscription(
            CameraInfo,
            self.cam_info_topic,
            self.update_camera_info,
            10
        )

        # Create service for creating the reconstruction
        self.srv = self.create_service(Reconstruct, 
                               self.service_topic,
                               self.reconstruct_view)

        # Create the publisher for the reconstructed point cloud
        self.pub = self.create_publisher(
            PointCloud2, 
            self.published_topic, 
            10
        )
        
        self.get_logger().info("Reconstructor node created")

    def update_img_l(self, img):
        """Callback to update left camera image."""
        self.image_time = img.header.stamp
        self.img_l = self.cv_bridge.imgmsg_to_cv2(img, 'rgb8')

    def update_img_r(self, img):
        """Callback to update right camera image."""
        self.image_time = img.header.stamp
        self.img_r = self.cv_bridge.imgmsg_to_cv2(img, 'rgb8')

    def update_camera_info(self, cam_info):
        """Callback to update camera info for left or right camera."""
        if "right_" in cam_info.header.frame_id:
            self.cam_info_r = cam_info
        else:
            self.cam_info_l = cam_info

    def reconstruct_view(self, request, response):
        """
        Handle the reconstruction service request.
        Steps:
        1. Obtain disparity map
        2. Compute Z from disparity map
        3. Compute X, Y from Z and internal camera matrices
        """

        self.get_logger().info("Reconstructing view")
        
        # Check if both images have been received
        if self.img_l is None or self.img_r is None:
            self.get_logger().info("No image received yet")
            response.reconstruction = PointCloud2()
            return response
        
        # ----------------------------------------------------------------
        # Check if the image is outdated
        current_time = self.get_clock().now().to_msg()
        if float(current_time.nanosec) > float(self.image_time.nanosec) + 1e8:  # if image outdated by 0.1sec
            self.get_logger().info("Image time outdated - time set to current with the assumption gazebo is paused")
            curr_time = rclpy.time.Time().from_msg(current_time)
            self.image_time = (curr_time - rclpy.duration.Duration(seconds=0, nanoseconds=1e8)).to_msg()
        # ----------------------------------------------------------------
        
        # Get camera spacing from request
        cam_spacing = request.camera_spacing
        
        # Obtain the disparity map
        disparity = self.find_disparity_map()
        
        # Get intrinsic camera matrix
        intrinsic_mat = np.array([self.cam_info_l.k[:3], self.cam_info_l.k[3:6], self.cam_info_l.k[6:]]).astype(np.float32)
        
        # Compute real-world coordinates
        pc_data = self.find_real_world_coords_bmm(disparity, intrinsic_mat, cam_spacing)

        # Create and publish point cloud message
        reconstruction = create_pointcloud_msg(pc_data, self.image_time, self.img_l.shape, self.cam_info_l.header.frame_id)
        self.pub.publish(reconstruction)
        response.reconstruction = reconstruction
        return response
    
    def find_disparity_map(self):
        """Compute the disparity map from the left and right images."""
        l_img_grey = cv.cvtColor(self.img_l, cv.COLOR_RGB2GRAY)
        r_img_grey = cv.cvtColor(self.img_r, cv.COLOR_RGB2GRAY)
        cv_stereo = cv.StereoBM_create(numDisparities=160, blockSize=25)
        disparity_map = cv_stereo.compute(l_img_grey, r_img_grey).astype(np.float32) / 16
        return disparity_map
        
    def find_real_world_coords_bmm(self, disparity_map, intrinsic_mat, cam_spacing):
        """
        Compute real-world coordinates using batched matrix multiplication.
        
        Args:
            disparity_map: The disparity map.
            intrinsic_mat: Intrinsic camera matrix.
            cam_spacing: The spacing between the cameras.

        Returns:
            np.ndarray: Flattened array of point cloud data.
        """
        fx = intrinsic_mat[0, 0]  # Focal length in x direction

        # Compute depth from disparity
        depth = fx * cam_spacing / disparity_map
        depth[abs(depth) > self.depth_lim] = np.nan
        
        # Compute image coordinates
        img_xy = np.indices((self.img_l.shape[1], self.img_l.shape[0])).reshape((2, -1)).T
        batched_imgxy1 = np.hstack((img_xy, np.ones((self.img_l.shape[0] * self.img_l.shape[1], 1))))  # Add ones in the z dimension
        
        # Multiply each [x, y, 1] with depth
        batched_imgxyz = np.einsum("ij, ik->ij", batched_imgxy1, depth.T.reshape(-1, 1))

        # Obtain XYZ coordinates from intrinsic camera matrix
        XYZ = np.einsum("bi, ij -> bj", batched_imgxyz, np.linalg.inv(intrinsic_mat).T)

        # Rotate to transform from typical camera coordinate system to world coordinate conventions
        pcl_o3d = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(XYZ))
        rot_mat = pcl_o3d.get_rotation_matrix_from_xyz((-np.pi / 2, np.pi / 2, 0))
        pcl_o3d.rotate(R=rot_mat, center=np.array([[0], [0], [0]]))

        # Flatten to a 1D array to make a point cloud out of it 
        point_cloud_data = np.asarray(pcl_o3d.points).flatten()
        return point_cloud_data

    def find_real_world_coords_ocv(self, disparity_map, intrinsic_mat, cam_spacing):
        """
        Compute real-world coordinates using OpenCV functions.
        
        Args:
            disparity_map: The disparity map.
            intrinsic_mat: Intrinsic camera matrix.
            cam_spacing: The spacing between the cameras.

        Returns:
            np.ndarray: Flattened array of point cloud data.
        
        Note: This method is not currently used as it is not working correctly.
        """
        intrinsic_mat_l = np.array([self.cam_info_l.k[:3], self.cam_info_l.k[3:6], self.cam_info_l.k[6:]]).astype(np.float32)
        intrinsic_mat_r = np.array([self.cam_info_r.k[:3], self.cam_info_r.k[3:6], self.cam_info_r.k[6:]]).astype(np.float32)
        
        # Rectify the images and obtain the reprojection matrix Q
        R1, R2, P1, P2, Q, *_extra_returns = cv.stereoRectify(
            intrinsic_mat_l, np.zeros((1, 5)), 
            intrinsic_mat_r, np.zeros((1, 5)), 
            self.img_l.shape[:2], np.zeros((3, 3)), 
            np.array([[cam_spacing], [0], [0]])
        )
        
        # Reproject image to 3D
        XYZ = cv.reprojectImageTo3D(disparity_map, Q)

        # Flatten to a 1D array to make a point cloud out of it 
        point_cloud_data = XYZ.reshape(-1, 3).flatten()
        assert (XYZ[0, 0, :] == point_cloud_data[:3]).all(), f"{XYZ[0, 0, :]}, {point_cloud_data[:3]} are not the same, the matrix is not flattened correctly"
        self.get_logger().info(f"XYZ first entries: {point_cloud_data[:6]}")
        return point_cloud_data


def create_pointcloud_msg(data, time, img_shape, reference_frame):
    """
    Create a PointCloud2 message from the given data.

    Args:
        data: A 1D numpy array containing the point cloud data. The points are ordered as [x1, y1, z1, x2, y2, z2, ...].
        time: The time stamp from the moment the service was called.
        img_shape: The shape of the image.

    Returns:
        PointCloud2: The created PointCloud2 message.
    """

    pointcloud = PointCloud2()
    pointcloud.header = Header(stamp=time, frame_id=reference_frame)

    pointcloud.height, pointcloud.width = img_shape[:2]
    bytes_per_point = 4
    fields = [PointField(name=direction, offset=i * bytes_per_point, datatype=PointField.FLOAT32, count=1) for i, direction in enumerate(['x', 'y', 'z'])]
    pointcloud.fields = fields
    pointcloud.point_step = len(fields) * bytes_per_point
    total_num_of_points = pointcloud.height * pointcloud.width
    pointcloud.row_step = pointcloud.point_step * total_num_of_points
    pointcloud.is_dense = False
    pointcloud.is_bigendian = False
    pointcloud.data = data.astype(np.float32).tobytes()  # This seems to take quite some time 
    return pointcloud


def main(args=None):
    """Main function to initialize and spin the reconstructor node."""
    rclpy.init(args=args)
    
    reconstructor = Reconstructor()

    rclpy.spin(reconstructor)

    reconstructor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
