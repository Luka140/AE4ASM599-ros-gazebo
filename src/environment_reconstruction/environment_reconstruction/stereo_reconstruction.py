import numpy as np
import rclpy 
import rclpy.node as node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, PointField
from std_msgs.msg import Header
from interfaces.srv import Reconstruct
from cv_bridge import CvBridge
import cv2 as cv
import matplotlib.pyplot as plt
import open3d as o3d


class Reconstructor(node.Node):
    def __init__(self):
        super().__init__('reconstructor')
        self.get_logger().info("Reconstructor node created")
        self.img_l, self.img_r, self.cam_info_l, self.cam_info_r = None, None, None, None
        self.cv_bridge = CvBridge()
        self.depth_lim = 20
        self.cam_sub_l = self.create_subscription(Image,
                                                '/camera_l',
                                                self.update_img_l,
                                                10)
        self.cam_sub_r = self.create_subscription(Image,
                                                '/camera_r',
                                                self.update_img_r,
                                                10)

        self.cam_sub_info = self.create_subscription(CameraInfo,
                                        '/camera_info',
                                        self.update_camera_info,
                                        10)
        
        self.srv = self.create_service(Reconstruct, 
                                       'reconstruct_3d_view',
                                       self.reconstruct_view)
        
        # TODO This pub shouldnt be here, temp 
        self.pub = self.create_publisher(PointCloud2, 
                                       'reconstruction', 10)
                
        """
        TODO:
        This is kinda inefficient... Update to make the camera only take pics on trigger
        """

    def update_img_l(self, img):
        self.image_time = self.get_clock().now().to_msg()
        self.img_l = self.cv_bridge.imgmsg_to_cv2(img, 'rgb8')

    def update_img_r(self, img):
        self.image_time = self.get_clock().now().to_msg()
        self.img_r = self.cv_bridge.imgmsg_to_cv2(img, 'rgb8')

    def update_camera_info(self, cam_info):
        if "right_" in cam_info.header.frame_id:
            self.cam_info_r = cam_info
        else:
            self.cam_info_l = cam_info
        

    def reconstruct_view(self, request, response):
        """
        TODO: Z-error grows quadratically with distance, maybe cap the maximum distance

        Steps:
        1. Obtain disparity map
        2. Compute Z from disparity map
        3. Compute X, Y from Z and internal camera matrices
        """
        if self.img_l is None or self.img_r is None:
            self.get_logger().info("No image received yet")
            response.reconstruction = PointCloud2()
            return response
        
        current_time = self.get_clock().now().to_msg()
        self.get_logger().info(f"{current_time.nanosec, self.image_time.nanosec}")
        if float(current_time.nanosec) > float(self.image_time.nanosec) + 1e8: # if image outdated by 0.1sec
            self.get_logger().info("Image time outdated - time set to current with the assumption gazebo is paused")
            
            curr_time = rclpy.time.Time().from_msg(current_time)
            self.image_time = (
                curr_time - rclpy.time.Time(seconds=0, nanoseconds=1e8, clock_type=curr_time.clock_type)
                ).to_msg()

        
        cam_spacing = request.camera_spacing
        disparity = self.find_disparity_map()
        
        intrinsic_mat = np.array([self.cam_info_l.k[:3], self.cam_info_l.k[3:6],self.cam_info_l.k[6:]]).astype(np.float32)

        """==============================================================================================================================================================

        Below this are the functions I use to calculate the real world coordinates from the disparity map.
        Comment and uncomment to see the different implementations.

        - find_real_words_coords_it() uses an iterative approach and seems to work but it is very slow.
        - find_real_world_coords_bmm() uses a batched matrix multiplication approach and is significantly faster. Equivalent results to approach 1.
        - find_real_world_coords_ocv() uses the opencv reproject_image_to_3d() function from opencv. The results are not correct right now.       
        
        
        =============================================================================================================================================================="""
        # pc_data = self.find_real_world_coords_it(disparity, intrinsic_mat, cam_spacing)
        pc_data = self.find_real_world_coords_bmm(disparity, intrinsic_mat, cam_spacing)
        # pc_data = self.find_real_world_coords_ocv(disparity, intrinsic_mat, cam_spacing)


        reconstruction = create_pointcloud_msg(pc_data, self.image_time, self.img_l.shape)
        self.pub.publish(reconstruction)
        response.reconstruction = reconstruction
        return response
    
    def find_disparity_map(self):
        l_img_grey, r_img_grey = cv.cvtColor(self.img_l, cv.COLOR_RGB2GRAY), cv.cvtColor(self.img_r, cv.COLOR_RGB2GRAY)
        cv_stereo = cv.StereoBM_create(numDisparities=160, blockSize=25)
        disparity_map = cv_stereo.compute(l_img_grey, r_img_grey).astype(np.float32)/16

        # plt.imshow(disparity_map, 'gray')
        # plt.show()
        return disparity_map
    
    def find_real_world_coords_it(self, disparity_map, intrinsic_mat, cam_spacing):
        """
        Version 1:
            iterate over every pixel and compute XYZ
        """
        fx = intrinsic_mat[0,0]
        # cx, cy = intrinsic_mat[0,2], intrinsic_mat[1,2]
        int_mat_inv = np.linalg.inv(intrinsic_mat)
        depth = fx * cam_spacing / disparity_map

        point_cloud_data = np.empty(np.prod(self.img_l.shape[:2])*3, dtype=np.float32)
        k = 0
        for j in range(self.img_l.shape[0]):
            for i in range(self.img_l.shape[1]):
                depth_world = depth[j,i]
                xy = int_mat_inv @ np.array([[i],[j],[1]]) 
                XYZ = xy * depth_world
                point_cloud_data[k:k + 3] = XYZ.T
                k += 3

        return point_cloud_data
    
    def find_real_world_coords_bmm(self, disparity_map, intrinsic_mat, cam_spacing):
        """
        Version 2:
            Should be the same math as version 1 but using batched matrix multiplication instead of iterating over every pixel
        """
        fx = intrinsic_mat[0,0]
        # self.get_logger().info(f"Intrinsic camera matrix: {intrinsic_mat}")
        # cx, cy = intrinsic_mat[0,2], intrinsic_mat[1,2]
        int_mat_inv = np.linalg.inv(intrinsic_mat)
        depth = fx * cam_spacing / disparity_map
        depth[abs(depth) > self.depth_lim] = np.nan
        img_xy = np.indices((self.img_l.shape[1], self.img_l.shape[0])).reshape((2,-1)).T 
        batched_imgxy1 = np.hstack((img_xy, np.ones((self.img_l.shape[0]*self.img_l.shape[1],1)))) # Add ones in the z dimension
        
        batched_imgxyz = np.einsum("ij, ik->ij", batched_imgxy1, depth.T.reshape(-1,1)) # multiply each [x,y,1] with depth

        XYZ = np.einsum("bi, ij -> bj", batched_imgxyz, int_mat_inv.T) # Obtain XYZ coordinates from intrinsic camera matrix 

        # Rotation to transform from typical camera coordinate system to world coordinate convetions
        # Camera has Z as depth and X to the right, with ROS convention being X forward and Z up
        # Maybe this can be added to the extrinsic camera matrix to bypass these extra steps
        pcl_o3d = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(XYZ))
        rot_mat = pcl_o3d.get_rotation_matrix_from_xyz((-np.pi/2, np.pi/2, 0))
        pcl_o3d.rotate(R=rot_mat, center=np.array([[0],[0],[0]]))

        # Flatten to a 1D array to make a pointcloud out of it 
        point_cloud_data = np.asarray(pcl_o3d.points).flatten()
        return point_cloud_data


    def find_real_world_coords_ocv(self, disparity_map, intrinsic_mat, cam_spacing):
        """
        Version 3:
            Using OpenCV functions to obtain real world coordinates from the disparity map
        """
        intrinsic_mat_l = np.array([self.cam_info_l.k[:3], self.cam_info_l.k[3:6],self.cam_info_l.k[6:]]).astype(np.float32)
        intrinsic_mat_r = np.array([self.cam_info_r.k[:3], self.cam_info_r.k[3:6],self.cam_info_r.k[6:]]).astype(np.float32)
        R1, R2, P1, P2, Q, *_extra_returns = cv.stereoRectify(intrinsic_mat_l, np.zeros((1,5)), intrinsic_mat_r, np.zeros((1,5)), self.img_l.shape[:2], np.zeros((3,3)), np.array([[cam_spacing],[0],[0]]))
        XYZ = cv.reprojectImageTo3D(disparity_map, Q)

        # Flatten to a 1D array to make a pointcloud out of it 
        point_cloud_data = XYZ.reshape(-1, 3).flatten()
        assert (XYZ[0,0,:] == point_cloud_data[:3]).all(), f"{XYZ[0,0,:]}, {point_cloud_data[:3]} are not the same, the matrix is not flattened correctly"
        self.get_logbyger().info(f"XYZ first entires: {point_cloud_data[:6]}")
        return point_cloud_data


def create_pointcloud_msg(data, time, img_shape):
    """
    Creates a PointCloud2 message from the given data.
    inputs:
        data: a 1D numpy array containing the point cloud data. The points are ordered as [x1, y1, z1, x2, y2, z2,...].
        time: the time stamp from the moment the service was called.

    For now this publishes the point cloud so RVIZ can visualize it. as well as returning it.
    Would like to change this so that it just returns the point cloud as a service response, but I didn't find a way to visualize the point cloud in RViz without publishing it as  a publisher
    """

    cam_reference_frame = "Test_car/chassis/left_camera"

    pointcloud = PointCloud2()
    pointcloud.header = Header(stamp=time, frame_id=cam_reference_frame)

    pointcloud.height, pointcloud.width = img_shape[:2]
    bytes_per_point = 4
    fields = [PointField(name=direction, offset=i * bytes_per_point, datatype=PointField.FLOAT32, count=1) for i, direction in enumerate(['x','y','z'])]
    pointcloud.fields = fields
    # Float occupies 4 bytes. Each point then carries 16 bytes.
    pointcloud.point_step = len(fields) * bytes_per_point
    total_num_of_points = pointcloud.height * pointcloud.width
    pointcloud.row_step = pointcloud.point_step * total_num_of_points
    pointcloud.is_dense = False
    pointcloud.is_bigendian = False
    pointcloud.data = data.astype(np.float32).tobytes() # This seems to take quite some time 
    return pointcloud


def main(args=None):
    rclpy.init(args=args)
    
    reconstructor = Reconstructor()

    rclpy.spin(reconstructor)

    reconstructor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
