import numpy as np
import rclpy 
import rclpy.node as node
from interfaces.srv import ReconstructImage
import cv2 as cv
import open3d as o3d
from image2map.utils import create_unstructured_pointcloud
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class Reconstructor(node.Node):
    def __init__(self):
        super().__init__('reconstructor')

        self.declare_parameters(
            namespace='',
            parameters = [('depth_lim', 30),
                          ('height_lim', 1.5),
                          ('voxel_downsample_size', 0.1)] 
        )

        self.depth_lim = self.get_parameter('depth_lim').value
        self.height_threshold = self.get_parameter('height_lim').value
        self.voxel_downsample_size = self.get_parameter('voxel_downsample_size').value

        self.floor_threshold = 0.1
        
        callbackgroup = ReentrantCallbackGroup()
        self.srv = self.create_service(ReconstructImage, 
                                       'reconstruct_3d_view',
                                       self.reconstruct_view,
                                       callback_group=callbackgroup)
    
        self.cv_bridge = CvBridge()

        self.get_logger().info("Reconstructor node created")
      

    def reconstruct_view(self, request, response):
        """
        Steps:
        1. Obtain disparity map
        2. Compute Z from disparity map
        3. Compute X, Y from Z and internal camera matrices
        """
        self.get_logger().info("Reconstructing view")

        cam_info = request.cam_info_l
        baseline = request.baseline
        img_header = request.left_image.header 
        img_l, img_r = self.cv_bridge.imgmsg_to_cv2(request.left_image, 'rgb8'), self.cv_bridge.imgmsg_to_cv2(request.right_image, 'rgb8')
        

        disparity = self.find_disparity_map(img_l, img_r)
        
        intrinsic_mat = np.array([cam_info.k[:3], cam_info.k[3:6], cam_info.k[6:]]).astype(np.float32)

        pc_data = self.find_real_world_coords_bmm(disparity, intrinsic_mat, baseline, img_l.shape)

        reconstruction = create_unstructured_pointcloud(pc_data, frame_id=img_header.frame_id, time=img_header.stamp)

        response.pointcloud = reconstruction
        response.pose = request.pose
        return response
    
    def find_disparity_map(self, left_image, right_image):
        l_img_grey, r_img_grey = cv.cvtColor(left_image, cv.COLOR_RGB2GRAY), cv.cvtColor(right_image, cv.COLOR_RGB2GRAY)
        cv_stereo = cv.StereoBM_create(numDisparities=160, blockSize=25)
        disparity_map = cv_stereo.compute(l_img_grey, r_img_grey).astype(np.float32)/16

        return disparity_map
    
     
    def find_real_world_coords_bmm(self, disparity_map, intrinsic_mat, cam_spacing, img_shape):
    
        fx = intrinsic_mat[0,0]
        int_mat_inv = np.linalg.inv(intrinsic_mat)
        depth = fx * cam_spacing / disparity_map
        depth[abs(depth) > self.depth_lim] = np.nan
        img_xy = np.indices((img_shape[1], img_shape[0])).reshape((2,-1)).T 
        batched_imgxy1 = np.hstack((img_xy, np.ones((img_shape[0]*img_shape[1],1)))) # Add ones in the z dimension

        # TODO: prune batched imgxyz and depth before this point 
        
        batched_imgxyz = np.einsum("ij, ik->ij", batched_imgxy1, depth.T.reshape(-1,1)) # multiply each [x,y,1] with depth

        XYZ = np.einsum("bi, ij -> bj", batched_imgxyz, int_mat_inv.T) # Obtain XYZ coordinates from intrinsic camera matrix 

        # Set points that are irrelevant to nan so they will be filtered out later. The floor threshold should filter out points that are reconstructions of the 
        # floor, and therefore not obstacles. The height threshold filters out points that can be driven underneath.
        XYZ[np.where((XYZ[:,1] < self.floor_threshold) + (XYZ[:,1] > self.height_threshold))] = [np.nan, np.nan, np.nan] 
        
        # To save computation time, the points can be projected to the floor (or at least a single plane) so they will be grouped by voxel_down_sample later.
        # This means Z should be zero. However due to the rotation below, this is actually still Y in this step (in the camera reference frame). 
        # The /home/ros/ros2_ws/src/image2map/resourcereason to do this projects at this point is to save on an additional np.asarray() casting and back to open3d

        XYZ[:,1] = 0 

        # Rotation to transform from typical camera coordinate system to world coordinate conventions
        # Camera has Z as depth and X to the right, with ROS convention being X forward and Z up
        # Maybe this can be added to the extrinsic camera matrix to bypass these extra steps
        pcl_o3d = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(XYZ)).remove_non_finite_points()
        rot_mat = pcl_o3d.get_rotation_matrix_from_xyz((-np.pi/2, np.pi/2, 0))
        pcl_o3d.rotate(R=rot_mat, center=np.array([[0],[0],[0]]))
        
        # self.get_logger().info(f"Initial point count: {pcl_o3d.points}")

        pcl_o3d, _ = pcl_o3d.remove_statistical_outlier(
            nb_neighbors=20,
            std_ratio=1.0
        )


        pcl_o3d = pcl_o3d.voxel_down_sample(self.voxel_downsample_size)

        point_cloud_data = np.asarray(pcl_o3d.points, dtype=np.float32)
        # self.get_logger().info(f"SHAPE OF FILTERED POITNCLOUD: \n\n {point_cloud_data.shape} \n\n NaNs: {np.sum(np.isnan(point_cloud_data))}")
    
        return point_cloud_data


def main(args=None):
    rclpy.init(args=args)
    
    reconstructor = Reconstructor()
    executor = MultiThreadedExecutor()
    executor.add_node(reconstructor)
    executor.spin()

    reconstructor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
