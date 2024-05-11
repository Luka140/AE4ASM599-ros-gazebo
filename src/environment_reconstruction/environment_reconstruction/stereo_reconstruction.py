import numpy as np
import rclpy 
import rclpy.node as node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, PointField
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Header
from interfaces.srv import Reconstruct
from cv_bridge import CvBridge
import cv2 as cv
import matplotlib.pyplot as plt
import struct


class Reconstructor(node.Node):
    def __init__(self):
        super().__init__('reconstructor')
        self.img_l, self.img_r, self.cam_info_l, self.cam_info_r = None, None, None, None
        self.cv_bridge = CvBridge()
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
        self.img_l = self.cv_bridge.imgmsg_to_cv2(img, 'rgb8')

    def update_img_r(self, img):
        self.img_r = self.cv_bridge.imgmsg_to_cv2(img, 'rgb8')

    def update_camera_info(self, cam_info):
        if "right_" in cam_info.header.frame_id:
            self.cam_info_r = cam_info
        else:
            self.cam_info_l = cam_info
        

    def reconstruct_view(self, request, response):
        """
        Z-error grows quadratically with distance, maybe cap the maximum distance

        Steps:
        1. Obtain disparity map
        2. Compute Z from disparity map
        """
        time = self.get_clock().now().to_msg()

        cam_spacing = request.camera_spacing
        disparity = self.find_disparity_map()
        
        intrinsic_mat = np.array([self.cam_info_l.k[:3], self.cam_info_l.k[3:6],self.cam_info_l.k[6:]]).astype(np.float32)
        # coords = self.find_real_world_coords(disparity, intrinsic_mat, cam_spacing)
        pc_data = self.find_real_world_coords(disparity, intrinsic_mat, cam_spacing)

        response.reconstruction = self.create_pointcloud_msg(pc_data, time)
        return response
    
    def find_disparity_map(self):
        l_img_grey, r_img_grey = cv.cvtColor(self.img_l, cv.COLOR_RGB2GRAY), cv.cvtColor(self.img_r, cv.COLOR_RGB2GRAY)
        cv_stereo = cv.StereoBM_create(numDisparities=160, blockSize=25)
        # cv_stereo.setTextureThreshold(5)
        disparity_map = cv_stereo.compute(l_img_grey, r_img_grey).astype(np.float32)/16
        # self.get_logger().info(f"min disparity: {np.min(disparity_map)} max disparity: {np.max(disparity_map)}")
        # plt.imshow(disparity_map, 'gray')
        # plt.show()
        return disparity_map
    
    def find_real_world_coords(self, disparity_map, intrinsic_mat, cam_spacing):
        fx = intrinsic_mat[0,0]
        # fy =  intrinsic_mat[1,1]
        # cx, cy = intrinsic_mat[0,2], intrinsic_mat[1,2]
        int_mat_inv = np.linalg.inv(intrinsic_mat)
        depth = fx * cam_spacing / disparity_map
        # scaling_factor = 1
        # translation = np.zeros((3,1))
        point_cloud_data = np.empty(np.prod(self.img_l.shape[:2])*3, dtype=np.float32)
        # world_coords = np.zeros((*self.img_l.shape[:2], 3))
        k = 0
        for j in range(self.img_l.shape[0]):
            for i in range(self.img_l.shape[1]):
                depth_world = depth[j,i]
                # x_img, y_img = (i-cx) / fx, (i-cy) / fy
                xy = int_mat_inv @ np.array([[i],[j],[1]]) 
                XYZ = xy * depth_world
                # world_coords[j,i,:] = XYZ.flatten()
                point_cloud_data[k:k + 3] = XYZ.T
                k += 3

        return point_cloud_data
    
    def create_pointcloud_msg(self, data, time):
        
        pointcloud = PointCloud2()
        pointcloud.header = Header(stamp=time, frame_id="camera_frame")
        pointcloud.height = self.img_l.shape[0]
        pointcloud.width = self.img_l.shape[1]
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1), 
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
        pointcloud.fields = fields
        # Float occupies 4 bytes. Each point then carries 16 bytes.
        pointcloud.point_step = len(fields) * 4 
        total_num_of_points = pointcloud.height * pointcloud.width
        pointcloud.row_step = pointcloud.point_step * total_num_of_points
        pointcloud.is_dense = True
        pointcloud.is_bigendian = False
        # data = []
        # for j in range(coords.shape[0]):
        #     for i in range(coords.shape[1]):
        #         for k in range(coords.shape[2]):
        #             data.extend(struct.pack('<f', coords[j,i,k]))
        pointcloud.data = data.astype(np.float32).tobytes()
        self.pub.publish(pointcloud)
        return pointcloud

def main(args=None):
    rclpy.init(args=args)
    
    reconstructor = Reconstructor()

    rclpy.spin(reconstructor)

    reconstructor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



# pointcloud_msg = PointCloud2()
# pointcloud_msg.header = Header()
# pointcloud_msg.header.frame_id = "frame"

# # Define the point fields (attributes)        
# fields =[PointField('x', 0, PointField.FLOAT32, 1),
#         PointField('y', 4, PointField.FLOAT32, 1),
#         PointField('z', 8, PointField.FLOAT32, 1),
#         PointField('intensity', 12, PointField.FLOAT32,1),
#         ]

# pointcloud_msg.fields = fields

# pointcloud_msg.height = height
# pointcloud_msg.width = width

# # Float occupies 4 bytes. Each point then carries 16 bytes.
# pointcloud_msg.point_step = len(fields) * 4 

# total_num_of_points = pointcloud_msg.height * pointcloud_msg.width
# pointcloud_msg.row_step = pointcloud_msg.point_step * total_num_of_points
# pointcloud_msg.is_dense = True