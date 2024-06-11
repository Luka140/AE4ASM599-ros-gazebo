import rclpy
import rclpy.node as node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from interfaces.srv import ReconstructImage, PointcloudTransform, CreateOccupancyMap

from tf2_ros import TransformListener, LookupException, ExtrapolationException
from tf2_ros.buffer import Buffer

from rclpy.callback_groups import ReentrantCallbackGroup


class Coordinator(node.Node):
    
    def __init__(self):
        super().__init__('coordinator')

        self.declare_parameters(
            namespace='',
            parameters = [('world_frame', 'world'),
                          ('baseline', 1.),
                          ('period', 3.5)] # period in seconds 
        )

        self.world_frame = self.get_parameter('world_frame').value
        self.baseline = self.get_parameter('baseline').value
        timer_period = self.get_parameter('period').value # seconds

        self.img_l, self.img_r, self.cam_info_l, self.cam_info_r = [None]*4
        self.camera_poses = []
        self.relative_pointclouds = []
        self.call_count = 0

        self.callback_group = ReentrantCallbackGroup()

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
        
        self.reconstruction_client = self.create_client(ReconstructImage, 'reconstruct_3d_view')
        self.filter_client = self.create_client(PointcloudTransform, 'filter_pcl')
        self.map_client = self.create_client(CreateOccupancyMap, 'create_occupancy_map')

        self.filtered_publisher = self.create_publisher(PointCloud2, 'filtered_reconstruction', 10)
        self.map_publisher = self.create_publisher(OccupancyGrid, 'occupancy_map', 10)

        self.tf_buffer = Buffer(cache_time=rclpy.time.Time(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(timer_period, self.timer_callback)

        """
        todo: make cameras on trigger
        """
        self.get_logger().info(f"Coordinator node started - time {self.get_clock().now()}")

    def timer_callback(self):
        """
        This callback executes the following pipeline:
        - Collect the latest images
        - Collect the current pose
        - Perform a the 3d reconstruction based on these images
        - Filter down these points based on voxel-downfiltering (this is already performed in the 3d reconstruction call)
        - Update the occupancy map based on these points 
        """
        self.get_logger().info("Starting reconstruction")

        if self.img_l is None or self.img_r is None or self.cam_info_l is None or self.cam_info_r is None:
            self.get_logger().info("No camera data received yet - skipping cycle")
            return
        
        self.call_count += 1

        reconstruct = self.create_reconstruct_msg()

        # Find the current position from the transform buffer 
        try:
            self.get_logger().info("Lookup up transform")
            tf_trans = self.tf_buffer.lookup_transform(self.world_frame, reconstruct.left_image.header.frame_id, 
                                                       rclpy.time.Time(seconds=0)) 
            
        except (LookupException, ExtrapolationException) as exception:
            self.get_logger().info(f"\n\nCould not lookup transform for call {self.call_count}\n\n {exception}")
            return
        
        # The current pose is passed along with the reconstruction request, so that it can be fed through to the mapping request
        reconstruct.pose = Pose()
        reconstruct.pose.position.x = tf_trans.transform.translation.x
        reconstruct.pose.position.y = tf_trans.transform.translation.y
        reconstruct.pose.position.z = tf_trans.transform.translation.z
        reconstruct.pose.orientation = tf_trans.transform.rotation

        # Request the reconstruction - the next steps will be called from the recon_done_callback
        reconstruction = self.reconstruction_client.call_async(reconstruct)
        reconstruction.add_done_callback(self.recon_done_callback)
        
        self.camera_poses.append(tf_trans)
        
    def recon_done_callback(self, result):
        res = result.result()
        pointcloud = res.pointcloud
        pose = res.pose

        self.relative_pointclouds.append(pointcloud)
        self.filtered_publisher.publish(pointcloud)
        self.get_logger().info("Published pointcloud ")

        # Request an update of the occupancy map based on the finished reconstruction
        map_request = CreateOccupancyMap.Request()
        map_request.pointcloud = pointcloud
        map_request.pose = pose 

        call_map = self.map_client.call_async(map_request)
        call_map.add_done_callback(self.map_done_callback)

    def map_done_callback(self, result):
        # publish the updated occupancy grid
        self.grid = result.result().occupancygrid
        self.map_publisher.publish(self.grid)
        self.get_logger().info("Published map")
    
    def update_img_l(self, img):
        # update to the latest image
        self.img_l = img

    def update_img_r(self, img):
        # update to the latest image
        self.img_r = img

    def update_camera_info(self, cam_info):
        # update to the latest camera info
        if "right_" in cam_info.header.frame_id:
            self.cam_info_r = cam_info
        else:
            self.cam_info_l = cam_info

    def create_reconstruct_msg(self):
        reconstruct_msg = ReconstructImage.Request()
        reconstruct_msg.left_image = self.img_l
        reconstruct_msg.right_image = self.img_r
        reconstruct_msg.cam_info_l = self.cam_info_l
        reconstruct_msg.cam_info_r = self.cam_info_r
        reconstruct_msg.baseline = self.baseline
        return reconstruct_msg
    

def main(args=None):
    rclpy.init(args=args)
    
    coordinator = Coordinator()
    rclpy.spin(coordinator)

    coordinator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
