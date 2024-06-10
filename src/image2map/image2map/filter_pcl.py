import rclpy 
import rclpy.node as node
import open3d as o3d 
from interfaces.srv import PointcloudTransform
import numpy as np
from environment_reconstruction.utils import create_unstructured_pointcloud
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor



class PClFilterNode(node.Node):
    
    def __init__(self):
        super().__init__('pcl_filter_node')

        callback_group = ReentrantCallbackGroup()
        self.srv_filter = self.create_service(PointcloudTransform,
                                              'filter_pcl',
                                              self.filter, 
                                              callback_group = callback_group)
        
        self.downsample_voxel_size = 0.1
        """
        TODO: SET FILTER DOWNSAMPLE SIZE FROM LAUNCH FILE
        """

    def filter(self, request, response):
        header = request.pointcloud.header 
        pcl_o3d = o3d.geometry.PointCloud()

        self.get_logger().info("Loading points into Open3D for filtering")
        
        loaded_array = np.frombuffer(request.pointcloud.data, dtype=np.float32).reshape(-1, 3)
        pcl_o3d.points = o3d.utility.Vector3dVector(loaded_array)
        
        self.get_logger().info(f"Initial point count: {pcl_o3d.points}")
        pcl_o3d, _ = pcl_o3d.remove_statistical_outlier(
            nb_neighbors=20,
            std_ratio=1.0
        )

        self.get_logger().info("Downsampling currently disabled - see filter_pcl.py")
        # pcl_o3d = pcl_o3d.voxel_down_sample(self.downsample_voxel_size)

        # self.get_logger().info(f"Filtered point count: {pcl_o3d.points}")

        pcl_msg = create_unstructured_pointcloud(np.asarray(pcl_o3d.points, dtype=np.float32), 
                                                            frame_id=header.frame_id, 
                                                            time=header.stamp)
        
        response.transformed_pointcloud = pcl_msg
        self.get_logger().info("Returning filtered pointcloud")
        return response 
    

def main(args=None):
    rclpy.init(args=args)
    
    filter_node = PClFilterNode()
    executor = MultiThreadedExecutor()
    executor.add_node(filter_node)
    executor.spin()


    filter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()