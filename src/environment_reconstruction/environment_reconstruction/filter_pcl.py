import rclpy 
import rclpy.node as node
import open3d as o3d 
from interfaces.srv import PointcloudTransform
import numpy as np
from environment_reconstruction.utils import create_unstructured_pointcloud


class PClFilterNode(node.Node):
    
    def __init__(self):
        super().__init__('pcl_filter_node')

        self.srv_filter = self.create_service(PointcloudTransform,
                                              'filter_pcl',
                                              self.filter)
        
        """
        Try:
            statistical filter
            radius filter
            voxel downsampling
        """

    def filter(self, request, response):
        header = request.pointcloud.header 
        pcl_o3d = o3d.geometry.PointCloud()

        self.get_logger().info("Loading points into Open3D")
        
        loaded_array = np.frombuffer(request.pointcloud.data, dtype=np.float32).reshape(-1, 3)
        pcl_o3d.points = o3d.utility.Vector3dVector(loaded_array)
        
        self.get_logger().info(f"Initial point count: {pcl_o3d.points}")
        pcl_o3d, _ = pcl_o3d.remove_statistical_outlier(
            nb_neighbors=20,
            std_ratio=1.0
        )
        self.get_logger().info(f"Filtered point count: {pcl_o3d.points}")

        pcl_msg = create_unstructured_pointcloud(np.asarray(pcl_o3d.points, dtype=np.float32), 
                                                            frame_id=header.frame_id, 
                                                            time=header.stamp)
        
        response.transformed_pointcloud = pcl_msg
        return response 
    

def main(args=None):
    rclpy.init(args=args)
    
    filter_node = PClFilterNode()
    rclpy.spin(filter_node)


    filter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()