import rclpy 
import rclpy.node as node
import open3d as o3d 
from interfaces.srv import PointcloudTransform
import numpy as np
from environment_reconstruction.utils import create_unstructured_pointcloud


class PClFilterNode(node.Node):
    def __init__(self):
        super().__init__('pcl_filter_node')

        # Declare ROS parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ("filter_service", "filter_pcl"),  # Service name parameter
            ]
        )

        # Get the service name from the parameter
        self.service_tag = self.get_parameter("filter_service").get_parameter_value().string_value  

        # Create the service for point cloud filtering
        self.srv_filter = self.create_service(
            PointcloudTransform,
            self.service_tag,
            self.filter
        )
        
    def filter(self, request, response):
        """
        Service callback to filter the input point cloud.
        Applies statistical outlier removal and voxel down-sampling.
        """
        # Extract the header and point cloud data from the request
        header = request.pointcloud.header 
        pcl_o3d = o3d.geometry.PointCloud()

        self.get_logger().info("Loading points into Open3D")

        # Load point cloud data into Open3D format
        loaded_array = np.frombuffer(request.pointcloud.data, dtype=np.float32).reshape(-1, 3)
        pcl_o3d.points = o3d.utility.Vector3dVector(loaded_array)
        
        self.get_logger().info(f"Initial point count: {len(pcl_o3d.points)}")

        # Apply statistical outlier removal filter
        pcl_o3d, _ = pcl_o3d.remove_statistical_outlier(
            nb_neighbors=20,
            std_ratio=1.0
        )

        # Apply voxel down-sampling filter
        pcl_o3d = pcl_o3d.voxel_down_sample(0.1)

        self.get_logger().info(f"Filtered point count: {len(pcl_o3d.points)}")

        # Create the filtered point cloud message
        pcl_msg = create_unstructured_pointcloud(
            np.asarray(pcl_o3d.points, dtype=np.float32), 
            frame_id=header.frame_id, 
            time=header.stamp
        )
        
        # Set the response with the filtered point cloud
        response.transformed_pointcloud = pcl_msg
        return response 
    

def main(args=None):
    """Main function to initialize and spin the PClFilterNode."""
    rclpy.init(args=args)
    
    filter_node = PClFilterNode()
    rclpy.spin(filter_node)

    filter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
