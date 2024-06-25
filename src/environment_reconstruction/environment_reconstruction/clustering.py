import rclpy
import rclpy.node as node
import open3d as o3d
import numpy as np
from interfaces.srv import PointcloudTransform
from environment_reconstruction.utils import create_unstructured_pointcloud
from sensor_msgs.msg import PointCloud2


class Clusterer(node.Node):

    def __init__(self):
        super().__init__('clusterer')

        # Declare ROS parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ("cluster_service", "cluster_reconstruction"),  # Service name parameter
            ]
        )

        # Get the service name from the parameter
        self.service_tag = self.get_parameter("cluster_service").get_parameter_value().string_value  

        # Initialize a PointCloud2 message
        self.pointcloud = PointCloud2()

        # Create the service for point cloud clustering
        self.serv_cluster = self.create_service(
            PointcloudTransform,
            self.service_tag,
            self.cluster
        )
        
        self.get_logger().info("Clusterer node created")

    def cluster(self, request, response):
        """
        Service callback to cluster the input point cloud using DBSCAN.
        """
        # Load the point cloud data from the request into Open3D format
        pcl_o3d = o3d.geometry.PointCloud()

        self.get_logger().info("Loading points into Open3D")
        
        loaded_array = np.frombuffer(request.pointcloud.data, dtype=np.float32).reshape(-1, 3)
        pcl_o3d.points = o3d.utility.Vector3dVector(loaded_array)

        # Perform DBSCAN clustering
        labels = pcl_o3d.cluster_dbscan(eps=0.05, min_points=10)

        # Prepare the data with cluster labels
        lbls = np.expand_dims(np.asarray(labels, dtype=np.int8), 1)
        datapoints = np.hstack((np.asarray(pcl_o3d.points, dtype=np.float32), lbls))

        # Create the clustered point cloud message
        pcl_msg = create_unstructured_pointcloud(
            datapoints, 
            frame_id=request.pointcloud.header.frame_id,
            time=request.pointcloud.header.stamp
        )
        
        # Set the response with the clustered point cloud
        response.transformed_pointcloud = pcl_msg
        return response
    

def main(args=None):
    """Main function to initialize and spin the Clusterer node."""
    rclpy.init(args=args)

    clusterer = Clusterer()

    rclpy.spin(clusterer)

    clusterer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
