import rclpy
import rclpy.node as node
from interfaces.srv import PointcloudTransform
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty


class ClusterClient(node.Node):

    def __init__(self):
        super().__init__('cluster_client')

        # Declare ROS parameters for configuring service and topic names
        self.declare_parameters(
            namespace='',
            parameters=[
                ("cluster_service", "cluster_reconstruction"),  # Service name for clustering
                ("filter_service", "filter_pcl"),  # Service name for filtering
                ("cluster_trigger_topic", "cluster_trigger"),  # Topic name for cluster trigger
                ("filter_trigger_topic", "filter_trigger"),  # Topic name for filter trigger
                ("pointcloud_topic", "total_pointcloud"),  # Topic name for incoming point cloud
                ("clustered_pcl_topic", "clustered_reconstruction"),  # Topic name for clustered point cloud
                ("filtered_pcl_topic", "filtered_reconstruction"),  # Topic name for filtered point cloud
            ]
        )

        # Get the parameters
        self.cluster_service = self.get_parameter("cluster_service").get_parameter_value().string_value
        self.filter_service = self.get_parameter("filter_service").get_parameter_value().string_value
        self.cluster_trigger_topic = self.get_parameter("cluster_trigger_topic").get_parameter_value().string_value
        self.filter_trigger_topic = self.get_parameter("filter_trigger_topic").get_parameter_value().string_value
        self.pointcloud_topic = self.get_parameter("pointcloud_topic").get_parameter_value().string_value
        self.clustered_pcl_topic = self.get_parameter("clustered_pcl_topic").get_parameter_value().string_value
        self.filtered_pcl_topic = self.get_parameter("filtered_pcl_topic").get_parameter_value().string_value

        self.pointcloud = PointCloud2()

        # Create subscriptions for cluster and filter triggers, and the point cloud data
        self.cluster_trigger = self.create_subscription(
            Empty,
            self.cluster_trigger_topic,
            self.call_cluster,
            10
        )
        
        self.filter_trigger = self.create_subscription(
            Empty,
            self.filter_trigger_topic,
            self.call_filter,
            10
        )

        self.pcl_sub = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self.set_pointcloud,
            10
        )
        
        # Create service clients for clustering and filtering
        self.cluster_cli = self.create_client(PointcloudTransform, self.cluster_service)
        self.filter_cli = self.create_client(PointcloudTransform, self.filter_service)

        # Create publishers for publishing the processed point cloud data
        self.cluster_pub = self.create_publisher(
            PointCloud2,
            self.clustered_pcl_topic,
            10
        )
        
        self.filter_pub = self.create_publisher(
            PointCloud2,
            self.filtered_pcl_topic,
            10
        )        

        self.get_logger().info("Cluster client node created")

    def call_cluster(self, msg):
        # Method to call the clustering service when triggered
        self.get_logger().info("Clustering pointcloud")
        self.req = PointcloudTransform.Request()
        self.req.pointcloud = self.pointcloud

        # Asynchronously call the clustering service
        future = self.cluster_cli.call_async(self.req)
        future.add_done_callback(self.cluster_done_callback)

    def cluster_done_callback(self, future):
        # Callback method to handle the response from the clustering service
        result = future.result()      
        clustered_pcl = result.transformed_pointcloud
        self.cluster_pub.publish(clustered_pcl)
        self.get_logger().info("Pointcloud clustered and published")

    def call_filter(self, msg):
        # Method to call the filtering service when triggered
        self.get_logger().info("Filtering pointcloud")
        self.req = PointcloudTransform.Request()
        self.req.pointcloud = self.pointcloud

        # Asynchronously call the filtering service
        future = self.filter_cli.call_async(self.req)
        future.add_done_callback(self.filter_done_callback)

    def filter_done_callback(self, future):
        # Callback method to handle the response from the filtering service
        result = future.result()      
        filtered_pcl = result.transformed_pointcloud
        self.filter_pub.publish(filtered_pcl)
        self.get_logger().info("Pointcloud filtered and published")

    def set_pointcloud(self, msg):
        # Method to set the incoming point cloud data
        self.pointcloud = msg


def main(args=None):
    rclpy.init(args=args)
    
    cluster_client = ClusterClient()
    rclpy.spin(cluster_client)

    cluster_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
