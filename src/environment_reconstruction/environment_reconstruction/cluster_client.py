import rclpy
import rclpy.node as node
from interfaces.srv import PointcloudTransform
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from threading import Event


class ClusterClient(node.Node):

    def __init__(self):
        super().__init__('cluster_client')

        self.pointcloud = PointCloud2()

        # client_group = MutuallyExclusiveCallbackGroup()
        # trigger_group = MutuallyExclusiveCallbackGroup()

        self.cluster_trigger = self.create_subscription(Empty,
                                                    'cluster_trigger',
                                                    self.call_cluster,
                                                    10)#, callback_group=trigger_group)
        
        self.filter_trigger = self.create_subscription(Empty,
                                                       'filter_trigger',
                                                       self.call_filter,
                                                       10)

        self.pcl_sub = self.create_subscription(PointCloud2,
                                                'total_pointcloud',
                                                self.set_pointcloud,
                                                10)
        
        
        self.cluster_cli = self.create_client(PointcloudTransform, 'cluster_reconstruction')#,
                                               #callback_group=client_group)

        self.filter_cli = self.create_client(PointcloudTransform, 'filter_pcl')

        
        self.cluster_pub = self.create_publisher(PointCloud2,
                                                 'clustered_reconstruction',
                                                 10)        
        
        self.filter_pub = self.create_publisher(PointCloud2,
                                                 'filtered_reconstruction',
                                                 10)        

        self.get_logger().info("Cluster client node created")

    def call_cluster(self, msg):
        self.get_logger().info("Clustering pointcloud")
        self.req = PointcloudTransform.Request()
        self.req.pointcloud = self.pointcloud

        future = self.cluster_cli.call_async(self.req)
        future.add_done_callback(self.cluster_done_callback)

    def cluster_done_callback(self, future):
        result = future.result()      
        clustered_pcl = result.transformed_pointcloud
        self.cluster_pub.publish(clustered_pcl)
        self.get_logger().info("Pointcloud clustered and published")

    def call_filter(self, msg):
        self.get_logger().info("Filtering pointcloud")
        self.req = PointcloudTransform.Request()
        self.req.pointcloud = self.pointcloud

        future = self.filter_cli.call_async(self.req)
        future.add_done_callback(self.filter_done_callback)

    def filter_done_callback(self, future):
        result = future.result()      
        filtered_pcl = result.transformed_pointcloud
        self.filter_pub.publish(filtered_pcl)
        self.get_logger().info("Pointcloud filtered and published")


    def set_pointcloud(self, msg):
        self.pointcloud = msg


def main(args=None):
    rclpy.init(args=args)
    
    cluster_client = ClusterClient()
    rclpy.spin(cluster_client)
    # executor = MultiThreadedExecutor()
    # executor.add_node(cluster_client)
    # executor.spin()

    cluster_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
