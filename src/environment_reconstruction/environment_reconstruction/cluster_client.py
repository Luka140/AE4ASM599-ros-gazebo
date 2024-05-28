import rclpy
import rclpy.node as node
from interfaces.srv import Cluster
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty

class ClusterClient(node.Node):

    def __init__(self):
        super().__init__('cluster_client')

        self.pointcloud = PointCloud2()

        # self.trigger_sub = self.create_subscription(Empty,
        #                                             'cluster_trigger',
        #                                             self.call_cluster,
        #                                             10)
        
        self.pcl_sub = self.create_subscription(PointCloud2,
                                                'total_pointcloud',
                                                self.set_pointcloud,
                                                10)
        
        self.cluster_cli = self.create_client(Cluster,
                                               'cluster_reconstruction')
        
        self.cluster_pub = self.create_publisher(PointCloud2,
                                                 'clustered_reconstruction',
                                                 10)        

        self.get_logger().info("Cluster client node created")

    # def call_cluster(self, msg):
    #     # TODO SPIN UNTIL COMPLETE BROKEN - ALSO DOUBLE RECEIVED ADDED MESSAGES
    #     self.get_logger().info("Clustering pointcloud")
    #     req = Cluster.Request()
    #     req.pointcloud = self.pointcloud

    #     self.response = self.cluster_cli.call_async(req)
    #     rclpy.spin_until_future_complete(node=self, future=self.response, timeout_sec=20)
    #     self.get_logger().info(f"{self.response.result()}")
        
    #     clustered_pcl = self.response.clustered_pointcloud
    #     self.cluster_pub.publish(clustered_pcl)
    #     self.get_logger().info("Pointcloud clustered")

    def call_cluster(self, msg):
        """
        This service just does not want to work.....
        rewriting to publisher listener for now 
        """

        self.get_logger().info("Waiting for service to become available...")
        self.cluster_cli.wait_for_service(timeout_sec=10.0)
        self.get_logger().info("Service available. Clustering pointcloud")

        req = Cluster.Request()
        req.pointcloud = self.pointcloud

        self.response_future = self.cluster_cli.call_async(req)
        self.response_future.add_done_callback(self.handle_service_response)


    def handle_service_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Service call succeeded: {response}")
            clustered_pcl = response.clustered_pointcloud
            self.cluster_pub.publish(clustered_pcl)
            self.get_logger().info("Pointcloud clustered and published")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def set_pointcloud(self, msg):
        self.pointcloud = msg


def main(args=None):
    rclpy.init(args=args)
    
    cluster_client = ClusterClient()

    rclpy.spin(cluster_client)

    cluster_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
