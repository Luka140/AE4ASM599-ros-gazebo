import rclpy 
import rclpy.node as node
from sensor_msgs.msg import PointCloud2
import numpy as np
from environment_reconstruction.stereo_reconstruction import create_pointcloud_msg


class PointcloudAggregator(node.Node):
    
    def __init__(self):
        super().__init__('pointcloud_aggregator')
        self.get_logger().info("Pointcloud aggregator node created")
        
        self.pointcloud = None
        self.reconstruction_sub = self.create_subscription(PointCloud2,
                                                           'reconstruction', 
                                                           self.reconstruction_callback, 
                                                           10)
        
        self.pointcloud_pub = self.create_publisher(PointCloud2, 'total_pointcloud', 10)



    def reconstruction_callback(self, msg):
        self.get_logger().info(f"Received a pointcloud message - adding to the total pointcloud")
        
        if self.pointcloud is None:
            self.pointcloud = msg
        else:
            new_pcl = msg
            self.pointcloud.width += new_pcl.width
            total_num_of_points = self.pointcloud.height * self.pointcloud.width
            self.pointcloud.row_step = self.pointcloud.point_step * total_num_of_points
            self.pointcloud.data.extend(new_pcl.data)
            # self.get_logger().info(f"new pointcloud data shape {self.pointcloud.data.shape}")
            # self.pointcloud.data = new_data

        self.pointcloud_pub.publish(self.pointcloud)



def main(args=None):
    rclpy.init(args=args)
    
    aggregator = PointcloudAggregator()

    rclpy.spin(aggregator)

    aggregator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
