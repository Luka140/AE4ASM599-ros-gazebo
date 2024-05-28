import rclpy
import rclpy.node as node
import open3d as o3d
import numpy as np
from interfaces.srv import Cluster
from environment_reconstruction.utils import create_unstructured_pointcloud
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty

class Clusterer(node.Node):

    def __init__(self):
        super().__init__('clusterer')

        self.pointcloud = PointCloud2()
        self.serv_cluster = self.create_service(Cluster,
                                                'cluster_reconstruction',
                                                self.cluster)
        
        # This listener and publisher is here because the damn service just doesn't want to work properly with
        # async calls. should be removed later
        self.pcl_listener = self.create_subscription(PointCloud2,
                                                     'total_pointcloud',
                                                     self.update_pcl,
                                                     10)   
        
        self.cluster_pub = self.create_publisher(PointCloud2,
                                            'clustered_reconstruction',
                                            10)   
        
        self.cluster_trigger = self.create_subscription(Empty, 
                                                    'cluster_trigger',
                                                    self.cluster_listener,
                                                    10)        
        
        self.get_logger().info("Clusterer node created")

    
    def cluster(self, request, response):
        # Loads the pointcloud message into Open3D format to allows for transformation
        pcl_o3d = o3d.geometry.PointCloud()
        loaded_array = np.frombuffer(request.pointcloud.data, dtype=np.float32).reshape(-1, 3)
        pcl_o3d.points = o3d.utility.Vector3dVector(loaded_array)
        # self.get_logger().info(f"width of og msg: {request.pointcloud.width}")
        labels = pcl_o3d.cluster_dbscan(eps=0.05, min_points=10)

        lbls = np.expand_dims(np.asarray(labels, dtype=np.int32), 1)
        datapoints = np.hstack((np.asarray(pcl_o3d.points), lbls))
        # self.get_logger().info(f"DATAPOINTS SHAPE: {datapoints.shape}")
        pcl_msg = create_unstructured_pointcloud(datapoints, 
                                                 frame_id=request.pointcloud.header.frame_id,
                                                 time=request.pointcloud.header.stamp)
        
        response.clustered_pointcloud = pcl_msg
        self.get_logger().info("check")
        self.get_logger().info(f"response: {pcl_msg}")
        return response
    
    def cluster_listener(self, msg):
        self.get_logger().info(f"Clustering")
        pointcloud = self.pointcloud
        # Loads the pointcloud message into Open3D format to allows for transformation
        pcl_o3d = o3d.geometry.PointCloud()
        loaded_array = np.frombuffer(pointcloud.data, dtype=np.float32).reshape(-1, 3)
        pcl_o3d.points = o3d.utility.Vector3dVector(loaded_array)
        # self.get_logger().info(f"width of og msg: {request.pointcloud.width}")
        labels = pcl_o3d.cluster_dbscan(eps=0.05, min_points=10)

        lbls = np.expand_dims(np.asarray(labels, dtype=np.int8), 1)
        datapoints = np.hstack((np.asarray(pcl_o3d.points, dtype=np.float32), lbls))
        # self.get_logger().info(f"DATAPOINTS NONES: {np.count_nonzero(datapoints == None)}")
        pcl_msg = create_unstructured_pointcloud(datapoints, 
                                                 frame_id=pointcloud.header.frame_id,
                                                 time=pointcloud.header.stamp)
        
        self.cluster_pub.publish(pcl_msg)
        self.get_logger().info(f"Finished clustering")
        return 

    def update_pcl(self, msg):
        self.pointcloud = msg

def main(args=None):
    rclpy.init(args=args)

    clusterer = Clusterer()

    rclpy.spin(clusterer)

    clusterer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
