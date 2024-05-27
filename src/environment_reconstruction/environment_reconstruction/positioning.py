import rclpy
import rclpy.node as node
from geometry_msgs.msg import TransformStamped, Transform, Pose, Vector3 
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage



class Locator(node.Node):

    def __init__(self):
        super().__init__('locator')
        self.get_logger().info("Locator node created")

        self.vehicle_name = "Test_car"

        # self.sub_pose = self.create_subscription(Odometry,
        #                                          f'/model/{self.vehicle_name}/odometry',
        #                                          self.broadcast_transform_odom,
        #                                          1)
        
        ground_truth_pos_sub = self.create_subscription(TFMessage, 
                                                        f'/model/{self.vehicle_name}/pose',
                                                        self.broadcast_transform, 
                                                        10)  # QoS profile depth

        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        
    # def broadcast_transform_odom(self, pose_msg):
    #     t = TransformStamped()
    #     t.header.stamp = self.get_clock().now().to_msg()
    #     t.header.frame_id = "world"
    #     t.child_frame_id = self.vehicle_name
    #     t.transform.translation = Vector3(x=pose_msg.pose.pose.position.x,
    #                                       y=pose_msg.pose.pose.position.y,
    #                                       z=pose_msg.pose.pose.position.z)
    #     t.transform.rotation = pose_msg.pose.pose.orientation
        
    #     self.tf_broadcaster.sendTransform(t)

    def broadcast_transform(self, tf_msg):
        # [self.get_logger().info(f"{tf}") for tf in tf_msg.transforms]
        # self.get_logger().info(f"publishing transforms: {[tf.header.frame_id for tf in tf_msg.transforms]}")
        
        # The transforms published by gazebo are in a different time than the clock, so the time needs to be replaced by current time
        for tf in tf_msg.transforms:
            tf.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(tf)
        # [self.tf_broadcaster.sendTransform(tf) for tf in tf_msg.transforms]
        


def main(args=None):
    rclpy.init(args=args)
    
    locator = Locator()

    rclpy.spin(locator)

    locator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
