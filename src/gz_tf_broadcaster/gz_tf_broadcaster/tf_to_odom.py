import rclpy
import rclpy.node as node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry

class TFOdom(node.Node):

    def __init__(self):
        super().__init__('tf_to_odometry')

        self.gz_tf_sub = self.create_subscription(TFMessage, 
                                            '/gz/model/pose',
                                            self.transform_callback, 
                                            10)  # QoS profile depth
        
        self.odom_pub = self.create_publisher(Odometry,
                                              '/ground_truth/odom',
                                              1,
        )
        

        
        self.get_logger().info("TF to Odom node initialised")


    def transform_callback(self, tf_msg):
        for tf in tf_msg.transforms:
            # The transforms published by gazebo are in a different time than the clock, 
            # so the time needs to be replaced by current time

            if tf.child_frame_id.split('/')[-1] == 'my_bot':
                msg = Odometry()
                #msg.header.stamp = self.get_clock().now().to_msg()
                msg.child_frame_id = 'base_link'
                msg.header.frame_id = 'empty_world'
                msg.pose.pose.position.x = tf.transform.translation.x
                msg.pose.pose.position.y = tf.transform.translation.y
                msg.pose.pose.position.z = tf.transform.translation.z

                msg.pose.pose.orientation = tf.transform.rotation

                self.odom_pub.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    
    tf_to_odom = TFOdom()

    rclpy.spin(tf_to_odom)

    tf_to_odom.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()