import rclpy
import rclpy.node as node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_msgs.msg import TFMessage

class TFBroadcaster(node.Node):

    def __init__(self):
        super().__init__('tf_broadcaster')

        ground_truth_pos_sub = self.create_subscription(TFMessage, 
                                                        '/gz/model/pose',
                                                        self.broadcast_transform, 
                                                        10)  # QoS profile depth
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.get_logger().info("TF broadcaster node initialised")


    def broadcast_transform(self, tf_msg):
        for tf in tf_msg.transforms:

            # The transforms published by gazebo are in a different time than the clock, 
            # so the time needs to be replaced by current time
            #tf.header.stamp = self.get_clock().now().to_msg()
            tf.child_frame_id = tf.child_frame_id.split('/')[-1] 
            self.tf_broadcaster.sendTransform(tf)
        

def main(args=None):
    rclpy.init(args=args)
    
    tf_broadcaster = TFBroadcaster()

    rclpy.spin(tf_broadcaster)

    tf_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()