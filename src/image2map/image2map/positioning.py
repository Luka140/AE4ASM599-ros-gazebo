import rclpy
import rclpy.node as node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_msgs.msg import TFMessage
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from threading import Event



class Locator(node.Node):

    def __init__(self):
        super().__init__('locator')

        # Should be altered so this is assigned from the launch file
        self.vehicle_name = "Test_car"     

        callback_group = ReentrantCallbackGroup()
        ground_truth_pos_sub = self.create_subscription(TFMessage, 
                                                        f'/model/{self.vehicle_name}/pose',
                                                        self.broadcast_transform, 
                                                        5, callback_group=callback_group)  # QoS profile depth
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.get_logger().info("Locator node created")


    def broadcast_transform(self, tf_msg):
        # time = self.get_clock().now().to_msg()
        for tf in tf_msg.transforms:
            # The transforms published by gazebo are in a different time than the clock, 
            # so the time needs to be replaced by current time
            # tf.header.stamp = time 
            self.tf_broadcaster.sendTransform(tf)
        

def main(args=None):
    rclpy.init(args=args)
    
    locator = Locator()
    executor = MultiThreadedExecutor()
    executor.add_node(locator)
    executor.spin()

    locator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
