import rclpy
import rclpy.node as node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_msgs.msg import TFMessage
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class Locator(node.Node):

    def __init__(self):
        super().__init__('locator') 

        self.declare_parameters(namespace='',
                                parameters = [('vehicle_path', '/model/Test_car')]
        )

        self.vehicle_path = self.get_parameter('vehicle_path').value

        callback_group = ReentrantCallbackGroup()
        ground_truth_pos_sub = self.create_subscription(TFMessage, 
                                                        f'{self.vehicle_path}/pose',
                                                        self.broadcast_transform, 
                                                        5, callback_group=callback_group)  
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.get_logger().info("Locator node created")

    def broadcast_transform(self, tf_msg):
        for tf in tf_msg.transforms:
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
