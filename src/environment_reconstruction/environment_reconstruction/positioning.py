import rclpy
import rclpy.node as node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_msgs.msg import TFMessage



class Locator(node.Node):

    def __init__(self):
        super().__init__('locator')

        self.declare_parameters(
            namespace='',
            parameters=[
                ("vehicle_name", "Test_car"),
            ]
        )
        # Should be altered so this is assigned from the launch file
        self.vehicle_name = self.get_parameter("vehicle_name").get_parameter_value().string_value  

        ground_truth_pos_sub = self.create_subscription(TFMessage, 
                                                        f'/model/{self.vehicle_name}/pose',
                                                        self.broadcast_transform, 
                                                        10)  # QoS profile depth
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.get_logger().info("Locator node created")


    def broadcast_transform(self, tf_msg):
        """
        Listen to the message and broadcast transformations as TF transforms
        """
        for tf in tf_msg.transforms:
            self.tf_broadcaster.sendTransform(tf)
        

def main(args=None):
    rclpy.init(args=args)
    
    locator = Locator()

    rclpy.spin(locator)

    locator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
