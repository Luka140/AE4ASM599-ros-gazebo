import rclpy
import rclpy.node as node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_msgs.msg import TFMessage
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class Locator(node.Node):

    def __init__(self):
        super().__init__('locator') 

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[('vehicle_path', '/model/Test_car')]
        )

        # Get the vehicle path parameter
        self.vehicle_path = self.get_parameter('vehicle_path').value

        # Create a ReentrantCallbackGroup for subscriptions
        callback_group = ReentrantCallbackGroup()

        # Create a subscription to ground truth pose topic
        ground_truth_pos_sub = self.create_subscription(
            TFMessage,  # Message type
            f'{self.vehicle_path}/pose',  # Topic name using the parameter
            self.broadcast_transform,  # Callback function
            5,  # QoS profile depth
            callback_group=callback_group  # Assign callback group
        )  

        # Create TF broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Log creation of Locator node
        self.get_logger().info("Locator node created")

    def broadcast_transform(self, tf_msg):
        # Callback function to broadcast TF transforms
        for tf in tf_msg.transforms:
            self.tf_broadcaster.sendTransform(tf)
        

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create an instance of the Locator node
    locator = Locator()

    # Create a MultiThreadedExecutor
    executor = MultiThreadedExecutor()

    # Add the Locator node to the executor
    executor.add_node(locator)

    # Spin the executor
    executor.spin()

    # Clean up
    locator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
