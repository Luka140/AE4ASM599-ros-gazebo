import rclpy
import rclpy.node as node
from std_msgs.msg import Int32, Empty
from interfaces.srv import Reconstruct
from geometry_msgs.msg import Twist


class KeyboardToCapture(node.Node):
    
    def __init__(self):
        super().__init__('keyboard_to_capture')
        """
        This node tracks the keyboard outputs from the 'keyboard/keypress' topic.
        If "Enter" is pressed, it does a service call to 'reconstruct_3d_view' to create a reconstruction.
        If "C" is pressed it does a service call to the cluster the pointcloud.
        If "F" is pressed it filters the pointcloud.
        If "S" is pressed, it publishes to the 'cmd_vel' topic to stop the vehicle.
        """
        
        # Declare ROS parameters for configuring service and topic names
        self.declare_parameters(
            namespace='',
            parameters=[
                ("keypress_topic", "keyboard/keypress"),  # Topic name for keyboard keypress
                ("reconstruct_service", "reconstruct_3d_view"),  # Service name for reconstruction
                ("cluster_trigger_topic", "cluster_trigger"),  # Topic name for cluster trigger
                ("filter_trigger_topic", "filter_trigger"),  # Topic name for filter trigger
                ("cmd_vel_topic", "cmd_vel"),  # Topic name for vehicle command velocity
            ]
        )

        # Get the parameters
        self.keypress_topic = self.get_parameter("keypress_topic").get_parameter_value().string_value
        self.reconstruct_service = self.get_parameter("reconstruct_service").get_parameter_value().string_value
        self.cluster_trigger_topic = self.get_parameter("cluster_trigger_topic").get_parameter_value().string_value
        self.filter_trigger_topic = self.get_parameter("filter_trigger_topic").get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value

        # Create subscriptions and publishers
        self.reconstruction_sub = self.create_subscription(
            Int32,
            self.keypress_topic, 
            self.capture_environment, 
            10
        )
        
        self.capture_cli = self.create_client(Reconstruct, self.reconstruct_service)
        self.cluster_pub = self.create_publisher(Empty, self.cluster_trigger_topic, 10)
        self.filter_pub = self.create_publisher(Empty, self.filter_trigger_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        
        self.get_logger().info("Keyboard to Capture node created")

                                                         
    def capture_environment(self, msg):
        # Capture keyboard inputs and trigger appropriate actions
        if msg.data == 16777220:
            # Press "Enter" to request a reconstruction
            req = Reconstruct.Request()
            req.camera_spacing = 1.0
            self.capture_cli.call_async(req)
        
        if msg.data == 67:
            # Press "C" to cluster the pointcloud. This sends a trigger to the request cluster client
            self.cluster_pub.publish(Empty())

        if msg.data == 70:
            # Press "F" to filter the pointcloud. This sends a trigger to the request filter client
            self.filter_pub.publish(Empty())

        if msg.data == 83:
            # Press "S" to stop movement of vehicle
            self.cmd_pub.publish(Twist())

    

def main(args=None):
    rclpy.init(args=args)
    
    keyboard_capture = KeyboardToCapture()

    rclpy.spin(keyboard_capture)

    keyboard_capture.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
