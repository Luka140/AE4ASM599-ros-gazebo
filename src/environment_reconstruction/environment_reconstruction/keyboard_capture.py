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
        If either "C" or "Enter" is pressed, the does a service call to 'reconstruct_3d_view' to create a reconstruction
        If "S" is pressed, it publishes to the 'cmd_vel' topic to stop the vehicle. 
        
        """
        self.reconstruction_sub = self.create_subscription(Int32,
                                                           'keyboard/keypress', 
                                                           self.capture_environment, 
                                                           10)
        
        self.capture_cli = self.create_client(Reconstruct, 'reconstruct_3d_view')
        self.cluster_pub = self.create_publisher(Empty, 'cluster_trigger', 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("capture node created")

                                                         
    def capture_environment(self, msg):
        if  msg.data == 16777220:
            # Press "Enter" to request a reconstruction
            req = Reconstruct.Request()
            req.camera_spacing = 1.
            self.capture_cli.call_async(req)
        
        if msg.data == 67:
            # Press "C" to cluster the pointcloud. This sends a trigger to the request cluster client
            self.cluster_pub.publish(Empty())


        if msg.data == 83:
            # Stop movement of vehicle by pressing "s"
            self.cmd_pub.publish(Twist())

    

def main(args=None):
    rclpy.init(args=args)
    
    keyboard_capture = KeyboardToCapture()

    rclpy.spin(keyboard_capture)

    keyboard_capture.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

