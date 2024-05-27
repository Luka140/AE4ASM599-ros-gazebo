import rclpy 
import rclpy.node as node
from std_msgs.msg import Int32
from interfaces.srv import Reconstruct
from geometry_msgs.msg import Twist


class KeyboardToCapture(node.Node):
    
    def __init__(self):
        super().__init__('keyboard_to_capture')
        self.get_logger().info("capture node created")
        
        self.reconstruction_sub = self.create_subscription(Int32,
                                                           'keyboard/keypress', 
                                                           self.capture_environment, 
                                                           10)
        
        self.capture_cli = self.create_client(Reconstruct, 'reconstruct_3d_view')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
                                                 
        
    def capture_environment(self, msg):
        if msg.data == 67 or msg.data == 1677220:
            # Press C or Enter to request a reconstruction
            req = Reconstruct.Request()
            req.camera_spacing = 1.
            self.capture_cli.call_async(req)
        
        if msg.data == 83:
            # Stop movement of vehicle
            self.cmd_pub.publish(Twist())



def main(args=None):
    rclpy.init(args=args)
    
    keyboard_capture = KeyboardToCapture()

    rclpy.spin(keyboard_capture)

    keyboard_capture.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

