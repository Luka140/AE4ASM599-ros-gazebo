import rclpy 
import rclpy.node as node
# from common_interfaces.geometry_msgs import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class LidarSubscriber(node.Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(LaserScan,
                                                     'lidar',
                                                     self.callback,
                                                     10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
    def callback(self, msg):
        # self.get_logger().info(f"{[i for i in range(len(msg.ranges)) if msg.ranges[i]==min(msg.ranges)], min(msg.ranges), len(msg.ranges)}")
        min_range = min(msg.ranges)
        self.get_logger().info(f"Minimum range: {min_range}")

        turn_threshold_range = 2.5        
        movement_cmd = self.determine_direction(msg.ranges, turn_threshold_range)
        self.publisher_.publish(movement_cmd)

    def determine_direction(self, ranges, threshold):
        lin_velocity = 2.0
        ang_velocity = 3.0 
        movement_command = Twist()

        # No objects within threshold -> drive straight 
        if min(ranges) > threshold:
            movement_command.linear.x = lin_velocity
            movement_command.angular.z = 0.0
            return movement_command

        obstructed_angles = [i for i in range(len(ranges)) if ranges[i] <= threshold]
        if sum(obstructed_angles) / len(obstructed_angles) > len(obstructed_angles) / 2:
            # more objects to the right -> turn left
            movement_command.linear.x = 0.0
            movement_command.angular.z = -ang_velocity

        else:
            # more objects to the left -> turn right  
            movement_command.linear.x = 0.0
            movement_command.angular.z = ang_velocity
        
        return movement_command


def main(args=None):
    rclpy.init(args=args)
    
    sub_lidar = LidarSubscriber()

    rclpy.spin(sub_lidar)

    sub_lidar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
