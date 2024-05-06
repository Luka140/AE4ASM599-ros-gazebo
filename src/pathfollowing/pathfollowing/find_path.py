import numpy as np
import rclpy 
import rclpy.node as node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry


class Pathfinder(node.Node):
    def __init__(self):
        super().__init__('pathfinder')
        self.get_logger().info("Pathfinder node created")
        self.linear_vel = 2.0
        self.rotation_vel = 2.0
        self.width = 1.5
        self.objective_pos = None
        self.obstructed = False
        self.lidar_data = None 
        self.position_sub = self.create_subscription(Odometry,
                                                     '/model/vehicle_blue/odometry',
                                                     self.movement_cmd,
                                                     10)
        
        self.objective_sub = self.create_subscription(Pose,
                                                    'waypoint',
                                                    self.set_objective,
                                                    10)

        self.lidar_sub = self.create_subscription(LaserScan,
                                                'lidar',
                                                self.set_lidar,
                                                10)
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    
    def set_objective(self, position):
        self.objective_pos = position
        self.get_logger().info(f"Set waypoint to {position.position}")

    def set_lidar(self, lidar_msg):
        self.lidar_data = lidar_msg

    def movement_cmd(self, current_position):
        # self.get_logger().info(f"Current position: {current_position.pose.pose.position}")
        if self.objective_pos is None:
            return 
        
        command = Twist()
        
        obj_pos = self.objective_pos.position
        cur_or = current_position.pose.pose.orientation
        curr_pos = current_position.pose.pose.position

        if sum([(obj_pos.x-curr_pos.x)**2, (obj_pos.y-curr_pos.y)**2, (obj_pos.z-curr_pos.z)**2]) < 0.1:
            self.get_logger().info(f"Arrived at {curr_pos.x} {curr_pos.y} {curr_pos.z}")
            command.linear.x = 0.
            command.linear.z = 0. 
            self.publisher_.publish(command)
            return 

        _,__, yaw_orientation = self.euler_from_quaternion(cur_or.x, cur_or.y, cur_or.z, cur_or.w)

        waypoint_vector = np.array([obj_pos.x - curr_pos.x, obj_pos.y - curr_pos.y, obj_pos.z - obj_pos.z])
        angle_misalignment = np.arctan2(waypoint_vector[1], waypoint_vector[0]) - yaw_orientation

        self.obstructed = False
        if self.lidar_data is not None:
            self.obstructed, transverse_locs = self.check_obstruction()
        else:
            transverse_locs = np.zeros((1,3))

        if not self.obstructed:
            self.get_logger().info(f"Angle misalignment: {angle_misalignment}")

            if abs(angle_misalignment) > 0.1 and not (abs(transverse_locs) < (self.width*2)).any():
                command.linear.x = 0.
                command.angular.z = np.sign(angle_misalignment) * self.rotation_vel * min(0.2, abs(angle_misalignment))
            else:
                command.linear.x = self.linear_vel
                command.angular.z = 0.
            
            self.publisher_.publish(command)
            return 

        # Turn away from obstructions. The negative exp makes obstructions near the middle more important than those towards the edges
        turn_dir = np.sign(np.mean(np.exp(-abs(transverse_locs)) * np.sign(transverse_locs)))
        command.linear.x = 0.
        command.angular.z = -turn_dir * self.rotation_vel / 5 
        self.publisher_.publish(command)

    def check_obstruction(self):
        # self.get_logger().info(f"Lidar data ranges:{self.lidar_data.angle_min} {self.lidar_data.angle_increment}")
        lidar_angles = np.array([(self.lidar_data.angle_min + self.lidar_data.angle_increment * i, self.lidar_data.ranges[i]) for i in range(len(self.lidar_data.ranges))])
        transverse_locs = (np.sin(lidar_angles[:,0]) + 0.001 )* lidar_angles[:,1] # Additional 0.001 to prevent ranges at infinity but angle 90 to be small 

        if any(abs(transverse_locs) < self.width):
            return True, transverse_locs
        return False, transverse_locs
 
    def euler_from_quaternion(self, x, y, z, w):
            """
            Convert a quaternion into euler angles (roll, pitch, yaw)
            roll is rotation around x in radians (counterclockwise)
            pitch is rotation around y in radians (counterclockwise)
            yaw is rotation around z in radians (counterclockwise)
            """
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = np.arctan2(t0, t1)
        
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = np.arcsin(t2)
        
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = np.arctan2(t3, t4)
        
            return roll_x, pitch_y, yaw_z # in radians
    
    def angle_diff(v1, v2):
        v1_u = v1 / np.linalg.norm(v1)
        v2_u = v2 / np.linalg.norm(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))    
    

def main(args=None):
    rclpy.init(args=args)
    
    pathfinder = Pathfinder()

    rclpy.spin(pathfinder)

    pathfinder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()