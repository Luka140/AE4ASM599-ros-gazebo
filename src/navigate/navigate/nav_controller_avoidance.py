"""
This ROS 2 node, named 'nav_controller_node', implements a PID controller for controlling the linear and angular velocities of a mobile robot based on a reference pose received from a topic.

The node subscribes to two topics:
1. '/input/pose': Receives the desired reference pose (position and orientation) for navigation.
2. '/input/odom': Receives the odometry data of the robot, including its current position and orientation.

The node computes the linear and angular velocities required to navigate the robot towards the reference pose using a PID control strategy. The computed velocities are published as TwistStamped messages to the topic '/output/cmd_vel'.

The PID controller parameters (proportional, integral, and derivative gains) and other control parameters (such as maximum velocities and tolerances) can be configured using ROS 2 parameters.

The linear velocity PID controller adjusts the forward velocity of the robot to minimize the position error between the current position and the reference position.

The angular velocity PID controller adjusts the angular velocity (yaw rate) of the robot to minimize the orientation error between the current orientation and the desired orientation (yaw) towards the reference pose.

The 'euler_from_quaternion' and 'angle_wrapping' utility functions are provided to convert quaternions to Euler angles and wrap angles to the range [-π, π], respectively.

The node utilizes the ROS 2 parameter server to store and retrieve controller parameters and the ROS 2 time API for time calculations.

To use this controller node, publishers must provide odometry data on the '/input/odom' topic, and the desired reference pose must be published on the '/input/pose' topic. The controller then computes and publishes velocity commands to control the robot's motion towards the reference pose.

Upon shutdown, the node releases its resources and shuts down the ROS 2 runtime.

Note: This controller assumes that the robot's odometry data is provided by the '/input/odom' topic and that the reference pose is specified in the 'PoseStamped' message format.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_system_default, ReliabilityPolicy

from geometry_msgs.msg import TwistStamped, Pose, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid

def euler_from_quaternion(x, y, z, w):
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

def angle_wrapping(angle):
    """
    Wraps angles to be limited to -pi to pi
    """
    if angle > np.pi:
        angle -= 2*np.pi
    elif angle < -np.pi:
        angle += 2* np.pi
    return angle
        

class NavController(Node):
    
    def __init__(self):
        super().__init__("nav_controller_avoidance_node")

        # Initialise controller parameters.
        self.declare_parameters(
            namespace='',
            parameters=[
                ('kp_linear', 1.0),
                ('ki_linear', 0.0),
                ('kd_linear', 0.2),
                ('kp_angular', 1.0),
                ('ki_angular', 0.2),
                ('kd_angular', 0.0),
                ('max_vel_linear', 2.0),
                ('max_vel_angular', 5.0),
                ('tolerance', 0.15),
                ('obstacle_avoidance_radius', 5.0),
                ('obstacle_influence_gain', 1250.0)
            ]
        )
        
        qos_profile_grid = qos_profile_system_default
        qos_profile_grid.reliability = ReliabilityPolicy.BEST_EFFORT
        # Subscribe to trajectory information
        self.reference_sub = self.create_subscription(
            PoseStamped, 
            '/input/pose', 
            self.reference_callback, 
            qos_profile_system_default
            )
        
        # Subscribe to vehicle odometry for feedback
        self.feedback_sub = self.create_subscription(
            Odometry,
            '/input/odom',
            self.feedback_callback,
            qos_profile_system_default
            )
        
        # Subscribe to the occupancy grid
        self.grid_sub = self.create_subscription(
            OccupancyGrid,
            '/input/grid',
            self.grid_callback,
            qos_profile_system_default
            )

        # Publish twist command
        self.command_vel_pub = self.create_publisher(
            TwistStamped,
            '/output/cmd_vel',
            qos_profile_system_default
            )

        # Initialise controller variables
        self.pose_ref = None
        self.occupancy_grid = None
        self.grid_info = None

        # Store PID terms
        self.delta_time = 0
        self.prev_time = self.get_clock().now().nanoseconds

        self.i_term_linear = 0
        self.d_error_linear = 0

        self.i_term_angular = 0
        self.d_error_angular = 0

    def feedback_callback(self, msg: Odometry) -> None:
        """
        Get the position and velocity feedback
        """   
        # Update delta time 
        current_time = self.get_clock().now().nanoseconds
        self.delta_time = (current_time - self.prev_time)*1e-9
        self.prev_time = current_time

        # Exit callback when there is no reference pose information known
        if self.pose_ref is None:
            return
        pose = msg.pose.pose

        # Get linear and angular command
        vel_x = self.linear_velocity(pose)
        yaw_rate = self.angular_velocity(pose)

        # Assemble command message
        command_msg = TwistStamped()
        command_msg.header.stamp = self.get_clock().now().to_msg()
        command_msg.twist.linear.x = float(vel_x)
        command_msg.twist.angular.z = float(yaw_rate)

        # Publish command
        self.command_vel_pub.publish(command_msg)

    def reference_callback(self, msg: PoseStamped) -> None:
        """
        Callback to keep track of the requested reference
        """
        self.pose_ref = msg.pose

    def grid_callback(self, msg: OccupancyGrid) -> None:
        """Callback to keep track of the latest occupancy grid"""
        self.occupancy_grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.grid_info = msg.info

    def linear_velocity(self, pose: Pose) -> None:
        """pid controller for linear control"""
        # Get parameters
        kp_linear = self.get_parameter("kp_linear").value
        ki_linear = self.get_parameter("ki_linear").value
        kd_linear = self.get_parameter("kd_linear").value

        max_vel_linear = self.get_parameter("max_vel_linear").value

        # Get current and reference position.
        position_ref = np.array([self.pose_ref.position.x, self.pose_ref.position.y])
        position = np.array([pose.position.x, pose.position.y])

        # Get obstacle avoidance repulsion
        obstacle_avoidance = self.obstacle_avoidance_vector(position)

        # Calculate position error.
        position_error = position_ref - position

        # Get current angle.
        yaw = euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)[2]

        # Transform to body frame
        delta_body = np.array([[np.cos(yaw), np.sin(yaw)],[-np.sin(yaw), np.cos(yaw)]])@position_error.T
        obstacle_body = np.array([[np.cos(yaw), np.sin(yaw)],[-np.sin(yaw), np.cos(yaw)]])@obstacle_avoidance.T

        # Command linear velocity
        error = max(min(5.0, delta_body[0]), -5.0) + obstacle_body[0]
        p_term_linear = kp_linear * error
        d_term_linear = kd_linear * (error - self.d_error_linear)/self.delta_time
        self.i_term_linear += ki_linear * error * self.delta_time
        self.d_error_linear = error

        vel_x = p_term_linear + d_term_linear + self.i_term_linear
        vel_x = max(min(vel_x, max_vel_linear), 0.0)

        return vel_x


    def angular_velocity(self, pose: Pose) -> None:
        """pid controller for angular control"""
        # Get parameters
        kp_angular = self.get_parameter("kp_angular").value
        ki_angular = self.get_parameter("ki_angular").value
        kd_angular = self.get_parameter("kd_angular").value

        max_vel_angular = self.get_parameter("max_vel_angular").value

        # Get reference.
        position_ref = np.array([self.pose_ref.position.x, self.pose_ref.position.y])

        # Get current pose.
        position = np.array([pose.position.x, pose.position.y])
        yaw = euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)[2]

        # Get obstacle avoidance repulsion
        obstacle_avoidance = self.obstacle_avoidance_vector(position)

        # Relate position error to reference angle 
        position_error = position_ref - position
        error = position_error + obstacle_avoidance
        yaw_ref = np.arctan2(error[1], error[0])

        # aim for reference orientation when close
        if np.linalg.norm(position_error) < self.get_parameter("tolerance").value:
            yaw_ref = euler_from_quaternion(self.pose_ref.orientation.x, self.pose_ref.orientation.y, self.pose_ref.orientation.z, self.pose_ref.orientation.w)[2]

        # Angle error
        yaw_error = angle_wrapping(yaw_ref - yaw)

        # Command angular velocity
        error = yaw_error
        p_term_angular = kp_angular * error
        d_term_angular = kd_angular * (error - self.d_error_angular)/self.delta_time
        self.i_term_angular += ki_angular * error * self.delta_time
        self.d_error_angular = error

        yaw_rate = p_term_angular + d_term_angular + self.i_term_angular
        yaw_rate = max(min(yaw_rate, max_vel_angular), -max_vel_angular)

        return yaw_rate
    
    def obstacle_avoidance_vector(self, position):
        if self.occupancy_grid is None:
            self.get_logger().info('No grid found')
            return np.array([0.0, 0.0])
        
        # Get parameters
        obstacle_avoidance_radius = self.get_parameter('obstacle_avoidance_radius').value
        influence_gain = self.get_parameter('obstacle_influence_gain').value

        # Convert position to grid coordinates
        grid_origin = np.array([self.grid_info.origin.position.x, self.grid_info.origin.position.y])
        grid_position = ((position - grid_origin)/self.grid_info.resolution).astype(int)

        obstacle_vector = np.array([0.0, 0.0]).astype(float)
        grid_radius = int(obstacle_avoidance_radius/self.grid_info.resolution)

        for i in range(-grid_radius, grid_radius + 1):
            for j in range(-grid_radius, grid_radius +1 ):
                grid_index = grid_position + np.array([i, j])

                if 0 <= grid_index[0] < self.grid_info.width and 0 <= grid_index[1] < self.grid_info.height:
                    self.get_logger().info(f"{self.grid_position * self.grid_info.resolution}")
                    if self.occupancy_grid[grid_index[0], grid_index[1]] >= 50:
                        self.get_logger().info("found obj")
                        dist = np.linalg.norm(grid_index - grid_position)
                        if dist > 0:
                            influence = influence_gain / dist**2
                            obstacle_vector += -influence * (grid_index - grid_position)/dist

        self.get_logger().info(f"Vector: {obstacle_vector}")
        return obstacle_vector
    
def main(args=None):
    rclpy.init(args=args)
    nav_controller = NavController()
    rclpy.spin(nav_controller)

    nav_controller.destroy()
    rclpy.shutdown()

if __name__ == "__main__":
    main()





