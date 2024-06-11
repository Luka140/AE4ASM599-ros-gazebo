import numpy as np

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from navigate_msgs.action import NavPose
from nav_msgs.msg import Odometry

class NavPoseServer(Node):

    def __init__(self):
        super().__init__("nav_pose_server")

        # Define quality of service
        qos = QoSProfile(depth=10)

        # Subscribers
        self.odometry_sub = self.create_subscription(Odometry, '/input/odom', self.odom_callback, 10)

        # Publishers
        self.command_pub = self.create_publisher(TwistStamped, '/output/cmd_vel', 10)

        # Initialise action server
        self._action_server = ActionServer(
            self,
            NavLine,
            'nav_pose',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Initialise vehicle variables
        self.pose = None
        self.goal = NavPose.Goal()

        self.get_logger().info("NavPose action has been initialised.")
  
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        # Accepts or rejects a client request to begin an action.
        self.get_logger().info("Received goal request")
        self.goal = goal_request
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        # Accepts or rejects a client request to cancel an action.
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT
    
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
    
    def odom_callback(self, msg):
        self.pose = msg.pose.pose

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")
        loop_rate = self.create_rate(10, self.get_clock())

        # Initialise result message
        result_msg = NavPose.Result()

        # Initialise feedback message
        feedback_msg = NavPose.Feedback()
        feedback_msg.pose.header.frame_id = "odom"
        feedback_msg.duration = 0.0
        feedback_msg.distance = np.inf

        # Initialise output command message
        command_msg = TwistStamped()

        # Initialise action specific variables
        start_time = self.get_clock().now()
        goal_position = np.array([self.goal.pose.pose.position.x, self.goal.pose.pose.position.y])
        goal_yaw = self.euler_from_quaternion(self.goal.pose.pose.orientation.x, self.goal.pose.pose.orientation.y, self.goal.pose.pose.orientation.z, self.goal.pose.pose.orientation.w)[2]

        while True:

            # If action canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result_msg.success = NavPose.Result.CANCEL
                self.get_logger().info("Goal canceled")
                return result_msg
            
            # If action timeout
            if feedback_msg.duration > self.timeout:
                goal_handle.abort()
                result_msg.success = NavPose.Result.TIMEOUT
                self.get_logger().info("Timeout reached")
                return result_msg

            # Get current position and heading from odometry data
            position = np.array([self.pose.position.x, self.pose.position.y])

            # Transform to body frame
            delta_body = np.array([[np.cos(yaw), np.sin(yaw)],[-np.sin(yaw), np.cos(yaw)]])@delta

            # Calcualte lateral velocity using p controller

            vel_x = self.proportional_gain_x * delta_body[0]
            vel_x = max(min(vel_x, self.max_vel_x), self.min_vel_x)

            delta_yaw = np.arctan2(delta_body[1], delta_body[0])

            if dist < self.rotate_dist_threshold:
                delta_yaw = goal_yaw - yaw
            if delta_yaw > np.pi:
                delta_yaw -= 2*np.pi
            elif delta_yaw < -np.pi:
                delta_yaw += 2*np.pi

            vel_yaw = self.proportional_gain_yaw * delta_yaw

            # Get current time
            current_time = self.get_clock().now()

            # Publish commands
            command_msg.header.stamp = current_time.to_msg()
            command_msg.twist.linear.x = -vel_x
            command_msg.twist.angular.z = vel_yaw
            self.command_pub.publish(command_msg)

            # Publish feedback
            feedback_msg.pose.header.stamp = current_time.to_msg()
            feedback_msg.pose.pose = self.pose
            feedback_msg.duration = (current_time - start_time).nanoseconds / 1e9
            feedback_msg.distance = dist
            goal_handle.publish_feedback(feedback_msg)

            # If goal reached
            if dist < self.position_tolerance and abs(delta_yaw) < np.deg2rad(self.yaw_tolerance):
                break

            # Enforce loop rate
            loop_rate.sleep()

        # Stop robot once goal reached
        command_msg.header.stamp = self.get_clock().now().to_msg()
        command_msg.twist.linear.x = 0.0
        command_msg.twist.angular.z = 0.0
        self.command_pub.publish(command_msg)

        goal_handle.succeed()
        result_msg.success = NavPose.Result.SUCCESS

        self.get_logger().info(f"Returning result: {result_msg}")
        return result_msg
    


def main(args=None):
    rclpy.init(args=args)

    nav_pose_action_server = NavPoseServer()

    executor = MultiThreadedExecutor()

    rclpy.spin(nav_pose_action_server, executor=executor)

    nav_pose_action_server.destroy()
    rclpy.shutdown()

if __name__ == "__main__":
    main()