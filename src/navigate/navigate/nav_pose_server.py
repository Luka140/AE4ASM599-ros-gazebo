"""
This ROS 2 action server node, named 'nav_pose_server', provides the functionality for navigating a robot to a specified pose. It listens for goal requests and cancel requests, tracks the current vehicle odometry, and executes the navigation action based on the received goals.

The node subscribes to odometry data from the topic '/input/odom' to keep track of the robot's current position and orientation.

Upon receiving a goal request from an action client, it accepts the request and begins executing the navigation action. The action server continuously publishes feedback to the action client, providing information about the progress of the navigation, such as the current position, duration, and distance to the goal.

The action server terminates the navigation action under the following conditions:
- If the goal is reached within the specified position and yaw tolerances.
- If a cancel request is received from the action client.
- If a timeout duration is exceeded.

The node publishes the reference position to the topic '/output/reference' for visualization or monitoring purposes.

Once the navigation action is completed, the action server returns the result to the action client, indicating whether the navigation was successful, canceled, or timed out.

To use this action server, an action client must send goal requests of type 'NavPose' to the 'nav_pose' action server. The client can also send cancel requests to terminate the navigation action prematurely.

The node utilizes multi-threaded execution for improved performance.

Note: This action server assumes that the robot's odometry data is provided by the '/input/odom' topic and that the goal pose is specified in the 'NavPose' action message format.
"""

import numpy as np

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from navigate_msgs.action import NavPose
from nav_msgs.msg import Odometry

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

class NavPoseServer(Node):

    def __init__(self):
        super().__init__("nav_pose_server")

        # Initialise server parameters.
        self.declare_parameters(
            namespace='',
            parameters=[
                ('position_tolerance', 0.3),
                ('yaw_tolerance', 0.1),
                ('timeout', 60.0),
            ]
        )

        # Quality of service
        qos_profile = QoSProfile(
            depth=1
        )

        # Subscribe to odometry data
        self.odometry_sub = self.create_subscription(
            Odometry, 
            '/input/odom',
            self.odom_callback,
            qos_profile)

        # Publish the reference position
        self.reference_pub = self.create_publisher(
            PoseStamped,
            '/output/reference',
            qos_profile)

        # Initialise action server
        self.action_server = ActionServer(
            self,
            NavPose,
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
        self.action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Callback to accept new goal requests"""

        self.get_logger().info("Received goal request")
        self.goal = goal_request
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Callback to accept new cancel request"""

        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT
    
    def odom_callback(self, msg):
        """Callback to keep track of the current vehicle odometry"""
        self.pose = msg.pose.pose

    async def execute_callback(self, goal_handle):
        """Execute the action, calculate the feedback and check for termination conditions"""

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
        reference_msg = PoseStamped()

        # Initialise action specific variables
        start_time = self.get_clock().now()
        goal_position = np.array([self.goal.pose.pose.position.x, self.goal.pose.pose.position.y])
        goal_yaw = euler_from_quaternion(self.goal.pose.pose.orientation.x, self.goal.pose.pose.orientation.y, self.goal.pose.pose.orientation.z, self.goal.pose.pose.orientation.w)[2]

        # Action loop, remains active throughout the action
        while True:
            # If action canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result_msg.success = NavPose.Result.CANCEL
                self.get_logger().info("Goal canceled")
                return result_msg
            
            # If action timeout
            if feedback_msg.duration > self.get_parameter('timeout').value:
                goal_handle.abort()
                result_msg.success = NavPose.Result.TIMEOUT
                self.get_logger().info("Timeout reached")
                return result_msg

            # Get current time
            current_time = self.get_clock().now()

            # Stop robot once goal reached
            reference_msg = self.goal.pose
            self.reference_pub.publish(reference_msg)

            # Calculate distance to goal.
            position = np.array([self.pose.position.x, self.pose.position.y])
            dist = np.linalg.norm(goal_position - position)

            # Calcualte yaw error
            yaw = euler_from_quaternion(self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w)[2]
            yaw_error = np.abs(goal_yaw - yaw)

            # Publish feedback
            feedback_msg.pose.header.stamp = current_time.to_msg()
            feedback_msg.pose.pose = self.pose
            feedback_msg.duration = (current_time - start_time).nanoseconds / 1e9
            feedback_msg.distance = dist
            goal_handle.publish_feedback(feedback_msg)

            # Stop if the goal is reached
            if dist < self.get_parameter('position_tolerance').value and yaw_error < self.get_parameter('yaw_tolerance').value:
                break

            # Enforce loop rate
            loop_rate.sleep()

        # Success
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