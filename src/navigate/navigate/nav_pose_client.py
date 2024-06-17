"""
This Python script implements a ROS 2 action client node for navigating to a specified pose. 

It creates a ROS 2 node named 'nav_pose_client' that listens for PoseStamped messages on the topic '/input/goal_pose'. 
When a new goal pose is received, it cancels any previous unfinished action and sends a new goal message to the 
'action server' named 'nav_pose'. The QoS profile used ensures reliable message delivery and keeps a history of the 
last 10 messages. 

Upon receiving feedback during the navigation action, it logs the remaining distance to the goal. Once the action 
server responds to the goal request, the node logs whether the goal was accepted or rejected. If accepted, it 
waits for the result and logs the outcome.

To use this script, ensure that the appropriate action server ('nav_pose') is running and available to handle 
navigation requests.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from navigate_msgs.action import NavPose

class NavPoseClient(Node):

    def __init__(self):
        super().__init__('nav_pose_client')

        # Quality of service
        qos_profile = QoSProfile(
            depth=1
        )

        # Subscribe to the desired goal pose
        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            '/input/goal_pose', 
            self.goal_pose_callback, 
            qos_profile
        )

        # Initialise Action Client
        self.action_client = ActionClient(
            self, 
            NavPose, 
            'nav_pose'
        )

        # Goal handle
        self.goal_handle = None

        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()

        # Log succesfull initialisation of the client
        self.get_logger().info("Nav Pose client has been initialised.")

    def goal_pose_callback(self, msg):
        """Callback for starting new action"""
        
        # Cancel previous unfinished action
        if self.goal_handle:
            self.action_client._cancel_goal_async(self.goal_handle)

        self.get_logger().info("Received goal position")

        # Create new goal message
        goal_msg = NavPose.Goal()
        goal_msg.pose = msg

        # Add the done and feedback callback
        future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback_msg):
        """Callback for logging feedback during action"""

        feedback = feedback_msg.feedback
        self.get_logger().info(f"Remaining distance: {feedback.distance}")

    def goal_response_callback(self, future):
        """Callback for handling goal future"""
        self.goal_handle = future.result()

        # Check if goal is rejected
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')

        # Add done callback 
        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Callback for logging result"""

        result = future.result().result
        self.get_logger().info(f'Result {result}')

def main(args=None):
    rclpy.init(args=args)

    nav_pose_action_client = NavPoseClient()

    rclpy.spin(nav_pose_action_client)

    nav_pose_action_client.destroy()
    rclpy.shutdown()

if __name__ == "__main__":
    main()