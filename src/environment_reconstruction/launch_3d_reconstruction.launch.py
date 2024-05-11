# REMEMBER TO SOURCE ROS BEFORE BUILDING THIS

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    bridge = Node(package='ros_gz_bridge', 
                  executable='parameter_bridge', 
                  arguments=['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                             '/camera_l@sensor_msgs/msg/Image@ignition.msgs.Image',
                             '/camera_r@sensor_msgs/msg/Image@ignition.msgs.Image',
                             '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'])

    reconstructor = Node(
        package="environment_reconstruction",
        executable="stereo_reconstruction"
    )

    ld.add_action(bridge)
    ld.add_action(reconstructor)
    return ld