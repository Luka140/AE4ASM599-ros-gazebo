# REMEMBER TO SOURCE ROS BEFORE BUILDING THIS

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    bridge = Node(package='ros_gz_bridge', 
                  executable='parameter_bridge', 
                  arguments=['/model/vehicle_blue/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                             '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                             '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'])

    
    # lidar_listener = Node(
    #     package="lidar_node_py",
    #     executable="lidar_listener"
    # )

    pathfinder = Node(
        package="pathfollowing",
        executable="pathfinder"
    )

    ld.add_action(bridge)
    # ld.add_action(lidar_listener)
    ld.add_action(pathfinder)
    return ld