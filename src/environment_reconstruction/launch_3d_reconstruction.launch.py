from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    bridge = Node(package='ros_gz_bridge', 
                  executable='parameter_bridge', 
                  arguments=['/camera_l@sensor_msgs/msg/Image@ignition.msgs.Image',
                             '/camera_r@sensor_msgs/msg/Image@ignition.msgs.Image',
                             '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                             '/model/Test_car/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                             '/model/Test_car/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                             '/keyboard/keypress@std_msgs/msg/Int32@ignition.msgs.Int32',
                             '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',])

    reconstructor = Node(
        package="environment_reconstruction",
        executable="stereo_reconstruction"
    )

    aggregator = Node(
        package="environment_reconstruction",
        executable="total_reconstruction"
    )

    locator = Node(
        package="environment_reconstruction",
        executable="positioning"
    )

    cluster = Node(
        package="environment_reconstruction",
        executable="cluster"
    )

    cluster_client = Node(
        package="environment_reconstruction",
        executable="cluster_client"
    )

    keyboard_capture = Node(
        package="environment_reconstruction",
        executable="keyboard_capture"
    )

    filter = Node(
        package="environment_reconstruction",
        executable="filter"
    )

    # Hacky transform. Im using PosePublisher from gazebo to publish the ground truth location of the vehicle, 
    # but this publishes it as a transform with respect to the world_demo frame instead of the world frame itself.
    # Without this there is a missing link in the TF tree. 
    static_tf_publisher = Node(package="tf2_ros",
                               executable='static_transform_publisher',
                               arguments= ["0", "0", "0", "0", "0", "0", "world", "world_demo"])
        

    ld.add_action(bridge)
    ld.add_action(static_tf_publisher)
    ld.add_action(reconstructor)
    ld.add_action(aggregator)
    ld.add_action(locator)
    ld.add_action(keyboard_capture)
    ld.add_action(cluster)
    ld.add_action(cluster_client)
    ld.add_action(filter)
    return ld