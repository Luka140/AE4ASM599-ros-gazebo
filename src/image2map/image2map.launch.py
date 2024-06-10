from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import launch_ros


def generate_launch_description():

    pkg = "image2map"
    ld = LaunchDescription()
    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    bridge = Node(package='ros_gz_bridge', 
                  executable='parameter_bridge', 
                  arguments=['/camera_l@sensor_msgs/msg/Image@ignition.msgs.Image',
                             '/camera_r@sensor_msgs/msg/Image@ignition.msgs.Image',
                             '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                             '/model/Test_car/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                             '/model/Test_car/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                             '/keyboard/keypress@std_msgs/msg/Int32@ignition.msgs.Int32',
                             '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                             '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock'])

    reconstructor = Node(
        package=pkg,
        executable="stereo_reconstruction",
        # parameters=[{'use_sim_time': use_sim_time}]
    )

    locator = Node(
        package=pkg,
        executable="positioning",
        # parameters=[{'use_sim_time': use_sim_time}]
    )

    # keyboard_capture = Node(
    #     package=pkg,
    #     executable="keyboard_capture"
    # )

    filter = Node(
        package=pkg,
        executable="filter"
    )
 
    coordinator = Node(
        package=pkg,
        executable="coordinator",
        # parameters=[{'use_sim_time': use_sim_time}]
    )

    mapper = Node(
        package=pkg,
        executable="mapper",
        # parameters=[{'use_sim_time': use_sim_time}]
    )

    # Hacky transform. Im using PosePublisher from gazebo to publish the ground truth location of the vehicle, 
    # but this publishes it as a transform with respect to the world_demo frame instead of the world frame itself.
    # Without this there is a missing link in the TF tree. 
    static_tf_publisher = Node(package="tf2_ros",
                               executable='static_transform_publisher',
                               arguments= ["0", "0", "0", "0", "0", "0", "world", "world_demo"],
                            #    parameters=[{'use_sim_time': use_sim_time}]
                               )

#     rviz = Node(package="rviz2",
#                 executable="rviz2",
#                 condition=IfCondition(LaunchConfiguration("rviz")),
#                 arguments=["-d", path.join(, "config", "slam_and_nav_v1.rviz")]
# )
        
    ld.add_action(bridge)
    ld.add_action(static_tf_publisher)
    ld.add_action(reconstructor)
    ld.add_action(locator)
    # ld.add_action(keyboard_capture)
    ld.add_action(filter)
    ld.add_action(coordinator)
    ld.add_action(mapper)
    return ld
