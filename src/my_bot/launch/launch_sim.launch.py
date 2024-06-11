import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    package_name='my_bot'

    # Get path to gazebo parameters file
    gazebo_params_path = os.path.join(get_package_share_directory(package_name),
        'config', 
        'gazebo_params.yaml'
    )

    # Get path to gazebo world file (not used)
    gazebo_world_path = os.path.join(get_package_share_directory(package_name),
        'worlds', 
        'empty_world.sdf'
    )

    # Get path to rviz config file
    rviz_config_path = os.path.join(get_package_share_directory(package_name),
        'config',
        'view_bot.rviz'
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name),'launch','rsp.launch.py')]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )



    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'params_file': gazebo_params_path, 'gz_args': gazebo_world_path}.items()
    )

    # Run the spawner node from the ros_gz_sim package.
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'my_bot',
            '-topic', 'robot_description',
            '-z', '0.06',
        ]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
        '/model/my_bot/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
        '/world/empty_world/dynamic_pose/info@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
        '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
        ]
    )

    tf_broad = Node(
        package='gz_tf_broadcaster',
        executable='tf_broadcaster',
        output='screen',
        remappings=[
            ('/gz/model/pose', '/model/my_bot/pose')
        ]
    )

    tf_to_odom = Node(
        package='gz_tf_broadcaster',
        executable='tf_to_odom',
        output='screen',
        remappings=[
            ('/gz/model/pose', '/model/my_bot/pose')
        ]
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_path,]
    )

    # Launch
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        bridge,
        tf_broad,
        tf_to_odom,
        diff_drive_spawner,
        joint_broad_spawner,
        rviz_node
    ]) 