import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    package_name='simulation'

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

    # Include the RSP (robot state publisher) launch file.
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
            '-name', 'robot',
            '-topic', 'robot_description',
            '-z', '0.06',
        ]
    )

    # Run the bridge node from the ros_gz_bridge package.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
        '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',   # Needed to know sim_time
        ]
    )

    # Spawn the differenctial drive node from the controller_manager package.
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
    )

    # Spawn the joint state broadcaster node from the controller_manager package.
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad']
    )

    # Start rviz.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_path,],
    )

    # Launch
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        bridge,
        diff_drive_spawner,
        joint_broad_spawner,
        rviz_node
    ]) 