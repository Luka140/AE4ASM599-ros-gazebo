from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    """
    This launch file launches the nodes from the 'pointcloud_to_laserscan' and 'pointcloud_to_grid' packages, and configures them such
    that they interact correctly. 
    """

    ld = LaunchDescription()

    bridge = Node(package='ros_gz_bridge', 
                  executable='parameter_bridge', 
                  arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'])

    # Topics over which to interface
    lidar_topic     = '/lidar'      # This is the lidar broadcast topic which is an input to the lidar to pointcloud node
    pointcloud_topic = '/cloud'     # Output of the lidar to pointcloud node and input to the pointcloud to map node

    # The node for converting lidar data to pointcloud2
    pcl2scan = Node(package='pointcloud_to_laserscan',
                    executable='laserscan_to_pointcloud_node',
                    name='laserscan_to_pointcloud',
                    remappings=[('scan_in', lidar_topic),
                                ('cloud', pointcloud_topic)],
                    parameters=[{'transform_tolerance': 0.01}]
                    )
    
    # The node for converting pointclouds to a gridmap
    pcl2grid = Node(
        package="pointcloud_to_occupancy_grid",
        executable="pointcloud_to_grid_node",
        name="pc2_to_grid",
        output="screen",
        parameters=[
            {'cloud_in_topic': pointcloud_topic},
            {'qos_best_effort': "True"},    
            ('cloud_in_topic', pointcloud_topic),
            ("lidar_range_lim", "30"),
        ]
    )
 
    ld.add_action(pcl2scan)
    ld.add_action(pcl2grid)
    ld.add_action(bridge)
    return ld
