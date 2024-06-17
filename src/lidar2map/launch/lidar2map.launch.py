from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():

    ld = LaunchDescription()

    # Topics over which to interface
    lidar_topic = '/lidar'                  # This is the lidar broadcast topic which is an input to the lidar to pointcloud node
    pointcloud_topic = '/cloud'             # Output of the lidar to pointcloud node and input to the pointcloud to map node
    height_map_topic = "/height_map"        # Output of topic of the pointcloud to map node
    intensity_map_topic = "/intensity_map"  # Output of topic of the pointcloud to map node

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
        package="pointcloud_to_grid",
        executable="pointcloud_to_grid_node",
        name="pc2_to_grid",
        output="screen",
        parameters=[
            {'cloud_in_topic': pointcloud_topic},
            {'maph_topic_name': height_map_topic},
            {'mapi_topic_name': intensity_map_topic},
            {'cell_size': 0.1},
            {'qos_best_effort': "True"},    
        ]
    )

    
    ld.add_action(pcl2scan)
    ld.add_action(pcl2grid)
    return ld
