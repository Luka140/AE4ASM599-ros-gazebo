import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # Include the sim launch file.
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('simulation'),'launch','launch_sim.launch.py')])
    )

    # Include the sim launch file.
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('lidar2map'),'launch','lidar2map.launch.py')])
    )

    # Include the sim launch file.
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('navigate'),'launch','launch_nav_avoidance.launch.py')])
    )

    return LaunchDescription([
        sim,
        lidar,
        nav
    ]) 