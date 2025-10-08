#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    # Get package directory
    pkg_dir = get_package_share_directory('radar_filters')
    
    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'filter_config.yaml')
    
    # Radar filter node
    radar_filter_node = Node(
        package='radar_filters',
        executable='radar_point_cloud_filter',
        name='radar_point_cloud_filter',
        output='screen',
        parameters=[config_file],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        radar_filter_node
    ])
