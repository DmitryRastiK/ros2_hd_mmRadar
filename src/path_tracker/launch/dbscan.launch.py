#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch file for the DBSCAN clustering node."""
    
    # Get the path to the config file
    config_file = os.path.join(
        get_package_share_directory('path_tracker'),
        'config',
        'dbscan_config.yaml'
    )
    
    # Declare launch argument for config file override
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the DBSCAN config file'
    )
    
    # Create the node
    dbscan_node = Node(
        package='path_tracker',
        executable='DBSCAN_node',
        name='dbscan_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )
    
    return LaunchDescription([
        config_file_arg,
        dbscan_node,
    ])

