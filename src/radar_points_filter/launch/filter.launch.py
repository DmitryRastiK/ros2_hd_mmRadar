#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch file for the radar points filter node."""
    
    # Get the path to the config file
    config_file = os.path.join(
        get_package_share_directory('radar_points_filter'),
        'config',
        'filter_config.yaml'
    )
    
    # Declare launch argument for config file override
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the filter config file'
    )
    
    # Create the node
    filter_node = Node(
        package='radar_points_filter',
        executable='filter_node',
        name='filter_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )
    
    return LaunchDescription([
        config_file_arg,
        filter_node,
    ])

