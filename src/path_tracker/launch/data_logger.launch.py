#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch file for the data logger node."""
    
    # Get the path to the config file
    config_file = os.path.join(
        get_package_share_directory('path_tracker'),
        'config',
        'data_logger_config.yaml'
    )
    
    # Declare launch argument for config file override
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the data logger config file'
    )
    
    # Create the node
    data_logger_node = Node(
        package='path_tracker',
        executable='data_logger_node',
        name='data_logger_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )
    
    return LaunchDescription([
        config_file_arg,
        data_logger_node,
    ])


