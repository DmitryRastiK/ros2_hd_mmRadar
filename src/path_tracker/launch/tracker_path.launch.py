#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch file for complete path tracking system.
    Launches both the Kalman filter and path tracker nodes.
    """
    
    # Get config file paths
    kalman_config_file = os.path.join(
        get_package_share_directory('path_tracker'),
        'config',
        'kalman_filter_config.yaml'
    )
    
    tracker_config_file = os.path.join(
        get_package_share_directory('path_tracker'),
        'config',
        'tracker_path_config.yaml'
    )
    
    # Declare launch arguments
    kalman_config_arg = DeclareLaunchArgument(
        'kalman_config_file',
        default_value=kalman_config_file,
        description='Path to the Kalman filter config file'
    )
    
    tracker_config_arg = DeclareLaunchArgument(
        'tracker_config_file',
        default_value=tracker_config_file,
        description='Path to the tracker path config file'
    )
    
    # Create the Kalman filter node
    kalman_filter_node = Node(
        package='path_tracker',
        executable='kalman_filter_node',
        name='kalman_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('kalman_config_file')]
    )
    
    # Create the tracker path node
    tracker_path_node = Node(
        package='path_tracker',
        executable='tracker_path_node',
        name='tracker_path_node',
        output='screen',
        parameters=[LaunchConfiguration('tracker_config_file')]
    )
    
    return LaunchDescription([
        kalman_config_arg,
        tracker_config_arg,
        kalman_filter_node,
        tracker_path_node,
    ])

