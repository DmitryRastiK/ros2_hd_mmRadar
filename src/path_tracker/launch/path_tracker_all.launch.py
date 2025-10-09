#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    """
    Unified launch file for all path_tracker nodes.
    Allows selective launching of individual nodes via arguments.
    """
    
    # Get config file paths
    package_share = get_package_share_directory('path_tracker')
    
    kalman_config = os.path.join(package_share, 'config', 'kalman_filter_config.yaml')
    tracker_config = os.path.join(package_share, 'config', 'tracker_path_config.yaml')
    corridor_config = os.path.join(package_share, 'config', 'corridor_config.yaml')
    dbscan_config = os.path.join(package_share, 'config', 'dbscan_config.yaml')
    rviz_config = os.path.join(package_share, 'rviz', 'corridor_walking_bag.rviz')
    
    # Declare launch arguments for enabling/disabling nodes
    launch_kalman_arg = DeclareLaunchArgument(
        'launch_kalman',
        default_value='true',
        description='Launch Kalman filter node'
    )
    
    launch_tracker_arg = DeclareLaunchArgument(
        'launch_tracker',
        default_value='true',
        description='Launch tracker path node'
    )
    
    launch_corridor_arg = DeclareLaunchArgument(
        'launch_corridor',
        default_value='false',
        description='Launch corridor node'
    )
    
    launch_dbscan_arg = DeclareLaunchArgument(
        'launch_dbscan',
        default_value='true',
        description='Launch DBSCAN clustering node'
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2 visualization'
    )
    
    # Declare arguments for config file overrides
    kalman_config_arg = DeclareLaunchArgument(
        'kalman_config',
        default_value=kalman_config,
        description='Path to Kalman filter config file'
    )
    
    tracker_config_arg = DeclareLaunchArgument(
        'tracker_config',
        default_value=tracker_config,
        description='Path to tracker path config file'
    )
    
    corridor_config_arg = DeclareLaunchArgument(
        'corridor_config',
        default_value=corridor_config,
        description='Path to corridor config file'
    )
    
    dbscan_config_arg = DeclareLaunchArgument(
        'dbscan_config',
        default_value=dbscan_config,
        description='Path to DBSCAN config file'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config,
        description='Path to RViz config file'
    )
    
    # Create nodes
    kalman_filter_node = Node(
        package='path_tracker',
        executable='kalman_filter_node',
        name='kalman_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('kalman_config')],
        condition=IfCondition(LaunchConfiguration('launch_kalman'))
    )
    
    tracker_path_node = Node(
        package='path_tracker',
        executable='tracker_path_node',
        name='tracker_path_node',
        output='screen',
        parameters=[LaunchConfiguration('tracker_config')],
        condition=IfCondition(LaunchConfiguration('launch_tracker'))
    )
    
    corridor_node = Node(
        package='path_tracker',
        executable='corridor_node',
        name='corridor_node',
        output='screen',
        parameters=[LaunchConfiguration('corridor_config')],
        condition=IfCondition(LaunchConfiguration('launch_corridor'))
    )
    
    dbscan_node = Node(
        package='path_tracker',
        executable='DBSCAN_node',
        name='dbscan_node',
        output='screen',
        parameters=[LaunchConfiguration('dbscan_config')],
        condition=IfCondition(LaunchConfiguration('launch_dbscan'))
    )
    
    # Static transform publisher for base_link frame
    # This publishes a static transform from map to base_link
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )
    
    return LaunchDescription([
        # Launch arguments for enabling/disabling nodes
        launch_kalman_arg,
        launch_tracker_arg,
        launch_corridor_arg,
        launch_dbscan_arg,
        launch_rviz_arg,
        
        # Config file arguments
        kalman_config_arg,
        tracker_config_arg,
        corridor_config_arg,
        dbscan_config_arg,
        rviz_config_arg,
        
        # Nodes
        static_tf_node,
        kalman_filter_node,
        tracker_path_node,
        corridor_node,
        dbscan_node,
        rviz_node,
    ])

