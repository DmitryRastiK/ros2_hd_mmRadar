#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for radar markers visualizer."""
    
    # Get package directory
    pkg_dir = get_package_share_directory('radar_markers')
    
    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'markers_config.yaml')
    
    # Radar markers visualizer node
    markers_node = Node(
        package='radar_markers',
        executable='radar_markers_visualizer',
        name='radar_markers_visualizer',
        output='screen',
        parameters=[config_file],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        markers_node
    ])
