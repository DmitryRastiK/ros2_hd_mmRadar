from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hd_radar_gui',
            executable='radar_gui',
            name='hd_radar_gui',
            output='screen',
        )
    ])


