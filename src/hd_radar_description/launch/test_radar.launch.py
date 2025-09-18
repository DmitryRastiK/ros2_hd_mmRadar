#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Common args
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Paths to included launch files
    description_share = get_package_share_directory("hd_radar_description")
    driver_share = get_package_share_directory("hd_radar_driver")

    rsp_launch = os.path.join(description_share, "launch", "rsp.launch.py")
    driver_launch = os.path.join(driver_share, "launch", "hd_radar.launch.py")

    # Include robot_state_publisher + RViz from description
    include_rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Include radar driver (without extra RViz/RQt, since RViz runs in rsp)
    include_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(driver_launch),
        launch_arguments={
            "rviz": "False",
            "rqt": "False",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time",
            ),
            include_rsp,
            include_driver,
        ]
    )


