#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


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
            "rviz": "True",
            "rqt": "True",
        }.items(),
    )

    SLAM_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("hd_radar_description"),
                "config",
                "mapper_params_online_async.yaml",
            ),
            {"use_sim_time": True},
            {
                "scan_queue_size": 200,
                "transform_timeout": 0.5,
                "tf_buffer_duration": 60.0,
            },
        ],
    )

    pointcloud_to_laserscan_node_0s = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan_0s",
        output="screen",
        remappings=[("cloud_in", "/hd_radar_0/points/static"), ("scan", "/scan_0")],
        parameters=[
            {
                "queue_size": 200,  # Увеличьте размер очереди
                "target_frame": "base_link",  # ← совпадает с frame_id облака!
                "transform_tolerance": 0.2,
                "min_height": -10.0,  ####
                "max_height": 10.0,  ####
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.01745,  # увеличил плотность скана
                "scan_time": 0.1,
                "range_min": 1.0,
                "range_max": 100.0,  ####
                "use_inf": True,
                "inf_epsilon": 1.0,
            }
        ],
    )
    pointcloud_to_laserscan_node_0d = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan_0d",
        output="screen",
        remappings=[("cloud_in", "/hd_radar_0/points/dynamic"), ("scan", "/scan_0")],
        parameters=[
            {
                "queue_size": 200,  # Увеличьте размер очереди
                "target_frame": "base_link",  # ← совпадает с frame_id облака!
                "transform_tolerance": 0.2,
                "min_height": -10.0,  ####
                "max_height": 10.0,  ####
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.01745,  # увеличил плотность скана
                "scan_time": 0.1,
                "range_min": 1.0,
                "range_max": 100.0,  ####
                "use_inf": True,
                "inf_epsilon": 1.0,
            }
        ],
    )

    pointcloud_to_laserscan_node_1s = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan_1s",
        output="screen",
        remappings=[("cloud_in", "/hd_radar_1/points/static"), ("scan", "/scan_0")],
        parameters=[
            {
                "queue_size": 200,  # Увеличьте размер очереди
                "target_frame": "base_link",  # ← совпадает с frame_id облака!
                "transform_tolerance": 0.2,
                "min_height": -10.0,  ####
                "max_height": 10.0,  ####
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.01745,  # увеличил плотность скана
                "scan_time": 0.1,
                "range_min": 1.0,
                "range_max": 100.0,  ####
                "use_inf": True,
                "inf_epsilon": 1.0,
            }
        ],
    )

    pointcloud_to_laserscan_node_1d = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan_1d",
        output="screen",
        remappings=[("cloud_in", "/hd_radar_1/points/dynamic"), ("scan", "/scan_0")],
        parameters=[
            {
                "queue_size": 200,  # Увеличьте размер очереди
                "target_frame": "base_link",  # ← совпадает с frame_id облака!
                "transform_tolerance": 0.2,
                "min_height": -10.0,  ####
                "max_height": 10.0,  ####
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.01745,  # увеличил плотность скана
                "scan_time": 0.1,
                "range_min": 1.0,
                "range_max": 100.0,  ####
                "use_inf": True,
                "inf_epsilon": 1.0,
            }
        ],
    )

    qos_bridge_node = Node(
        package="qos_bridge",
        executable="qos_bridge_node",
        name="qos_bridge_node",
        output="screen",
        parameters=[
            {
                "input_topic": "/scan_0",
                "output_topic": "/scan_0_reliable",
            }
        ],
    )

    # Добавьте этот нод для эмуляции odometry (если нет реальной одометрии)
    fake_odom_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
        name="fake_odom_publisher",
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time",
            ),
            # include_rsp,
            include_driver,
            SLAM_toolbox,
            pointcloud_to_laserscan_node_0s,
            pointcloud_to_laserscan_node_1s,
            pointcloud_to_laserscan_node_0d,
            pointcloud_to_laserscan_node_1d,
            qos_bridge_node,
            fake_odom_node,
        ]
    )
