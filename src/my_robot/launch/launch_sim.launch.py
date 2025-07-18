import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name = "my_robot"  # <--- CHANGE ME

    world_path = os.path.join(
        get_package_share_directory(package_name),
        "worlds",
        "my_city.world",
    )

    # 2. Объявляем параметр запуска
    declare_world_arg = DeclareLaunchArgument(
        "world",
        default_value=world_path,  # реальный путь к файлу
        description="Path to Gazebo world file (.world format)",  # просто подсказка
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name),
                    "launch",
                    "rsp.launch.py",
                )
            ]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                )
            ]
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "verbose": "true",
            "pause": "false",
        }.items(),
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "my_bot"],
        output="screen",
    )

    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        remappings=[("cloud_in", "/hd_radar_0/points/dynamic"), ("scan", "/scan")],
        parameters=[
            {
                "target_frame": "radar_mount",  # ← совпадает с frame_id облака!
                "transform_tolerance": 0.01,
                "min_height": -1.0,
                "max_height": 1.0,
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.01745,  # увеличил плотность скана
                "scan_time": 0.1,
                "range_min": 1.0,
                "range_max": 20.0,
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
    )

    return LaunchDescription(
        [
            declare_world_arg,  # Должен быть первым
            rsp,
            gazebo,
            spawn_entity,
            pointcloud_to_laserscan_node,
            qos_bridge_node,
        ]
    )
