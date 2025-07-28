from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    lidar_topic = "/hd_radar_0/points/dynamic"
    frame_id = "radar_mount"
    voxel_size = float(LaunchConfiguration("voxel_size").perform(context))
    localization = LaunchConfiguration("localization").perform(context) in [
        "true",
        "True",
    ]

    max_corr_dist = voxel_size * 10.0

    common_params = {
        "use_sim_time": True,  # всегда true
        "frame_id": frame_id,
        "qos": 1,
        "wait_for_transform": 0.2,
        "Icp/PointToPlane": "true",
        "Icp/Iterations": "10",
        "Icp/VoxelSize": str(voxel_size),
        "Icp/Epsilon": "0.001",
        "Icp/PointToPlaneK": "20",
        "Icp/PointToPlaneRadius": "0",
        "Icp/MaxTranslation": "3",
        "Icp/MaxCorrespondenceDistance": str(max_corr_dist),
        "Icp/Strategy": "1",
        "Icp/OutlierRatio": "0.7",
    }

    odom_params = {
        "expected_update_rate": 10.0,
        "odom_frame_id": "icp_odom",
        "Icp/CorrespondenceRatio": "0.01",
        "Odom/ScanKeyFrameThr": "0.4",
        "OdomF2M/ScanSubtractRadius": str(voxel_size),
        "OdomF2M/ScanMaxSize": "15000",
        "OdomF2M/BundleAdjustment": "false",
    }

    slam_params = {
        "use_sim_time": True,
        "subscribe_depth": False,
        "subscribe_rgb": False,
        "subscribe_odom_info": True,
        "subscribe_scan_cloud": True,
        "map_frame_id": "map",
        "odom_sensor_sync": True,
        "RGBD/ProximityMaxGraphDepth": "0",
        "RGBD/ProximityPathMaxNeighbors": "1",
        "RGBD/AngularUpdate": "0.05",
        "RGBD/LinearUpdate": "0.05",
        "RGBD/CreateOccupancyGrid": "false",
        "Mem/NotLinkedNodesKept": "false",
        "Mem/STMSize": "30",
        "Reg/Strategy": "1",
        "Icp/CorrespondenceRatio": "0.2",
    }

    # Если localization включена (локализация, не SLAM), настройка параметров
    arguments = []
    if localization:
        slam_params["Mem/IncrementalMemory"] = "False"
        slam_params["Mem/InitWMWithAllNodes"] = "True"
    else:
        arguments.append("-d")  # удалить старую базу данных при запуске

    nodes = [
        Node(
            package="rtabmap_odom",
            executable="icp_odometry",
            output="screen",
            parameters=[common_params, odom_params],
            remappings=[("scan_cloud", lidar_topic)],
        ),
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            output="screen",
            parameters=[common_params, slam_params],
            remappings=[("scan_cloud", lidar_topic)],
            arguments=arguments,
        ),
        Node(
            package="rtabmap_viz",
            executable="rtabmap_viz",
            output="screen",
            parameters=[common_params, slam_params],
            remappings=[("scan_cloud", "odom_filtered_input_scan")],
        ),
    ]

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "voxel_size", default_value="0.3", description="Voxel grid filter size"
            ),
            DeclareLaunchArgument(
                "localization",
                default_value="false",
                description="Run in localization mode",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
