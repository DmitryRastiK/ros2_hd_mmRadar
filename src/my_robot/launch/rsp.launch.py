import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import UnlessCondition


def generate_launch_description():
    # Проверка использования времени симуляции
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Пути к файлам
    pkg_path = os.path.join(get_package_share_directory("my_robot"))
    xacro_file = os.path.join(pkg_path, "urdf", "my_robot.urdf.xacro")
    rviz_config_file = os.path.join(pkg_path, "rviz", "radar.rviz")
    controller_params_file = os.path.join(
        pkg_path, "config", "diff_drive_controllers.yaml"
    )

    # Описание робота
    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)
    robot_description_param = {
        "robot_description": robot_description,
        "use_sim_time": use_sim_time,
        "ignore_timestamp": False,  # Критически важный параметр ?
    }

    # Узел robot_state_publisher
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
    )

    # Узел ROS2 Control (только для реального робота)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description_param, controller_params_file],
        condition=UnlessCondition(use_sim_time),
    )

    # Спавним контроллеры
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--param-file", controller_params_file],
        output="screen",
    )

    # (Опционально) узел joint_state_publisher_gui
    # Для дифф. робота обычно не нужен, но если хотите видеть слайдеры, оставьте
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    # Узел RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    path_publisher = Node(
        package="my_path_publisher",
        executable="path_publisher",
        name="path_publisher",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use sim time if true",
            ),
            node_robot_state_publisher,
            ros2_control_node,
            # joint_state_spawner,
            # diff_drive_spawner,
            # joint_state_publisher_node,
            rviz_node,
            path_publisher,
        ]
    )
