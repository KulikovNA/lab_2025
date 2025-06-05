#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    try:
        # Путь к пакету
        two_wheeled_robot_dir = get_package_share_directory('two_wheeled_robot')

        pkg_share = get_package_share_directory('two_wheeled_robot')
        urdf_file = os.path.join(pkg_share, 'urdf', 'two_wheeled_robot_with_gazebo_plugins.urdf')
        controller_config = os.path.join(pkg_share, 'config', 'joint_controllers.yaml')

        # Подключение URDF в Gazebo
        urdf_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(two_wheeled_robot_dir, 'launch', 'launch_urdf_into_gazebo.launch.py')
            )
        )

        robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ', urdf_file
        ])

        ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            controller_config
        ], output='screen')


        delay_spawner = TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['camera_position_controller'],  # Исправлено здесь
                    output='screen')
                ])

        camera_rotator = Node(
            package="slam_with_filter",
            executable="camera_rotator",
            name="camera_rotator",
            output="screen"
        )

        inference_node = Node(
            package="slam_with_filter",
            executable="segmentation_filter",
            name="segmentation_filter",
            output="screen"
        )

        lidar_node = Node(
            package="slam_with_filter",
            executable="lidar_filtered",
            name="lidar_filter",
            output="screen"
        )

        return LaunchDescription([
            urdf_launch,
            ros2_control_node,
            delay_spawner,
            camera_rotator,
            inference_node,
            lidar_node
            ])
    except Exception as e:
        print("Ошибка при запуске launch-файла:", e)
        raise
