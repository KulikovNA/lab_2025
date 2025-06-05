#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    try:
        # Путь к пакету
        two_wheeled_robot_dir = get_package_share_directory('two_wheeled_robot')

        # Подключение URDF в Gazebo
        urdf_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(two_wheeled_robot_dir, 'launch', 'launch_urdf_into_gazebo.launch.py')
            )
        )

        # Запуск ноды для вращения камеры
        camera_rotator = Node(
            package="slam_with_filter",
            executable="camera_rotator",
            name="camera_rotator",
            output="screen",
        )

        return LaunchDescription([
            urdf_launch,
            camera_rotator,
        ])
    except Exception as e:
        print("Ошибка при запуске launch-файла:", e)
        raise
