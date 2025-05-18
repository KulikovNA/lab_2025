#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Получаем пути к пакетам
    two_wheeled_robot_dir = get_package_share_directory('two_wheeled_robot')
    cv_basics_dir = get_package_share_directory('cv_basics')

    # Включаем существующий launch файл, который запускает URDF в Gazebo
    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(two_wheeled_robot_dir, 'launch', 'launch_urdf_into_gazebo.launch.py')
        )
    )

    # Запускаем ноду inference_yolo_node из пакета cv_basics
    inference_yolo_node = Node(
        package='cv_basics',
        executable='inference_yolo_node',
        name='inference_yolo_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Запускаем ноду scan_subscriber из пакета cv_basics
    scan_subscriber_node = Node(
        package='cv_basics',
        executable='scan_subscriber',
        name='scan_subscriber',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        urdf_launch,
        inference_yolo_node,
        scan_subscriber_node
    ])
