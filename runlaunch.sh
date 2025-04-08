#!/bin/bash
# Источаем окружение ROS, Gazebo и нашего проекта
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source /workspace/lab_2025/install/setup.bash

# Запускаем launch-файл
ros2 launch two_wheeled_robot launch_sim_cv.launch.py