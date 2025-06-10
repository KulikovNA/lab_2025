import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ParameterValue
import xacro
from launch_ros.actions import Node

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def generate_launch_description():
    # Get the directory of the 'youbot_description' package
    world_file_path = get_package_file('youbot_description', 'worlds/two.world')
    urdf_file_path = get_package_file('youbot_description', 'urdf/youbot.urdf')

    # Преобразование xacro файла в строку URDF
    doc = xacro.process_file(urdf_file_path)
    robot_description_content = doc.toprettyxml(indent='  ')

    # Launch Gazebo with the specified world
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file_path, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                               '-entity', 'youbot'],
                    output='screen')
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(robot_description_content, value_type=str)}],  # Передача URDF как параметра
    )

    # Create and return the launch description
    ld = LaunchDescription()
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(robot_state_publisher)

    return ld
