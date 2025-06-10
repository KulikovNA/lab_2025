import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def generate_launch_description():
    # Get the directory of the 'youbot_description' package
    world_file_path = get_package_file('youbot_description', 'worlds/one.world')

    # Launch Gazebo with the specified world
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file_path],
        output='screen'
    )

    """     
    # Teleop node for keyboard control
    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        namespace='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        remappings=[
            ('/teleop_twist_keyboard/cmd_vel', '/youbot/cmd_vel'),
        ],
        output='screen',
        prefix='xterm -e'
    ) 
    """

    # Create and return the launch description
    ld = LaunchDescription()
    ld.add_action(gazebo)
    #ld.add_action(teleop_keyboard)

    return ld
