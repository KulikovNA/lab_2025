# Author: Addison Sears-Collins
# Date: September 23, 2021
# Description: Load a URDF and world file into Gazebo.
# https://automaticaddison.com

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

  # Constants for paths to different files and folders
  gazebo_models_path = 'models'
  # Имя пакета, который предоставляет URDF/XACRO для YouBot
  # Это 'youbot_description', как и было.
  package_name_robot = 'youbot_description' # <-- ИЗМЕНЕНО ИМЯ ПЕРЕМЕННОЙ
  
  # НОВОЕ: Имя пакета, который предоставляет World-файл
  # Согласно вашей структуре, cafe.world находится в two_wheeled_robot.
  package_name_world = 'two_wheeled_robot' # <-- ДОБАВЛЕНО
  
  # Имя робота, используемое при спауне в Gazebo
  robot_name_in_model = 'youbot'
  
  # Путь к URDF/XACRO файлу YouBot внутри пакета youbot_description
  urdf_file_path = 'urdf/youbot.urdf'
  
  # rviz_config_file_path
  rviz_config_file_path = 'rviz/urdf_gazebo_config.rviz'
  
  # world_file_path: Теперь связан с two_wheeled_robot
  world_file_path = 'worlds/cafe.world' # Этот путь внутри two_wheeled_robot

  # Pose where we want to spawn the robot
  spawn_x_val = '0.0'
  spawn_y_val = '-10.0'
  spawn_z_val = '1.0'
  spawn_yaw_val = '1.57'

  ############ You do not need to change anything below this line #############
  
  # Set the path to different files and folders.  
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
  
  # Пакет для URDF и RViz
  pkg_share_robot = FindPackageShare(package=package_name_robot).find(package_name_robot) # <-- ИЗМЕНЕНО
  default_urdf_model_path = os.path.join(pkg_share_robot, urdf_file_path) # <-- ИЗМЕНЕНО
  default_rviz_config_path = os.path.join(pkg_share_robot, rviz_config_file_path) # <-- ИЗМЕНЕНО
  gazebo_models_path = os.path.join(pkg_share_robot, gazebo_models_path) # Assuming models are with robot description
  os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
  
  # НОВОЕ: Пакет для world файла
  pkg_share_world = FindPackageShare(package=package_name_world).find(package_name_world) # <-- ДОБАВЛЕНО
  world_path = os.path.join(pkg_share_world, world_file_path) # <-- ИЗМЕНЕНО

  # Launch configuration variables specific to simulation
  gui = LaunchConfiguration('gui')
  headless = LaunchConfiguration('headless')
  namespace = LaunchConfiguration('namespace')
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  urdf_model = LaunchConfiguration('urdf_model')
  use_namespace = LaunchConfiguration('use_namespace')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_rviz = LaunchConfiguration('use_rviz')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')
  
  # Declare the launch arguments  
  declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
    name='gui',
    default_value='True',
    description='Flag to enable joint_state_publisher_gui')
    
  declare_namespace_cmd = DeclareLaunchArgument(
    name='namespace',
    default_value='',
    description='Top-level namespace')

  declare_use_namespace_cmd = DeclareLaunchArgument(
    name='use_namespace',
    default_value='false',
    description='Whether to apply a namespace to the navigation stack')
            
  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')

  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')

  declare_urdf_model_path_cmd = DeclareLaunchArgument(
    name='urdf_model', 
    default_value=default_urdf_model_path, 
    description='Absolute path to robot urdf file')
    
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')

  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')
    
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true')

  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')

  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')
  
  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.    
  start_robot_state_publisher_cmd = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': ParameterValue(Command(['xacro ', urdf_model]), value_type=str)}])

  # Publish the joint states of the robot
  start_joint_state_publisher_cmd = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    condition=UnlessCondition(gui))

  # Launch RViz
  start_rviz_cmd = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])

  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world, 'verbose': 'true'}.items()
  )

  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])),
    launch_arguments={'verbose': 'true'}.items()
  )
  # Launch the robot
  spawn_entity_cmd = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', robot_name_in_model, 
                '-topic', 'robot_description',
                    '-x', spawn_x_val,
                    '-y', spawn_y_val,
                    '-z', spawn_z_val,
                    '-Y', spawn_yaw_val],
                    output='screen')

  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_use_joint_state_publisher_cmd)
  ld.add_action(declare_namespace_cmd)
  ld.add_action(declare_use_namespace_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_urdf_model_path_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_world_cmd)

  # Add any actions
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
  ld.add_action(spawn_entity_cmd)
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_joint_state_publisher_cmd)
  ld.add_action(start_rviz_cmd)

  return ld