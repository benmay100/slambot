#!/usr/bin/env python3
"""
Launch a Gazebo simulation for the Slambot.
This launch file launches Gazebo only, to get a working simulation it needs to be launched alongside RVIZ and robot state publisher.

"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

def generate_launch_description():

    # ================== Get Package Directories =================== #
    
    pkg_slambot_gazebo = get_package_share_directory('slambot_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_slambot_description = get_package_share_directory('slambot_description')
    pkg_slambot_localization = get_package_share_directory('slambot_localization')


    # ================== Declare Launch Arguments =================== #

    # Robot name - for Gazebo entity name
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name', 
        default_value='slambot',
        description='The name of the robot')

    # World to launch in Gazebo
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='indoor_world_with_qr_codes.sdf',
        description='The world file to launch in Gazebo')

    # Whether to run Gazebo without a GUI
    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to run Gazebo without a GUI')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    

    # Path to the correct ros_gz_bridge yaml file 
    ros_gz_bridge_params_file = os.path.join(pkg_slambot_gazebo, 'config', 'ros_gz_bridge.yaml')


    
    # ================== Set Environment Variables =================== #
    
    # This is critical to allow Gazebo to find the meshes from the slambot_description package
    parent_of_share_path = os.path.dirname(pkg_slambot_description)

    # --- Set GZ_SIM_RESOURCE_PATH  ---
    # This is critical to allow Gazebo to find the meshes from the slambot_description package
    set_gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=[
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            os.pathsep,
            parent_of_share_path
        ]
    )
    
    # --- Set GAZEBO_MODEL_PATH --- this is handled from inside the package.xml file - see near the bottom of the file some important exports!


    # ================== Start Gazebo Simulation =================== #

    world_path = PathJoinSubstitution([pkg_slambot_gazebo, 'worlds', LaunchConfiguration('world')])

    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': ['-r -s ', world_path],
            'on_exit_shutdown': 'true'
        }.items()
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': '-g ',
            'on_exit_shutdown': 'true'
        }.items(),
        #Below tells gz_sim_gui not to launch if 'headless' set to True
        condition=IfCondition(PythonExpression(['not ', LaunchConfiguration('headless')]))
    )

    # ================== Start ROS-Gazebo Bridge =================== #
    
    #If not using namespacing (we don't remap any topics)
    start_ros_gz_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': ros_gz_bridge_params_file,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    # ================== Start Localization (EKF) =================== #
    
    start_ekf_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slambot_localization, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items(),
    )


    # ================== Spawn Robot into Gazebo =================== #
    
    spawn_robot_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-x', '-6.0',
            '-y', '-2.0',
            '-z', '0.2',
            '-topic', 'robot_description' 
        ],
        output='screen'
    )

    # ================== Create Launch Description =================== #
    
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add actions
    ld.add_action(set_gz_sim_resource_path)
    ld.add_action(gz_sim_server)
    ld.add_action(gz_sim_gui)
    ld.add_action(start_ros_gz_bridge_cmd)
    ld.add_action(start_ekf_localization_cmd)
    ld.add_action(spawn_robot_cmd)

    return ld