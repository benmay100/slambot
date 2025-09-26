#!/usr/bin/env python3
"""
Top-level launch file to start the Gazebo simulation WITH SLAM.

This launch file acts as an entry point, bringing up the complete simulation
environment (Gazebo, RViz, Localization) and also starting the SLAM Toolbox
node to enable map creation. It also handles the lifecycle management of the
SLAM node.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    """Generate the launch description for the simulation with SLAM."""

    # Get paths to the other packages
    pkg_slambot_gazebo = get_package_share_directory('slambot_gazebo')
    pkg_slambot_slam = get_package_share_directory('slambot_slam')

    # --- Declare Launch Arguments ---
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='indoor_world_1.sdf',
        description='The world file to launch in Gazebo'
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='slambot',
        description='The name/namespace for the robot'
    )

    # Whether to run Gazebo without a GUI
    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to run Gazebo without a GUI')

    #Namespaces the /tf topics when using Nav2
    declare_using_namespace_cmd = DeclareLaunchArgument(
        'using_namespace', 
        default_value='False',
        description='Namespaces all topics (best for multiple robot setups) if set to true')

    # This argument allows us to specify the SLAM params file from the command line
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_slambot_slam, 'config', 'slam_params.yaml'),
        description='Full path to the ROS2 parameters file for SLAM'
    )

    # Path to the RViz configuration file
    rviz_config_path = os.path.join(pkg_slambot_slam, 'rviz', 'gazebo_rviz_and_slam_config.rviz')
    rviz_config_path_namespaced = os.path.join(pkg_slambot_slam, 'rviz', 'gazebo_rviz_and_slam_config_namespaced.rviz')

    # The rviz config file choice is conditional on the status of 'using_namespace.
    # It uses a case-insensitive check on 'using_namespace' to select the correct path.
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PythonExpression([
            "'", rviz_config_path_namespaced, "' if '",
            LaunchConfiguration('using_namespace'),
            "'.lower() == 'true' else '", rviz_config_path, "'"
        ]),
        description='Full path to the RViz config file. Automatically selects namespaced version if using_namespace=True.'
    )

    # ========= Start Gazebo and Localization - Using Gazebo Launch File ========= #

    # This includes the simulation environment (Gazebo, Localization, RViz)
    start_simulation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slambot_gazebo, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'robot_name': LaunchConfiguration('robot_name'),
            'rviz_config_file': LaunchConfiguration('rviz_config_file'), # Pass the config file down
            'using_namespace': LaunchConfiguration('using_namespace'),
            'headless': LaunchConfiguration('headless')
        }.items()
    )

    # ========= Start SLAM - Using Slam Launch File ========= #

    # 1. If NOT using namespacing...
    start_slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slambot_slam, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'robot_name': LaunchConfiguration('robot_name'),
        }.items(),
        #Below tells this NOT to launch if 'using_namespace' set to True
        condition=IfCondition(PythonExpression(['not ', LaunchConfiguration('using_namespace')]))
    )

    # 1. If using namespacing...
    start_slam_cmd_namespaced = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slambot_slam, 'launch', 'slam_namespaced.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'robot_name': LaunchConfiguration('robot_name'),
        }.items(),
        #Below tells this NOT to launch if 'using_namespace' set to False
        condition=IfCondition(PythonExpression([' ', LaunchConfiguration('using_namespace')]))
    )
    

    # --- Create Launch Description ---
    ld = LaunchDescription()

    # Add the declared arguments
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_using_namespace_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    
    # Add the actions to launch and manage the nodes
    ld.add_action(start_simulation_cmd)
    ld.add_action(start_slam_cmd)
    ld.add_action(start_slam_cmd_namespaced)

    return ld

