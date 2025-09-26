#!/usr/bin/env python3
"""
Top-level launch file to start the Gazebo simulation without SLAM but with Nav2 running.

This launch file acts as an entry point and includes the main gazebo.launch.py
from the slambot_gazebo package, providing a complete simulation environment
with localization but without mapping capabilities.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    """Generate the launch description for the simulation without SLAM."""

    # Get package directories
    #pkg_slambot_gazebo = get_package_share_directory('slambot_gazebo')
    pkg_slambot_nav2 = get_package_share_directory('slambot_nav2')
    pkg_slambot_bringup = get_package_share_directory('slambot_bringup')

    # --- Declare Launch Arguments ---
    #These are arguments you can edit, and which are passed through into the gazebo.launch.py file, and
    #the nav2.launch.py file. It allows you to control certain parameters from the top level in the bringup launch file
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
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([pkg_slambot_nav2, 'maps', 'my_maze_map_1.yaml']),
        description='Full path to the map file to load for navigation')
    
    #Namespaces the /tf topics when using Nav2
    declare_using_nav2_cmd = DeclareLaunchArgument(
        'using_nav_2', 
        default_value='true',
        description='Namespaces the /tf topics if set to true')

    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution(
            [pkg_slambot_nav2, 'rviz', 'nav2_gz_rviz_noslam_config.rviz']
        ),
        description='Full path to the RViz configuration file to use'
    )


    # --- Include Gazebo Launch File ---
    # start_simulation_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_slambot_gazebo, 'launch', 'gazebo.launch.py')
    #     ),
    #     # Pass the launch arguments to the included launch file
    #     launch_arguments={
    #         'world': LaunchConfiguration('world'),
    #         'robot_name': LaunchConfiguration('robot_name'),
    #         'use_sim_time': LaunchConfiguration('use_sim_time'),
    #         'rviz_config_file': LaunchConfiguration('rviz_config_file'),
    #         'using_nav_2': LaunchConfiguration('using_nav_2') # ENSURES /slambot/tf!!!
    #     }.items()
    # )

        # --- Include Gazebo Launch File ---
    start_simulation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slambot_bringup, 'launch', 'sim_no_slam.launch.py')
        ),
        # Pass the launch arguments to the included launch file
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'robot_name': LaunchConfiguration('robot_name'),
            'rviz_config_file': LaunchConfiguration('rviz_config_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'using_nav_2': LaunchConfiguration('using_nav_2') # ENSURES /slambot/tf!!!
        }.items()
    )

    # --- Include Nav2 Launch File ---
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slambot_nav2, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('robot_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': LaunchConfiguration('map'),
        }.items(),
    )

    # --- Create Launch Description ---
    ld = LaunchDescription()

    # Add the declared arguments and the include action
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_cmd)  
    ld.add_action(declare_using_nav2_cmd)
    ld.add_action(declare_rviz_config_cmd)          
    ld.add_action(start_simulation_cmd)
    ld.add_action(start_nav2_cmd)

    return ld