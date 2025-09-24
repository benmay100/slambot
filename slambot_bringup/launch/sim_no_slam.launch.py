#!/usr/bin/env python3
"""
Top-level launch file to start the Gazebo simulation without SLAM.

This launch file acts as an entry point and includes the main gazebo.launch.py
from the slambot_gazebo package, providing a complete simulation environment
with localization but without mapping capabilities.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Generate the launch description for the simulation without SLAM."""

    # Get the path to the slambot_gazebo package
    pkg_slambot_gazebo = get_package_share_directory('slambot_gazebo')

    # --- Declare Launch Arguments ---
    # These arguments are passed through to the included gazebo.launch.py file,
    # allowing you to configure the simulation from this top-level launch file.
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='maze_world_1.sdf',
        description='The world file to launch in Gazebo'
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='slambot',
        description='The name/namespace for the robot'
    )
    
    # You can add declarations for any other arguments from gazebo.launch.py here
    # if you want to be able to set them from the command line, e.g., 'headless'.

    # --- Include Gazebo Launch File ---
    # This is the core of the launch file. It includes the entire simulation setup
    # from the slambot_gazebo package.
    start_simulation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slambot_gazebo, 'launch', 'gazebo.launch.py')
        ),
        # Pass the launch arguments to the included launch file
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'robot_name': LaunchConfiguration('robot_name')
        }.items()
    )

    # --- Create Launch Description ---
    ld = LaunchDescription()

    # Add the declared arguments and the include action
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(start_simulation_cmd)

    return ld