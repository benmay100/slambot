#!/usr/-bin/env python3
"""
Launch file for the SLAM Toolbox.

This launch file starts the slam_toolbox node in asynchronous mode, which is suitable
for creating maps from laser scan data. It is designed to be included in a
higher-level launch file.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Generate the launch description for the SLAM Toolbox node."""

    # Get the path to this package's share directory
    pkg_slambot_slam = get_package_share_directory('slambot_slam')

    # --- Declare Launch Arguments ---
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_slambot_slam, 'config', 'slam_params.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )

    # This argument allows the parent launch file to specify the robot's namespace
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='slambot',
        description='The namespace of the robot'
    )

    # --- SLAM Toolbox Node ---
    start_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
          LaunchConfiguration('slam_params_file'),
          {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        # Remap the generic 'scan' and 'map' topics to the robot's namespaced topic
        remappings=[
            ('scan', [LaunchConfiguration('robot_name'), '/scan']),
            ('map', [LaunchConfiguration('robot_name'), '/map'])
        ]
    )

    # --- Create Launch Description ---
    ld = LaunchDescription()

    # Add the launch arguments to the launch description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_robot_name_cmd) # <-- Added the new argument

    # Add the slam_toolbox node to the launch description
    ld.add_action(start_slam_toolbox_node)

    return ld
