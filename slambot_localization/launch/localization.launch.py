#!/usr/bin/env python3
"""
Launch the robot_localization EKF node.

This launch file starts the Extended Kalman Filter (EKF) node from the
robot_localization package, which fuses odometry and IMU data to produce
a filtered, more accurate odometry estimate.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Generate the launch description for the EKF node."""
    
    # Get the path to the package's share directory
    pkg_slambot_localization = get_package_share_directory('slambot_localization')
    
    # --- EKF Configuration File ---
    # Construct the full path to the EKF configuration file
    ekf_config_path = os.path.join(pkg_slambot_localization, 'config', 'ekf.yaml')

    # --- Declare Launch Arguments ---
    # This argument is no longer needed for the namespace but is kept for potential future use
    # or if you want to pass it to other nodes within this launch file.
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='slambot',
        description='The namespace of the robot'
    )

    # This argument is passed in from the parent gazebo.launch.py file
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # --- EKF Node ---
    # Start the ekf_filter_node from the robot_localization package
    start_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node', # The node name is still unique
        output='screen',
        # namespace=LaunchConfiguration('robot_name'), # <-- REMOVED: Run node in global namespace
        parameters=[
            ekf_config_path,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        # Remap the default output topic to the desired namespaced topic
        remappings=[
            ('/odometry/filtered', [LaunchConfiguration('robot_name'), '/odometry/filtered'])
        ]
    )

    # --- Create Launch Description ---
    ld = LaunchDescription()

    # Add actions to the launch description
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(start_ekf_node)

    return ld
