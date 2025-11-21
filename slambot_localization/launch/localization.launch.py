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
    ekf_config_path_sim = os.path.join(pkg_slambot_localization, 'config', 'ekf_sim.yaml')


    # --- Declare Launch Arguments ---

    declare_ekf_param_file_cmd = DeclareLaunchArgument(
        'ekf_param_file',
        default_value=ekf_config_path_sim,
        description='Full path to the EKF parameter file (non-namespaced)'
    )

    # This argument is passed in from the parent gazebo.launch.py file
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # ================== Start EKF Node =================== #

    # When NOT using namespace
    start_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node', # The node name is still unique
        output='screen',
        # namespace=LaunchConfiguration('robot_name'), # <-- REMOVED: Run node in global namespace
        parameters=[
            LaunchConfiguration('ekf_param_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )


    # --- Create Launch Description ---
    ld = LaunchDescription()

    # Add actions to the launch description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_ekf_param_file_cmd)
    ld.add_action(start_ekf_node)

    return ld