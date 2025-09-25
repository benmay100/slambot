"""
Launch the robot_localization EKF node.

This launch file starts the Extended Kalman Filter (EKF) node from the
robot_localization package, which fuses odometry and IMU data to produce
a filtered, more accurate odometry estimate.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
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

    # ================== Start EKF Node =================== #

    # When not using Nav2, we launch in global namespace and just remap to slambot/odometry/filtered
    start_ekf_node = Node(
        condition=UnlessCondition(LaunchConfiguration('using_nav_2')),
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

    # When we ARE Nav2, we launch in global namespace and remap slambot/odometry/filtered, and  slambot/tf, and slambot/tf_static
    start_ekf_node_nav2 = Node(
        condition=IfCondition(LaunchConfiguration('using_nav_2')),
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
            ('/odometry/filtered', [LaunchConfiguration('robot_name'), '/odometry/filtered']),
            ('/tf', [LaunchConfiguration('robot_name'), '/tf']),
            ('/tf_static', [LaunchConfiguration('robot_name'), '/tf_static'])
        ]
    )



    # --- Create Launch Description ---
    ld = LaunchDescription()

    # Add actions to the launch description
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(start_ekf_node)
    ld.add_action(start_ekf_node_nav2)

    return ld