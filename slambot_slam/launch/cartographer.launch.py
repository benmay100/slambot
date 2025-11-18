#!/usr/bin/env python3
"""
Launch file for Cartographer.

This launch file starts the ros_cartographer node which can work better than slam_toolbox for certain simulations
It is used for creating maps from laser scan data. It is designed to be included in a
higher-level launch file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for the Cartographer node."""

    # Get the path to this package's share directory
    pkg_slambot_slam = get_package_share_directory('slambot_slam')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  pkg_slambot_slam, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='cartographer_params.lua')
    

    # --- Declare Launch Arguments ---
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # --- Cartographer requires a configuration directory ---
    declare_configuration_directory_cmd = DeclareLaunchArgument(
        'configuration_directory',
        # Cartographer often needs to load assets from its package, so this is crucial.
        default_value=cartographer_config_dir,
        description='Directory containing the Cartographer .lua configuration file'
    )

    # --- Cartographer requires a .lua configuration file ---
    declare_cartographer_config_file_cmd = DeclareLaunchArgument(
        'cartographer_config_file',
        # Set this path to where your new .lua file is located
        # We will use a standard one for now, but you should copy and modify it.
        default_value=configuration_basename,
        description='Full path to the .lua configuration file for Cartographer'
    )
    
    # This argument allows the parent launch file to specify the robot's namespace
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='slambot',
        description='The namespace of the robot'
    )

    # This argument allows the parent launch file to specify the robot's namespace
    declare_using_namespace_cmd = DeclareLaunchArgument(
        'using_namespace',
        default_value='False',
        description='If set to true it launches cartographer with namespaced topics'
    )

    # =================== Cartographer Node (Non-Namespaced) =================== #
    
    start_cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        namespace='', 
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-configuration_directory', LaunchConfiguration('configuration_directory'),
            '-configuration_basename', LaunchConfiguration('cartographer_config_file')
        ],
        remappings=[
            ('odom', 'odometry/filtered') # Critical, as we're using an ekf node which takes the raw /odom and raw /imu/data topics, filters them and republishes to /odometry/filtered we MUST remap the '/odom' topic when launching cartographer!
        ],
        # Condition remains: launch if 'using_namespace' is False
        condition=IfCondition(PythonExpression(['not ', LaunchConfiguration('using_namespace')]))
    )


    # =================== Cartographer Node (Namespaced) =================== #
    start_cartographer_node_namespaced = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        namespace=LaunchConfiguration('robot_name'), 
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-configuration_directory', LaunchConfiguration('configuration_directory'),
            '-configuration_basename', LaunchConfiguration('cartographer_config_file')
        ],
        # REVISED REMAPPING: If the node is in namespace /slambot, it looks for /slambot/scan. 
    remappings=[
        ('odom', 'odometry/filtered'), # Critical, as we're using an ekf node which takes the raw /odom and raw /imu/data topics, filters them and republishes to /odometry/filtered we MUST remap the '/odom' topic when launching cartographer!
        ('/scan', 'scan'), #Force cartographer to subscribe to a namespaced version of this topic
        ('/tf', 'tf'),# Force Cartographer to publish its TFs to the namespaced topics
        ('/tf_static', 'tf_static')# Force Cartographer to publish its TFs to the namespaced topics
        ],
        # Condition remains: launch if 'using_namespace' is True
        condition=IfCondition(PythonExpression([' ', LaunchConfiguration('using_namespace')]))
    )
    
    # =================== Cartographer Occupancy Grid Node (Non-Namespaced) =================== #
    
    start_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node', # <--- FIXED EXECUTABLE NAME
        name='occupancy_grid_node',
        namespace='',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0'
        ],
        condition=IfCondition(PythonExpression(['not ', LaunchConfiguration('using_namespace')]))
    )

    # =================== Cartographer Occupancy Grid Node (Namespaced) =================== #
    start_occupancy_grid_node_namespaced = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node', # <--- FIXED EXECUTABLE NAME
        name='occupancy_grid_node',
        namespace=LaunchConfiguration('robot_name'),
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0'
        ],
        condition=IfCondition(PythonExpression([' ', LaunchConfiguration('using_namespace')]))
    )



    # --- Create Launch Description ---
    ld = LaunchDescription()

    # Add the launch arguments to the launch description (UNCHANGED)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_cartographer_config_file_cmd)
    ld.add_action(declare_configuration_directory_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_using_namespace_cmd)

    # Add the Cartographer nodes to the launch description
    ld.add_action(start_cartographer_node)
    ld.add_action(start_cartographer_node_namespaced)
    ld.add_action(start_occupancy_grid_node)
    ld.add_action(start_occupancy_grid_node_namespaced)


    return ld