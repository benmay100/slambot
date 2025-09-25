#!/usr/bin/env python3
"""
Launch RViz and the robot_state_publisher for a namespaced robot.

This launch file is responsible for visualizing the robot's state. It starts:
1. robot_state_publisher: To publish TF transforms from the URDF and joint states.
2. joint_state_publisher_gui: A GUI to manually control robot joints (optional).
3. RViz: The visualization tool, loaded with a specific configuration.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # ================== Get Package Directories =================== #
    
    pkg_slambot_description = FindPackageShare('slambot_description')

    # ================== Declare Launch Arguments =================== #

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name', 
        default_value='slambot',
        description='The name/namespace of the robot')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_jsp_gui_cmd = DeclareLaunchArgument(
        'jsp_gui', 
        default_value='true',
        description='Flag to enable joint_state_publisher_gui')
    
    declare_using_nav2_cmd = DeclareLaunchArgument(
        'using_nav_2', 
        default_value='false',
        description='Namespaces the /tf topics if set to true')
        
    declare_urdf_model_cmd = DeclareLaunchArgument(
        'urdf_model',
        default_value=PathJoinSubstitution([pkg_slambot_description, 'urdf', 'slambot.urdf.xacro']),
        description='Absolute path to robot URDF file')
    
    # RENAMED for consistency with the other files
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution([pkg_slambot_description, 'rviz', 'rviz_only_config.rviz']),
        description='Full path to the RViz configuration file to use')
    
    # ================== Robot Description Setup =================== #

    # Process the URDF file from the xacro
    robot_description_content = ParameterValue(
        Command(['xacro ', LaunchConfiguration('urdf_model')]),
        value_type=str
    )

    # ================== Node Definitions =================== #

    # ---- Robot State Publisher ----
    
    # 1. Namespaced version for Nav2
    start_robot_state_publisher_nav2_cmd = Node(
        condition=IfCondition(LaunchConfiguration('using_nav_2')),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('robot_name'),
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description_content,
        }],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    # 2. Global version for standalone use
    start_robot_state_publisher_global_cmd = Node(
        condition=UnlessCondition(LaunchConfiguration('using_nav_2')),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('robot_name'),
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description_content,
        }]
        # No remappings here
    )

    # Joint State Publisher GUI Node
    # Provides a GUI to manually move the robot's joints.
    start_joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        namespace=LaunchConfiguration('robot_name'),
        condition=IfCondition(LaunchConfiguration('jsp_gui')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # ---- RViz2 Node ----

    # 1. Namespaced version for Nav2
    start_rviz_nav2_cmd = Node(
        condition=IfCondition(LaunchConfiguration('using_nav_2')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_file')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/tf', [LaunchConfiguration('robot_name'), '/tf']),
            ('/tf_static', [LaunchConfiguration('robot_name'), '/tf_static'])
        ]
    )

    # 2. Global version for standalone use
    start_rviz_global_cmd = Node(
        condition=UnlessCondition(LaunchConfiguration('using_nav_2')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_file')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        # No remappings here
    )

    # ================== Create Launch Description =================== #

    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_urdf_model_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_using_nav2_cmd)

    # Add nodes to the launch description
    ld.add_action(start_robot_state_publisher_nav2_cmd)
    ld.add_action(start_robot_state_publisher_global_cmd)
    ld.add_action(start_joint_state_publisher_gui_cmd)
    ld.add_action(start_rviz_nav2_cmd)
    ld.add_action(start_rviz_global_cmd)

    return ld