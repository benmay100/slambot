#!/usr/bin/env python3
"""
Launch RViz and the robot_state_publisher for visualizing the robot model.

This launch file is responsible for visualizing the robot's state. It starts:
1. robot_state_publisher: To publish TF transforms from the URDF and joint states.
2. joint_state_publisher_gui: A GUI to manually control robot joints (optional).
3. RViz: The visualization tool, loaded with a specific configuration.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ================== Get Package Directories =================== #
    
    pkg_slambot_description_share = FindPackageShare('slambot_description')

    # ================== Declare Launch Arguments =================== #

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_jsp_gui_cmd = DeclareLaunchArgument(
        'jsp_gui', 
        default_value='True',
        description='Flag to enable joint_state_publisher_gui')
        
    declare_urdf_model_cmd = DeclareLaunchArgument(
        'urdf_model',
        default_value=PathJoinSubstitution([pkg_slambot_description_share, 'urdf', 'slambot.urdf.xacro']),
        description='Absolute path to robot URDF file')
    
    # The rviz config file choice is passed down as an argument by top level launch files
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution([pkg_slambot_description_share, 'rviz', 'rviz_config.rviz']),
        description='Full path to the RViz config file.'
    )

    
    # ================== Robot Description Setup =================== #

    # Process the URDF file from the xacro
    robot_description_content = ParameterValue(
        Command(['xacro ', LaunchConfiguration('urdf_model')]),
        value_type=str
    )

    # ================== Node Definitions =================== #


    # ---- Robot State Publisher ----#
    
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description_content,
        }]
        # No remappings here
    )


    # --------- Joint State Publisher GUI Node ----------

    start_joint_state_publisher_gui_cmd = Node(
        condition=IfCondition(PythonExpression([LaunchConfiguration('jsp_gui')])),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        
    )

    # ---- RViz2 Node ----

    start_rviz_cmd = Node(
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
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_urdf_model_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Add nodes to the launch description
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_gui_cmd)
    ld.add_action(start_rviz_cmd)

    return ld