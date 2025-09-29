#!/usr/bin/env python3
"""
Launch RViz and the robot_state_publisher for a namespaced robot.

This launch file is responsible for visualizing the robot's state. It starts:
1. robot_state_publisher: To publish TF transforms from the URDF and joint states.
2. joint_state_publisher_gui: A GUI to manually control robot joints (optional).
3. RViz: The visualization tool, loaded with a specific configuration.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ================== Get Package Directories =================== #
    
    pkg_slambot_description_share = FindPackageShare('slambot_description')
    pkg_slambot_description = get_package_share_directory('slambot_description')

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
        default_value='True',
        description='Flag to enable joint_state_publisher_gui')
    
    declare_using_namespace_cmd = DeclareLaunchArgument(
        'using_namespace', 
        default_value='False',
        description='Namespaces all topics (best for multiple robot setups) if set to true')
        
    declare_urdf_model_cmd = DeclareLaunchArgument(
        'urdf_model',
        default_value=PathJoinSubstitution([pkg_slambot_description_share, 'urdf', 'slambot.urdf.xacro']),
        description='Absolute path to robot URDF file')
    

    # Path to the RViz configuration file
    rviz_config_path = os.path.join(pkg_slambot_description, 'rviz', 'rviz_config.rviz')
    rviz_config_path_namespaced = os.path.join(pkg_slambot_description, 'rviz', 'rviz_config_namespaced.rviz')

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

    
    # ================== Robot Description Setup =================== #

    # Process the URDF file from the xacro
    robot_description_content = ParameterValue(
        Command(['xacro ', LaunchConfiguration('urdf_model')]),
        value_type=str
    )

    # ================== Node Definitions =================== #


    # ---- Robot State Publisher ----#
    
    # 1. Non Namespaced version
    start_robot_state_publisher_cmd = Node(
        condition=UnlessCondition(LaunchConfiguration('using_namespace')),
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

    # 2. Namespaced version
    start_robot_state_publisher_namespaced_cmd = Node(
        condition=IfCondition(LaunchConfiguration('using_namespace')),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('robot_name'),
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description_content,
        }],
        remappings=[
            ('/tf', 'tf'), 
            ('/tf_static', 'tf_static')
        ]
    )



    # --------- Joint State Publisher GUI Node ----------

    # 1. Non namespaced (Provides a GUI to manually move the robot's joints)
    start_joint_state_publisher_gui_cmd = Node(
        condition=IfCondition(PythonExpression(
                            [LaunchConfiguration('jsp_gui'), 
                             ' and not ', 
                             LaunchConfiguration('using_namespace')])),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        
    )

    # 1. Namespaced (Provides a GUI to manually move the robot's joints)
    start_joint_state_publisher_gui_namespaced_cmd = Node(
        condition=IfCondition(PythonExpression(
                            [LaunchConfiguration('jsp_gui'), 
                             ' and ', 
                             LaunchConfiguration('using_namespace')])),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )


    # ---- RViz2 Node ----

    # 1. Non Namespaced version
    start_rviz_cmd = Node(
        condition=UnlessCondition(LaunchConfiguration('using_namespace')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_file')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        # No remappings here
    )

    # 2. Namespaced version (we don't actually namespace RVIZ, but we do remap topics)
    start_rviz_namespaced_cmd = Node(
        condition=IfCondition(LaunchConfiguration('using_namespace')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_file')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/tf', [LaunchConfiguration('robot_name'), '/tf']),
            ('/tf_static', [LaunchConfiguration('robot_name'), '/tf_static']),
            ('/initialpose', [LaunchConfiguration('robot_name'), '/initialpose']),
            ('/goal_pose', [LaunchConfiguration('robot_name'), '/goal_pose']),
            ('/clicked_point', [LaunchConfiguration('robot_name'), '/clicked_point']),
            ('/waypoints', [LaunchConfiguration('robot_name'), '/waypoints']),
        ]
    )


    # ================== Create Launch Description =================== #

    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_urdf_model_cmd)
    ld.add_action(declare_using_namespace_cmd) #Needs to come BEFORE the declare_rviz_config_file_cmd
    ld.add_action(declare_rviz_config_file_cmd)

    # Add nodes to the launch description
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_robot_state_publisher_namespaced_cmd)
    ld.add_action(start_joint_state_publisher_gui_cmd)
    ld.add_action(start_joint_state_publisher_gui_namespaced_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_rviz_namespaced_cmd)

    return ld