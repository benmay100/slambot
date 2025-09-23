#!/usr/bin/env python3
"""
Launch RViz visualization for a namespaced robot.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument('robot_name', default_value='slambot',
                          description='The name/namespace of the robot'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          description='Use simulation (Gazebo) clock if true'),
    DeclareLaunchArgument('use_rviz', default_value='true',
                          description='Whether to start RViz'),
    DeclareLaunchArgument('jsp_gui', default_value='true',
                          description='Flag to enable joint_state_publisher_gui'),
    DeclareLaunchArgument('urdf_model',
                          default_value=PathJoinSubstitution([
                              FindPackageShare('slambot_description'), 'urdf', 'slambot.urdf.xacro'
                          ]),
                          description='Absolute path to robot URDF file'),
    DeclareLaunchArgument('rviz_config',
                          default_value=PathJoinSubstitution([
                              FindPackageShare('slambot_description'), 'rviz', 'rviz_only_config.rviz'
                          ]),
                          description='Full path to the RViz config file to use')
]

def generate_launch_description():
    """Generate the launch description for the robot visualization."""

    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time') # Define for use in parameters
    
    # Process the URDF file. Note: no prefix is passed to xacro anymore.
    robot_description_content = ParameterValue(
        Command(['xacro', ' ', LaunchConfiguration('urdf_model')]),
        value_type=str
    )

    # Robot State Publisher Node
    # Takes the generic URDF and publishes TFs in the robot's namespace.
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content,
        }]
    )

    # Joint State Publisher GUI Node
    start_joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        namespace=robot_name,
        condition=IfCondition(LaunchConfiguration('jsp_gui')),
        parameters=[{'use_sim_time': use_sim_time}] # <-- CORRECTED: Added use_sim_time
    )

    # RViz2 Node
    start_rviz_cmd = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_gui_cmd)
    ld.add_action(start_rviz_cmd)

    return ld