#!/usr/bin/env python3
"""
Top-level launch file to start the Gazebo and RVIZ without any SLAM or Nav2 functionality.
This launch file acts as an entry point and includes the main gazebo.launch.py and rviz.launch.py files, from the slambot_gazebo and slambot_description packages.

"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition




def generate_launch_description():
    """Generate the launch description for the simulation without SLAM."""

    # Get the path to the slambot_gazebo package
    pkg_slambot_gazebo = get_package_share_directory('slambot_gazebo')
    pkg_slambot_description = get_package_share_directory('slambot_description')
    pkg_slambot_bringup = get_package_share_directory('slambot_bringup')

    # Define Config File Paths
    twist_mux_file = os.path.join(pkg_slambot_bringup, 'config', 'twist_mux.yaml')
    joy_config_file = os.path.join(pkg_slambot_bringup, 'config', 'joy_teleop.yaml')
    rviz_config_path = os.path.join(pkg_slambot_gazebo, 'rviz', 'gazebo_and_rviz_and_localization_config.rviz')


    # ========================= Declare Launch Arguments =========================== #   
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='indoor_world_with_qr_codes.sdf',
        description='The world file to launch in Gazebo'
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='slambot',
        description='The name/namespace for the robot'
    )

    # Whether to run Gazebo without a GUI
    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to run Gazebo without a GUI')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_jsp_gui_cmd = DeclareLaunchArgument(
        'jsp_gui', 
        default_value='False',
        description='Flag to enable joint_state_publisher_gui'
    )

    declare_using_joy_cmd = DeclareLaunchArgument(
        'using_joy',
        default_value='True',
        description='launches the joystick teleop nodes if true. If you do not have a joystick/controller set to false or will crash on launch.'
    )


    # =============================================================================== # 


    # =========== Launch RVIZ and Robot State Publisher From rviz.launch.py ============ # 
    
    start_rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slambot_description, 'launch', 'rviz.launch.py')
        ),
        # Pass the launch arguments to the included launch file
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rviz_config_file': rviz_config_path, # Pass the config file down
            'jsp_gui': LaunchConfiguration('jsp_gui')
        }.items()
    )

    # ================================================================================ # 
    
    
    # =================== Launch Gazebo From gazebo.launch.py ========================= # 

    start_gz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slambot_gazebo, 'launch', 'gazebo.launch.py')
        ),
        # Pass the launch arguments to the included launch file
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'robot_name': LaunchConfiguration('robot_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'headless': LaunchConfiguration('headless'),
        }.items()
    )

    # ================================================================================ # 

    # =================== Launch Twist Mux and Teleop Nodes Here =================== #

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_file],
        remappings=[("cmd_vel_out", "/cmd_vel")] 
    )

    # Start the 'joy' driver node, *if* using_joy is true
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'dev': '/dev/input/js0'}],
        condition=IfCondition(LaunchConfiguration('using_joy'))
    )

    # Start the 'teleop_twist_joy' node, *if* using_joy is true
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_config_file],
        remappings=[
            # Remap the output to the topic twist_mux is listening for
            ('cmd_vel', '/cmd_vel_joy') 
        ],
        condition=IfCondition(LaunchConfiguration('using_joy'))
    )

    # =================================================================================#


    # ========================= Create Launch Description ============================ # 
    ld = LaunchDescription()

    # Add the declared arguments and the include action
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_using_joy_cmd)
    
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_gz_cmd)
    ld.add_action(twist_mux_node)
    ld.add_action(joy_node)
    ld.add_action(teleop_twist_joy_node)

    return ld