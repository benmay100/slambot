#!/usr/bin/env python3
"""
Top-level launch file to start the Gazebo and RVIZ without SLAM.

This launch file acts as an entry point and includes the main gazebo.launch.py and rviz.launch.py files, from the slambot_gazebo and slambot_description packages.

You can launch with or without a namespaced environment, and with or without using localization (i.e. ekf node for filtered odometry readings)

"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    """Generate the launch description for the simulation without SLAM."""

    # Get the path to the slambot_gazebo package
    pkg_slambot_gazebo = get_package_share_directory('slambot_gazebo')
    pkg_slambot_description = get_package_share_directory('slambot_description')


    # ========================= Declare Launch Arguments =========================== #   
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='indoor_world_1.sdf',
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

    #Namespaces the /tf topics when using Nav2
    declare_using_namespace_cmd = DeclareLaunchArgument(
        'using_namespace', 
        default_value='False',
        description='Namespaces all topics (best for multiple robot setups) if set to true'
    )
    
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
    
    declare_using_localization_cmd = DeclareLaunchArgument(
        'using_localization',
        default_value='True',
        description='uses the "ros_gz_bridge_localization.yaml" file if true'
    )

    # =============================================================================== # 


    # ========================= Dynamic File Path Changes ========================== #   

    # Path to the RViz configuration file (depending on if 'using_namespace' and on if 'using_localization')
    rviz_config_path = os.path.join(pkg_slambot_gazebo, 'rviz', 'gazebo_and_rviz_config.rviz')
    rviz_config_path_namespaced = os.path.join(pkg_slambot_gazebo, 'rviz', 'gazebo_and_rviz_config_namespaced.rviz')
    rviz_config_path_localization = os.path.join(pkg_slambot_gazebo, 'rviz', 'gazebo_and_rviz_and_localization_config.rviz')
    rviz_config_path_localization_namespaced = os.path.join(pkg_slambot_gazebo, 'rviz', 'gazebo_and_rviz_and_localization_config_namespaced.rviz')


    declare_rviz_config_file_cmd = DeclareLaunchArgument(
    'rviz_config_file',
    default_value=PythonExpression([
        # Expression to evaluate if using_localization is true
        "('",
        rviz_config_path_localization_namespaced,
        "' if '",
        LaunchConfiguration('using_namespace'),
        "'.lower() == 'true' else '",
        rviz_config_path_localization,
        "')",
        # The main condition checking using_localization
        " if '",
        LaunchConfiguration('using_localization'),
        "'.lower() == 'true' else ",
        # Expression to evaluate if using_localization is false
        "('",
        rviz_config_path_namespaced,
        "' if '",
        LaunchConfiguration('using_namespace'),
        "'.lower() == 'true' else '",
        rviz_config_path,
        "')"
    ]),
    description='Full path to the RViz config file. Automatically selects based on using_namespace and using_localization.'
    )

    # Path to the correct ros_gz_bridge yaml file (depending on if 'using_localization')
    ros_gz_bridge_path = os.path.join(pkg_slambot_gazebo, 'config', 'ros_gz_bridge.yaml')
    ros_gz_bridge_with_localization_path = os.path.join(pkg_slambot_gazebo, 'config', 'ros_gz_bridge_localization.yaml')

    declare_ros_gz_bridge_file_cmd = DeclareLaunchArgument(
        'ros_gz_bridge_file',
        default_value=PythonExpression([
            "'", ros_gz_bridge_with_localization_path, "' if '",
            LaunchConfiguration('using_localization'),
            "'.lower() == 'true' else '", ros_gz_bridge_path, "'"
        ]),
        description='Full path to the correct ros_gz_bridge file dependig on if using localization package or not'
    )

    # ================================================================================== #   


    # =========== Launch RVIZ and Robot State Publisher From rviz.launch.py ============ # 
    
    start_rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slambot_description, 'launch', 'rviz.launch.py')
        ),
        # Pass the launch arguments to the included launch file
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rviz_config_file': LaunchConfiguration('rviz_config_file'), # Pass the config file down
            'jsp_gui': LaunchConfiguration('jsp_gui'),
            'using_namespace': LaunchConfiguration('using_namespace')
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
            'using_namespace': LaunchConfiguration('using_namespace'),
            'headless': LaunchConfiguration('headless'),
            'using_localization': LaunchConfiguration('using_localization'), #If true, also launched ekf node via slambot_localization package
            'ros_gz_bridge_file': LaunchConfiguration('ros_gz_bridge_file') #Pass correct ros_gz_bridge depending on if using localization package
        }.items()
    )

    # ================================================================================ # 


    # ========================= Create Launch Description ============================ # 
    ld = LaunchDescription()

    # Add the declared arguments and the include action
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_using_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_using_localization_cmd)
    #Dynamic file changes here
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_ros_gz_bridge_file_cmd)
    
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_gz_cmd)

    return ld