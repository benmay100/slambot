#!/usr/bin/env python3
"""
Top-level launch file to start the Gazebo and RVIZ with SLAM (you can choose between 'cartographer' and 'slam_toolbox')
Importantly, this launch file also runs the 'qr_code_reader' node from the 'slambot_scripts' package
As this is designed for launching into an environment with QR codes dotted around, so you can map your environment AND...
...stop at each QR code to automatically get the coordinates and robot position for use in a later Nav2 automated program 

"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():

    # Get the path to the slambot_gazebo package
    pkg_slambot_gazebo = get_package_share_directory('slambot_gazebo')
    pkg_slambot_description = get_package_share_directory('slambot_description')
    pkg_slambot_slam = get_package_share_directory('slambot_slam')
    pkg_slambot_bringup = get_package_share_directory('slambot_bringup')

    # Define Config File Paths
    # Path to the new twist_mux config
    twist_mux_file = os.path.join(pkg_slambot_bringup, 'config', 'twist_mux.yaml')
    
    # Path for the joystick config
    joy_config_file = os.path.join(pkg_slambot_bringup, 'config', 'joy_teleop.yaml')


    # ========================= Declare Launch Arguments =========================== #   
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='indoor_world_with_qr_codes.sdf',
        description='The world file to launch in Gazebo'
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='slambot',
        description='The name for the robot'
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
    
    # This argument allows us to specify the SLAM params file from the command line
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_slambot_slam, 'config', 'slam_params.yaml'),
        description='Full path to the ROS2 parameters file for SLAM'
    )

    declare_slam_type_cmd = DeclareLaunchArgument(
        'slam_type', 
        default_value='slamtoolbox', #or can put 'cartographer'
        description='Launches the version of slam you want to use'
    )

    declare_using_joy_cmd = DeclareLaunchArgument(
        'using_joy',
        default_value='True',
        description='launches the joystick teleop nodes if true. If you do not have a joystick/controller set to false or will crash on launch.'
    )

    # =============================================================================== # 


    # ========================= Dynamic File Path Changes (RVIZ) ========================== #   

    # Path to the RVIZ configuration file depending on if using 'slam_toolbox' or 'cartographer_ros'
    rviz_config_path_slamtoolbox = os.path.join(pkg_slambot_slam, 'rviz', 'gazebo_rviz_slamtoolbox_config.rviz')
    rviz_config_path_cartographer = os.path.join(pkg_slambot_slam, 'rviz', 'gazebo_rviz_cartographer_config.rviz')


    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PythonExpression([
            # Check 1: If slam_type is 'cartographer'
            "'", rviz_config_path_cartographer, "' if '",
            LaunchConfiguration('slam_type'),
            "'.lower() == 'cartographer' else '",
            # Check 2: If slam_type is 'slamtoolbox' 
            rviz_config_path_slamtoolbox, "'"
        ]),
        description='Full path to the RViz config file. Automatically selects based on slam_type'
    )


    # ================================================================================== # 
    
    # ======================= Cartographer File Path Setup ==============================#   

    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(pkg_slambot_slam, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='cartographer_params.lua')
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

    # ================================================================================== #

    # =========== Launch RVIZ and Robot State Publisher From rviz.launch.py ============ # 
    
    start_rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slambot_description, 'launch', 'rviz.launch.py')
        ),
        # Pass the launch arguments to the included launch file
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rviz_config_file': LaunchConfiguration('rviz_config_file'), # Pass the config file down
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
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'headless': LaunchConfiguration('headless'),
            'robot_name': LaunchConfiguration('robot_name')
        }.items()
    )

    # ================================================================================ # 

    # ================ Start slamtoolbox (the default argument) ====================== #

    start_slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slambot_slam, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        #Below tells this only to launch if 'slamtoolbox' is selected
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('slam_type'), "'.lower() == 'slamtoolbox'"
        ]))
    )

    
    # ================================================================================ # 

    # ============= Start cartographer (only if selected as argument) ================ #

    start_cartographer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slambot_slam, 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={
            'cartographer_config_file': LaunchConfiguration('cartographer_config_file'),
            'configuration_directory': LaunchConfiguration('configuration_directory'), 
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('slam_type'), "'.lower() == 'cartographer'"
        ]))
    )
    
    # ================================================================================ # 

    # =========== Run the 'qr_code_reader.py' node (in 'slambot_scripts') ============ #

    start_qr_reader_cmd = Node(
    package='slambot_scripts',
        executable='qr_code_reader', 
        name='qr_code_mazer_driver',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
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
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_slam_type_cmd)
    ld.add_action(declare_configuration_directory_cmd)
    ld.add_action(declare_cartographer_config_file_cmd)
    ld.add_action(declare_using_joy_cmd)
    #Dynamic file changes here
    ld.add_action(declare_rviz_config_file_cmd)
    
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_gz_cmd)
    ld.add_action(twist_mux_node)
    ld.add_action(joy_node)
    ld.add_action(teleop_twist_joy_node)
    ld.add_action(start_slam_cmd)
    ld.add_action(start_cartographer_cmd)
    ld.add_action(start_qr_reader_cmd)

    return ld



        
