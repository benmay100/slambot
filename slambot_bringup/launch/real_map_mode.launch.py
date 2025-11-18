#!/usr/bin/env python3
"""
Top-level launch file to start the REAL robot with SLAM, localization and EKF sensor fusion.
You can choose which version of slam you want to use 'cartographer_ros' or 'slam_toolbox'....
...depending on preference and which works best in the environment you're mapping
You can launch with or without a namespaced environment

"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, TimerAction


def generate_launch_description():

    # ========================= Paths & Environment Setup =========================== #   

    # 1. Get the paths to the required packages
    pkg_slambot_description = get_package_share_directory('slambot_description')
    pkg_slambot_slam = get_package_share_directory('slambot_slam')
    pkg_slambot_bringup = get_package_share_directory('slambot_bringup')
    pkg_slambot_localization = get_package_share_directory('slambot_localization')

    # 2. Define file paths
    xacro_file = os.path.join(pkg_slambot_description, 'urdf', 'slambot.urdf.xacro')
    controllers_file = os.path.join(pkg_slambot_bringup, 'config', 'slambot_controllers.yaml')
    twist_mux_file = os.path.join(pkg_slambot_bringup, 'config', 'twist_mux.yaml') # Path to the new twist_mux config
    joy_config_file = os.path.join(pkg_slambot_bringup, 'config', 'joy_teleop.yaml') # Path for the joystick config
    ekf_real_params = os.path.join(pkg_slambot_localization, 'config', 'ekf_real.yaml')
    ekf_real_params_namespaced = os.path.join(pkg_slambot_localization, 'config', 'ekf_namespaced_real.yaml')

    # 3. Process URDF
    robot_description_content = Command(
        [FindExecutable(name='xacro'), ' ', xacro_file, ' ', 'using_sim:=false']
    )
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # ===============================================================================#

    # ========================= Declare Launch Arguments =========================== #   

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='slambot',
        description='The name/namespace for the robot'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='MUST BE SET TO FALSE FOR REAL ROBOT!'
    )

    #Namespaces the /tf topics when using Nav2
    declare_using_namespace_cmd = DeclareLaunchArgument(
        'using_namespace', 
        default_value='False',
        description='Namespaces all topics (best for multiple robot setups) if set to true'
    )
    
    declare_using_localization_cmd = DeclareLaunchArgument(
        'using_localization',
        default_value='True', #Do not change to false, or SLAM won't work
        description='A "True" value is required when using slam or you wont get accurate map'
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

    
    # ======================= Cartographer File Path Setup (If using) ==============================#   

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

    # =========================== Start ROS2 Control Nodes ============================= #

    # 1. Micro-ROS Agent
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyACM0', '-b', '115200'],
        output='screen'
    )

    # 2. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )
    
    # 3. Twist Mux Node (loads twist_mux.yaml)
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_file],
        remappings=[("cmd_vel_out", "/cmd_vel")] 
    )

    # 4. Twist Stamper (listens to twist_mux)
    twist_stamper_node = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='twist_stamper',
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),
            ('cmd_vel_out', '/cmd_vel_stamped')
        ]
    )

    # 5. Control Node (listens to stamper)
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_file],
        output='both',
        remappings=[
            ('~/robot_description', '/robot_description'),
            ('/diff_drive_controller/odom', '/odom'),
            ('/diff_drive_controller/cmd_vel', '/cmd_vel_stamped') 
        ],
    )

    # 6. Spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
    )

    imu_sensor_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # Delay spawners to give the controller manager time to load the hardware interface
    delayed_spawners = TimerAction(
        period=3.0,
        actions=[
            joint_state_broadcaster_spawner,
            diff_drive_spawner,
            imu_sensor_spawner
        ]
    )

    
    # ================================================================================== #

    # =========================== Start Localization (EKF) ============================= #
    
    start_ekf_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slambot_localization, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'using_namespace': LaunchConfiguration('using_namespace'),
            'ekf_param_file': ekf_real_params,
            'ekf_param_file_namespaced': ekf_real_params_namespaced
        }.items(),
        condition=IfCondition(LaunchConfiguration('using_localization'))
    )

    # ================================================================================== #


    # ============== Start slamtoolbox (the default argument) ================= #

    # 1. If NOT using namespacing...
    start_slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slambot_slam, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        #Below tells this NOT to launch if 'using_namespace' set to True
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('slam_type'), "'.lower() == 'slamtoolbox' and '",
            LaunchConfiguration('using_namespace'), "'.lower() != 'true'"
        ]))
    )

    # 1. If using namespacing...
    start_slam_cmd_namespaced = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slambot_slam, 'launch', 'slam_namespaced.launch.py')
        ),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        #Below tells this NOT to launch if 'using_namespace' set to False
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('slam_type'), "'.lower() == 'slamtoolbox' and '",
            LaunchConfiguration('using_namespace'), "'.lower() == 'true'"
        ]))
    )
    
    # ================================================================================ # 

    # ============== Start cartographer(only if selected as argument) ================= #

    start_cartographer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slambot_slam, 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'cartographer_config_file': LaunchConfiguration('cartographer_config_file'),
            'configuration_directory': LaunchConfiguration('configuration_directory'), 
            'using_namespace': LaunchConfiguration('using_namespace'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('slam_type'), "'.lower() == 'cartographer'"
        ]))
    )
    
    # ================================================================================ # 

    # =================== Launch Teleop Nodes Here =================== #


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
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_using_namespace_cmd)
    ld.add_action(declare_using_localization_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_slam_type_cmd)
    ld.add_action(declare_configuration_directory_cmd)
    ld.add_action(declare_cartographer_config_file_cmd)
    ld.add_action(declare_using_joy_cmd)
    
    ld.add_action(micro_ros_agent)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(twist_mux_node)
    ld.add_action(twist_stamper_node)
    ld.add_action(control_node)
    ld.add_action(delayed_spawners)
    ld.add_action(start_ekf_localization_cmd)
    ld.add_action(start_slam_cmd)
    ld.add_action(start_slam_cmd_namespaced)
    ld.add_action(start_cartographer_cmd)
    ld.add_action(joy_node)
    ld.add_action(teleop_twist_joy_node)

    return ld