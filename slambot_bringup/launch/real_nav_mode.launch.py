#!/usr/bin/env python3
"""
Top-level launch file to start the REAL robot with SLAM, localization and EKF sensor fusion.
You can choose which version of slam you want to use 'cartographer_ros' or 'slam_toolbox'....
...depending on preference and which works best in the environment you're mapping

"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command, FindExecutable
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ========================= Paths & Environment Setup =========================== #   

    # 1. Get the paths to the required packages
    pkg_slambot_description = get_package_share_directory('slambot_description')
    pkg_slambot_bringup = get_package_share_directory('slambot_bringup')
    pkg_slambot_localization = get_package_share_directory('slambot_localization')
    pkg_ldlidar_ros2 = get_package_share_directory('ldlidar_ros2')
    pkg_slambot_nav2 = get_package_share_directory('slambot_nav2')
    pkg_slambot_slam = get_package_share_directory('slambot_slam')

    # 2. Define file paths
    xacro_file = os.path.join(pkg_slambot_description, 'urdf', 'slambot.urdf.xacro')
    controllers_file = os.path.join(pkg_slambot_bringup, 'config', 'slambot_controllers.yaml')
    twist_mux_file = os.path.join(pkg_slambot_bringup, 'config', 'twist_mux.yaml') 
    joy_config_file = os.path.join(pkg_slambot_bringup, 'config', 'joy_teleop.yaml') 
    ekf_real_params = os.path.join(pkg_slambot_localization, 'config', 'ekf_real.yaml')
    camera_config_file = os.path.join(pkg_slambot_bringup, 'config', 'camera.yaml')

    # 3. Process URDF
    robot_description_content = Command(
        [FindExecutable(name='xacro'), ' ', xacro_file, ' ', 'using_sim:=false']
    )
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # ========================= Declare Launch Arguments =========================== #   

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='MUST BE SET TO FALSE FOR REAL ROBOT!'
    )

    declare_using_joy_cmd = DeclareLaunchArgument(
        'using_joy',
        default_value='True',
        description='launches the joystick teleop nodes if true.'
    )

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_slambot_slam, 'maps', 'campion_bedroom_2.yaml'),
        description='Full path to the map file to load for navigation'
    )

    declare_nav_params_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_slambot_nav2, 'config', 'nav2_real_params.yaml'), #Makes sure to send into Nav the 'real' params file, not the sim one
        description='Full path to the navigation parameters file to load'
    )



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
    
    # 3. Twist Mux Node
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_file],
        remappings=[("cmd_vel_out", "/cmd_vel")] 
    )

    # 4. Twist Stamper
    twist_stamper_node = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='twist_stamper',
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),
            ('cmd_vel_out', '/cmd_vel_stamped')
        ]
    )

    # 5. Control Node
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

    delayed_spawners = TimerAction(
        period=3.0,
        actions=[
            joint_state_broadcaster_spawner,
            diff_drive_spawner,
            imu_sensor_spawner
        ]
    )

    # =========================== Start Sensors ============================= #
    
    start_lidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ldlidar_ros2, 'launch', 'ld06.launch.py')
        )
    )

    # Note needed for Nav2, and better to leave out to reduce CPU load on the real robot
    # camera_node = Node(
    #     package='camera_ros',
    #     executable='camera_node',
    #     name='camera',
    #     output='screen',
    #     parameters=[camera_config_file]
    # )

    # ======================================================================= #

    # =========================== Start Localization (EKF) ============================= #
    
    start_ekf_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slambot_localization, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'ekf_param_file': ekf_real_params,
        }.items(),
    )

    # ================================================================================== #


    # ============== Include Nav2 Launch File & Launch =======================#

    # Note the real robot nav2 launch includes a remap of /cmd_vel to /cmd_vel_nav but this is handled by the nav2.launch.py file 
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slambot_nav2, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
        }.items()
    )

    cmd_vel_watchdog_node = Node(
        package='slambot_nav2',
        executable='cmd_vel_watchdog',
        name='cmd_vel_watchdog',
        output='screen',
        parameters=[{
            'cmd_vel_topic': '/cmd_vel_nav',
            'status_topics': [
                '/navigate_to_pose/_action/status',
                '/navigate_through_poses/_action/status'
            ],
            'linear_threshold': 0.01,
            'angular_threshold': 0.01,
            'hold_stop_duration': 0.75,
            'log_throttle_seconds': 2.0,
        }]
    )

    # ================================================================================ # 
    
    # =================== Launch Teleop Nodes Here =================== #

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'dev': '/dev/input/js0'}],
        condition=IfCondition(LaunchConfiguration('using_joy'))
    )

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_config_file],
        remappings=[('cmd_vel', '/cmd_vel_joy')],
        condition=IfCondition(LaunchConfiguration('using_joy'))
    )

    # ========================= Create Launch Description ============================ # 
    
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_using_joy_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_nav_params_cmd)
    
    # Add Nodes
    ld.add_action(micro_ros_agent)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(twist_mux_node)
    ld.add_action(twist_stamper_node)
    ld.add_action(control_node)
    ld.add_action(delayed_spawners)
    ld.add_action(start_nav2_cmd)
    ld.add_action(cmd_vel_watchdog_node)
    
    # Add Sensor Launch
    ld.add_action(start_lidar_cmd)
    # ld.add_action(camera_node)
    
    # Add Logic
    ld.add_action(start_ekf_localization_cmd)
    ld.add_action(joy_node)
    ld.add_action(teleop_twist_joy_node)

    return ld