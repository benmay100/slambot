#!/usr/bin/env python3
"""
Launch a Gazebo simulation for the Slambot.

This launch file orchestrates the entire simulation, including:
1. Starting the Gazebo server and GUI using the standard ros_gz_sim launch file.
2. Launching the ROS-Gazebo bridge for communication.
3. Spawning the robot model into the simulation.
4. Including the separate RViz launch file for visualization.
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node

# Pre-define the package share directory path for use in default arguments
pkg_slambot_gazebo = get_package_share_directory('slambot_gazebo')

ARGUMENTS = [
    # --- Basic Arguments ---
    DeclareLaunchArgument('world', default_value='maze_world_1.sdf', 
                          #change world name here for custom worlds 
                          # OR change them in the terminal at launch (e.g. ros2 launch slambot_gazebo gazebo.launch.py world:=maze_world_1.sdf)
                          description='The world file to launch in Gazebo'),
    DeclareLaunchArgument('robot_name', default_value='slambot',
                          description='The name/namespace for the robot'),
    DeclareLaunchArgument('use_rviz', default_value='true',
                          description='Whether to start RViz'),
    DeclareLaunchArgument('headless', default_value='False',
                          description='Whether to run Gazebo without a GUI'),

    # --- File Name Arguments (for modularity) ---
    DeclareLaunchArgument('bridge_config', default_value='ros_gz_bridge.yaml',
                          description='ROS-Gazebo bridge config file'),
    # This new argument specifies the RViz config file for the Gazebo simulation
    DeclareLaunchArgument('rviz_config',
                          default_value=os.path.join(pkg_slambot_gazebo, 'rviz', 'gazebo_and_rviz_config.rviz'),
                          description='Full path to the RViz config file to use'),

    # --- Spawning Arguments ---
    DeclareLaunchArgument('spawn_x', default_value='0.0',
                          description='X coordinate for robot spawn position'),
    DeclareLaunchArgument('spawn_y', default_value='0.0',
                          description='Y coordinate for robot spawn position'),
    DeclareLaunchArgument('spawn_z', default_value='0.2',
                          description='Z coordinate for robot spawn position'),
]

def generate_launch_description():

    # --- DIRECTORY SETUP ---
    # We already got pkg_slambot_gazebo outside the function for the arguments
    pkg_slambot_description = get_package_share_directory('slambot_description')
    pkg_slambot_localization = get_package_share_directory('slambot_localization')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    parent_of_share_path = os.path.dirname(pkg_slambot_description)

    # --- Set GZ_SIM_RESOURCE_PATH  ---
    # This is critical to allow Gazebo to find the meshes from the slambot_description package
    set_gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=[
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            os.pathsep,
            parent_of_share_path
        ]
    )

    # --- Set GAZEBO_MODEL_PATH ---
    # This is handled from inside the package.xml file - see near the bottom of the file some important exports!


    # --- GAZEBO ---
    world_path = PathJoinSubstitution([pkg_slambot_gazebo, 'worlds', LaunchConfiguration('world')])

    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': ['-r -s ', world_path]}.items()
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': '-g'}.items(),
        condition=IfCondition(PythonExpression(['not ', LaunchConfiguration('headless')]))
    )


    # --- ROS-GAZEBO BRIDGE ---
    bridge_config_path = PathJoinSubstitution([pkg_slambot_gazebo, 'config', LaunchConfiguration('bridge_config')])
    start_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config_path}],
        # Remap all necessary topics from the bridge to be namespaced
        # You may want to leave some not namespaced (e.g. /cmd_vel if its being used by a teleop node)
        remappings=[
            ('/scan', [LaunchConfiguration('robot_name'), '/scan']),
            ('/imu/data', [LaunchConfiguration('robot_name'), '/imu/data']),
            ('/camera/image_raw', [LaunchConfiguration('robot_name'), '/camera/image_raw']),
            ('/camera/camera_info', [LaunchConfiguration('robot_name'), '/camera/camera_info']),
            ('/odom', [LaunchConfiguration('robot_name'), '/odom']),
            ('/joint_states', [LaunchConfiguration('robot_name'), '/joint_states']),
        ],
        output='screen'
    )

    # --- RVIZ & NAMESPACED ROBOT STATE PUBLISHER ---
    launch_rviz_and_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_slambot_description, 'launch', 'rviz.launch.py'])
        ),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_sim_time': 'true',
            'rviz_config': LaunchConfiguration('rviz_config'), # Pass the selected config to the RViz launch file
            'jsp_gui': 'false'  # <-- Disables joint state publisher in RVIZ when we're using gazebo
        }.items()
    )

    
    # --- EKF LOCALIZATION NODE ---  <-- ADDED SECTION
    launch_ekf_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_slambot_localization, 'launch', 'localization.launch.py'])
        ),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
        }.items()
    )

    # --- ROBOT SPAWNING ---
    spawn_robot_into_gazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),
            '-topic', [LaunchConfiguration('robot_name'), '/robot_description']
        ],
        output='screen'
    )

    # --- CREATE LAUNCH DESCRIPTION ---
    ld = LaunchDescription(ARGUMENTS)
    
    # Add actions
    ld.add_action(set_gz_sim_resource_path)
    ld.add_action(gz_sim_server)
    ld.add_action(gz_sim_gui)
    ld.add_action(start_ros_gz_bridge)
    ld.add_action(launch_rviz_and_description)
    ld.add_action(launch_ekf_localization)
    ld.add_action(spawn_robot_into_gazebo)

    return ld

