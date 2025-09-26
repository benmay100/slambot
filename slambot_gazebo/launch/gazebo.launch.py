#!/usr/bin/env python3
"""
Launch a Gazebo simulation for the Slambot.

This launch file orchestrates the entire simulation, including:
1. Starting Gazebo with a specified world.
2. Launching the ROS-Gazebo bridge for communication.
3. Spawning the robot model into the simulation.
4. Including separate launch files for RViz, robot description, and localization.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

def generate_launch_description():

    # ================== Get Package Directories =================== #
    
    pkg_slambot_description = get_package_share_directory('slambot_description')
    pkg_slambot_gazebo = get_package_share_directory('slambot_gazebo')
    pkg_slambot_localization = get_package_share_directory('slambot_localization')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # ================== Declare Launch Arguments =================== #

    # World to launch in Gazebo
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='indoor_world_1.sdf',
        description='The world file to launch in Gazebo')

    # Robot's name and namespace
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='slambot',
        description='The name/namespace for the robot')

    # Whether to run Gazebo without a GUI
    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to run Gazebo without a GUI')
    
    #Namespaces the /tf topics when using Nav2
    declare_using_namespace_cmd = DeclareLaunchArgument(
        'using_namespace', 
        default_value='False',
        description='Namespaces all topics (best for multiple robot setups) if set to true')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_jsp_gui_cmd = DeclareLaunchArgument(
        'jsp_gui', 
        default_value='False',
        description='Flag to enable joint_state_publisher_gui')

    # Path to the RViz configuration file
    rviz_config_path = os.path.join(pkg_slambot_gazebo, 'rviz', 'gazebo_and_rviz_config.rviz')
    rviz_config_path_namespaced = os.path.join(pkg_slambot_gazebo, 'rviz', 'gazebo_and_rviz_config_namespaced.rviz')

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


    # ================== Set Environment Variables =================== #
    
    # This is critical to allow Gazebo to find the meshes from the slambot_description package
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
    
    # --- Set GAZEBO_MODEL_PATH --- this is handled from inside the package.xml file - see near the bottom of the file some important exports!


    # ================== Start Gazebo Simulation =================== #

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
        #Below tells gz_sim_gui not to launch if 'headless' set to True
        condition=IfCondition(PythonExpression(['not ', LaunchConfiguration('headless')]))
    )

    # ================== Start ROS-Gazebo Bridge =================== #
    
    #If not using namespacing (we don't remap any topics)
    start_ros_gz_bridge_cmd = Node(
        condition=UnlessCondition(LaunchConfiguration('using_namespace')),
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([
                pkg_slambot_gazebo, 'config', 'ros_gz_bridge.yaml'
            ])
        }],
        output='screen'
    )

    #If using namespacing (we remap topics to 'slambot/...')
    start_ros_gz_bridge_namespaced_cmd = Node(
        condition=IfCondition(LaunchConfiguration('using_namespace')),
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([
                pkg_slambot_gazebo, 'config', 'ros_gz_bridge.yaml'
            ])
        }],
        remappings=[
            ('/scan', [LaunchConfiguration('robot_name'), '/scan']),
            ('/imu/data', [LaunchConfiguration('robot_name'), '/imu/data']),
            ('/camera/image_raw', [LaunchConfiguration('robot_name'), '/camera/image_raw']),
            ('/camera/camera_info', [LaunchConfiguration('robot_name'), '/camera/camera_info']),
            ('/odom', [LaunchConfiguration('robot_name'), '/odom']),
            ('/joint_states', [LaunchConfiguration('robot_name'), '/joint_states']),
            ('/cmd_vel', [LaunchConfiguration('robot_name'), '/cmd_vel']),
            ('/tf', [LaunchConfiguration('robot_name'), '/tf']), # <--- this is important
        ],
        output='screen'
    )
    
    # ================== Start Robot Description & RViz =================== #
    
    start_rviz_and_description_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slambot_description, 'launch', 'rviz.launch.py')
        ),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rviz_config_file': LaunchConfiguration('rviz_config_file'), # Pass the config file down
            'jsp_gui': LaunchConfiguration('jsp_gui'),
            'using_namespace': LaunchConfiguration('using_namespace')
        }.items()
    )

    # ================== Start Localization (EKF) =================== #
    
    start_ekf_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slambot_localization, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'using_namespace': LaunchConfiguration('using_namespace')
        }.items()
    )

    # ================== Spawn Robot into Gazebo =================== #
    
    spawn_robot_cmd = Node(
        condition=UnlessCondition(LaunchConfiguration('using_namespace')),
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2',
            '-topic', 'robot_description' 
        ],
        output='screen'
    )

    spawn_robot_namespaced_cmd = Node(
        condition=IfCondition(LaunchConfiguration('using_namespace')),
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2',
            '-topic', [LaunchConfiguration('robot_name'), '/robot_description']
        ],
        output='screen'
    )

    # ================== Create Launch Description =================== #
    
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_using_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Add actions
    ld.add_action(set_gz_sim_resource_path)
    ld.add_action(gz_sim_server)
    ld.add_action(gz_sim_gui)
    ld.add_action(start_ros_gz_bridge_cmd)
    ld.add_action(start_ros_gz_bridge_namespaced_cmd)
    ld.add_action(start_rviz_and_description_cmd)
    ld.add_action(start_ekf_localization_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(spawn_robot_namespaced_cmd)

    return ld