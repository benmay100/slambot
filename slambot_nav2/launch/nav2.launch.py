import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Get the path to the official Nav2 bringup launch file
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

    # 2. Define the path to YOUR map file
    # Make sure 'my_map.yaml' is in the 'maps' directory of your package
    map_file_path = PathJoinSubstitution([
        FindPackageShare('slambot_nav2'), # <-- Change to your package name
        'maps',
        'my_maze_map_1.yaml' # <-- This is a default, if you want a different map, change it from launch file in slambot_bringup
    ])

    # 3. Define the path to YOUR custom nav2_params.yaml
    params_file_path = PathJoinSubstitution([
        FindPackageShare('slambot_nav2'), # <-- Change to your package name
        'config',
        'nav2_params.yaml' # <-- Your custom params file
    ])

    # 4. Declare the launch arguments
    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_file_path,
        description='Full path to the map file to load')

    declare_params_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=params_file_path,
        description='Full path to the Nav2 parameters file')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='slambot', # <-- Set your desired namespace here
        description='Top-level namespace for the Nav2 stack')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true', # <-- Set to true for simulation
        description='Use simulation (Gazebo) clock if true')

    # 5. Create the IncludeLaunchDescription action to call bringup_launch.py
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            # Arguments for bringup_launch.py
            'slam': 'False',               # We are not doing SLAM, we are navigating
            'use_localization': 'True',    # We want to use AMCL for localization
            'map': LaunchConfiguration('map'),
            'namespace': LaunchConfiguration('namespace'),
            'use_namespace': 'True',       # THIS IS THE KEY: It activates PushROSNamespace
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        declare_map_cmd,
        declare_params_cmd,
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        start_nav2_cmd
    ])