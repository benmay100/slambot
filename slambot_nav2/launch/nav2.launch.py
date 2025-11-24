import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import SetRemap


def generate_launch_description():
    # 1. Get the path to the official Nav2 bringup launch file
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slambot_navigation_dir = get_package_share_directory('slambot_nav2')
    
    # 2. File paths
    map_file_path = os.path.join(slambot_navigation_dir, 'maps', 'indoor_map_cartographer.yaml')
    params_file_path = os.path.join(slambot_navigation_dir, 'config', 'nav2_custom_params.yaml')


    # 3. Declare the launch arguments
    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_file_path,
        description='Full path to the map file to load')

    declare_params_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=params_file_path,
        description='Full path to the Nav2 parameters file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true', # <-- Set to true for simulation
        description='Use simulation (Gazebo) clock if true')
    


    # ============== Nav2 Launch (for sim, so uses /cmd_vel) =======================#

    start_nav2_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            # Arguments for bringup_launch.py
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_sim_time'))
    )

    # ================================================================================= #
    

    # ============== Nav2 Launch (for real robot, so remaps /cmd_vel to /cmd_vel_nav) =======================#

    start_nav2_real_cmd = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    # Arguments for bringup_launch.py
                    'map': LaunchConfiguration('map'),
                    'params_file': LaunchConfiguration('params_file'),
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }.items()
            ),
        ],
        condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
    )

    # ====================================================================================================== #


    # --- Create Launch Description ---
    ld = LaunchDescription()

    # Add the declared arguments and the include action
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_params_cmd)
    ld.add_action(declare_use_sim_time_cmd)  
    ld.add_action(start_nav2_sim_cmd)
    ld.add_action(start_nav2_real_cmd)

    return ld