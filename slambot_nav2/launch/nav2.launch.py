import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():
    # 1. Get the path to the official Nav2 bringup launch file
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slambot_navigation_dir = get_package_share_directory('slambot_nav2')
    
    # 2. File paths
    map_file_path = os.path.join(slambot_navigation_dir, 'maps', 'indoor_map_cartographer.yaml')
    params_file_path = os.path.join(slambot_navigation_dir, 'config', 'nav2_custom_params.yaml')
    params_file_path_namespaced = os.path.join(slambot_navigation_dir, 'config', 'nav2_custom_params_namespaced.yaml')


    # 3. Declare the launch arguments
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='slambot',
        description='The name/namespace for the robot'
    )

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_file_path,
        description='Full path to the map file to load')

    # declare_params_cmd = DeclareLaunchArgument(
    #     'params_file',
    #     default_value=params_file_path,
    #     description='Full path to the Nav2 parameters file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true', # <-- Set to true for simulation
        description='Use simulation (Gazebo) clock if true')
    
    #Namespaces the /tf topics when using Nav2
    declare_using_namespace_cmd = DeclareLaunchArgument(
        'using_namespace', 
        default_value='False',
        description='Namespaces all topics (best for multiple robot setups) if set to true'
    )

    # Dynamic change of params file depending on if using namespace
    declare_params_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PythonExpression([
            "'", params_file_path_namespaced, "' if '",
            LaunchConfiguration('using_namespace'),
            "'.lower() == 'true' else '", params_file_path, "'"
        ]),
        description='Full path to the Nav2 parameters file'
    )
    


    # ============== Nav2 Launch  =======================#

    #1. If NOT using namespaces
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            # Arguments for bringup_launch.py
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items(),
        condition=IfCondition(PythonExpression(['not ', LaunchConfiguration('using_namespace')]))
    )

    #2. If using namespaces
    start_nav2_cmd_namespaced = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            # Arguments for bringup_launch.py
            'map': LaunchConfiguration('map'),
            'namespace': LaunchConfiguration('robot_name'),
            'use_namespace': 'True',       # THIS IS THE KEY: It activates PushROSNamespace
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(PythonExpression([' ', LaunchConfiguration('using_namespace')]))
    )


    # --- Create Launch Description ---
    ld = LaunchDescription()

    # Add the declared arguments and the include action
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_params_cmd)
    ld.add_action(declare_use_sim_time_cmd)  
    ld.add_action(declare_using_namespace_cmd)
    ld.add_action(start_nav2_cmd)
    ld.add_action(start_nav2_cmd_namespaced)

    return ld