
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

# ================== ENVIRONMENT SETUP =================== #


    #CHANGE THESE TO BE RELEVANT TO THE SPECIFIC PACKAGE
    robot_description_path = get_package_share_directory('slambot_description')  # -----> Change me!
    robot_package = FindPackageShare('slambot_description') # -----> Change me!
    robot_name = 'slambot' # Verify this matches your robot's actual spawned name/tf_prefix
    robot_urdf_file_name = 'slambot.urdf.xacro'
    rviz_config_file_name = 'gazebo_and_rviz_config.rviz'

    parent_of_share_path = os.path.dirname(robot_description_path)

    # --- Set GZ_SIM_RESOURCE_PATH / GAZEBO_MODEL_PATH ---
    set_gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=[
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            os.path.pathsep, # Separator for paths
            parent_of_share_path # Add the path containing your package's share directory
        ]
    )

    # --- Use sim time setup ---
    use_sim_time_declare = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')




# ========================================================= #


# ======================== RVIZ ========================== #

    # Declare arguments
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=PathJoinSubstitution([
            robot_package,
            'urdf',
            robot_urdf_file_name
        ]),
        description='Path to the URDF file for the robot description.'
    )

    rviz_config_path_arg = DeclareLaunchArgument(
        'rviz_config_path',
        default_value=PathJoinSubstitution([
            robot_package,
            'config',
            rviz_config_file_name
        ]),
        description='Path to the RViz configuration file.'
    )

    # Get the robot description from the URDF file
    robot_description_content = ParameterValue(
        Command(['xacro ', LaunchConfiguration('urdf_path')]),
        value_type=str
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    # RViz2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_path')],
        parameters=[{'use_sim_time': use_sim_time}] 
    )

# ========================================================= #


# ============== GAZEBO - SETUP AND LAUNCH ================ #

    # Include the Gazebo Sim launch file (using gz_sim.launch.py)
    gz_sim_launch_file = PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch_file]),
        launch_arguments={
                        'gz_args': '-r empty.sdf',
                        'use_sim_time': use_sim_time,
                        'on_exit_shutdown': 'True'            
                        }.items() # Use -r for 'run' and loads the world.
    )

    # Reads the robot_description from the parameter server and spawns it.
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name, 
            '-topic', 'robot_description', # Read URDF from /robot_description topic
            '-x', '0', # Default spawn location
            '-y', '0',
            '-z', '0.5'
        ],
        output='screen'
    )

# ========================================================= #


# ================= GAZEBO BRIDGES & SENSOR SETUP =================== #

    bridge_config_file = os.path.join(robot_description_path, 'config', 'gazebo_bridge.yaml')

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[
            {'config_file': bridge_config_file}
        ],
        output='screen'
    )    

    # New Node: Static Transform Publisher for map to odom
    # This places your robot's 'odom' frame at the origin of the 'map' frame.
    # Can remove if using Nav2/Slam toolbot
    map_odom_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_broadcaster',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', f'{robot_name}/odom']
    )


# ========================================================= #

    return LaunchDescription([
        urdf_path_arg,
        rviz_config_path_arg,
        use_sim_time_declare,
        set_gz_sim_resource_path, # This must come before any nodes that rely on it
        map_odom_publisher_node,
        robot_state_publisher_node,
        gazebo_launch,
        spawn_entity_node,
        ros_gz_bridge,
        rviz2_node
        ])

