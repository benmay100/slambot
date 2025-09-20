
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
    rviz_config_file_name = 'slambot_config.rviz'

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
            'rviz',
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
            'use_sim_time': use_sim_time,
            'frame_prefix': robot_name + '/' 
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

    #Joint State Publisher GUI node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )


# ========================================================= #

    return LaunchDescription([
        urdf_path_arg,
        rviz_config_path_arg,
        use_sim_time_declare,
        set_gz_sim_resource_path, # This must come before any nodes that rely on it
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node
        ])

