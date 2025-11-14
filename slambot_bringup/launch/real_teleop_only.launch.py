import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # --- LAUNCH ARGUMENTS ---
    
    # ADDED: Argument to select teleop mode
    using_joy_arg = DeclareLaunchArgument(
        'using_joy',
        default_value='true',
        description='Set to true to launch joystick teleop nodes, false to use keyboard.'
    )

    # --- CONFIGURATION ---
    
    # 1. Get package paths
    description_pkg_path = get_package_share_directory('slambot_description')
    bringup_pkg_path = get_package_share_directory('slambot_bringup')
    
    # 2. Define file paths
    xacro_file = os.path.join(description_pkg_path, 'urdf', 'slambot.urdf.xacro')
    controllers_file = os.path.join(bringup_pkg_path, 'config', 'slambot_controllers.yaml')
    
    # ADDED: Path to the new twist_mux config
    twist_mux_file = os.path.join(bringup_pkg_path, 'config', 'twist_mux.yaml')
    
    # ADDED: Path for the joystick config
    joy_config_file = os.path.join(bringup_pkg_path, 'config', 'joy_teleop.yaml')

    # 3. Process URDF
    robot_description_content = Command(
        [FindExecutable(name='xacro'), ' ', xacro_file, ' ', 'using_sim:=false']
    )
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # --- NODES ---

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

    # --- ADDED: CONDITIONAL JOYSTICK NODES ---
    
    # Create the condition
    joy_condition = IfCondition(LaunchConfiguration('using_joy'))

    # 7. Start the 'joy' driver node, *if* using_joy is true
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'dev': '/dev/input/js0'}],
        condition=joy_condition # <-- This is the magic
    )

    # 8. Start the 'teleop_twist_joy' node, *if* using_joy is true
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_config_file],
        remappings=[
            # Remap the output to the topic twist_mux is listening for
            ('cmd_vel', '/cmd_vel_joy') 
        ],
        condition=joy_condition # <-- This is the magic
    )

    # --- RETURN LAUNCH DESCRIPTION ---
    return LaunchDescription([
        # Add the new launch argument
        using_joy_arg,
        
        # Add the core robot nodes
        micro_ros_agent,
        robot_state_publisher_node,
        control_node,
        twist_mux_node,
        twist_stamper_node,
        delayed_spawners,
        
        # ADDED: Add the new conditional joystick nodes
        joy_node,
        teleop_twist_joy_node
    ])