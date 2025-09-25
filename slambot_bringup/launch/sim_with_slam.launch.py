#!/usr/bin/env python3
"""
Top-level launch file to start the Gazebo simulation WITH SLAM.

This launch file acts as an entry point, bringing up the complete simulation
environment (Gazebo, RViz, Localization) and also starting the SLAM Toolbox
node to enable map creation. It also handles the lifecycle management of the
SLAM node.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    """Generate the launch description for the simulation with SLAM."""

    # Get paths to the other packages
    pkg_slambot_gazebo = get_package_share_directory('slambot_gazebo')
    pkg_slambot_slam = get_package_share_directory('slambot_slam')

    # --- Declare Launch Arguments ---
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='indoor_world_1.sdf',
        description='The world file to launch in Gazebo'
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='slambot',
        description='The name/namespace for the robot'
    )

    #Namespaces the /tf topics when using Nav2
    declare_using_nav2_cmd = DeclareLaunchArgument(
        'using_nav_2', 
        default_value='false',
        description='Namespaces the /tf topics if set to true')

    # This argument allows us to specify the SLAM params file from the command line
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_slambot_slam, 'config', 'slam_params.yaml'),
        description='Full path to the ROS2 parameters file for SLAM'
    )

    # Path to the RViz configuration file
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(pkg_slambot_gazebo, 'rviz', 'gazebo_rviz_and_slam_config.rviz'),
        description='Full path to the RViz configuration file to use')


    # --- Include Gazebo Launch File ---
    # This includes the simulation environment (Gazebo, Localization, RViz)
    start_simulation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slambot_gazebo, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'robot_name': LaunchConfiguration('robot_name'),
            'rviz_config_file': LaunchConfiguration('rviz_config_file'), # Pass the config file down
            'using_nav_2': LaunchConfiguration('using_nav_2')
        }.items()
    )

    # --- SLAM Toolbox Node ---
    # We launch the SLAM node as a LifecycleNode to manage its state
    start_slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
          LaunchConfiguration('slam_params_file'),
          {'use_sim_time': True} # <-- CRITICAL
        ],
        remappings=[
            ('scan', [LaunchConfiguration('robot_name'), '/scan']),
            ('map', [LaunchConfiguration('robot_name'), '/map']) 
        ]
    )

    # --- Lifecycle Management for SLAM Node ---
    # This sequence automates the manual "ros2 lifecycle set" commands.

    # 1. After a 3-second delay, issue the "configure" transition
    configure_slam_node = TimerAction(
        period=3.0,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_slam_toolbox_node),
                    transition_id=Transition.TRANSITION_CONFIGURE
                )
            )
        ]
    )

    # 2. Once the node transitions to "inactive" (after configuring),
    #    wait another 3 seconds and issue the "activate" transition.
    activate_slam_node = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                TimerAction(
                    period=3.0,
                    actions=[
                        EmitEvent(
                            event=ChangeState(
                                lifecycle_node_matcher=matches_action(start_slam_toolbox_node),
                                transition_id=Transition.TRANSITION_ACTIVATE
                            )
                        )
                    ]
                )
            ]
        )
    )

    # --- Create Launch Description ---
    ld = LaunchDescription()

    # Add the declared arguments
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_using_nav2_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    
    # Add the actions to launch and manage the nodes
    ld.add_action(start_simulation_cmd)
    ld.add_action(start_slam_toolbox_node)
    ld.add_action(configure_slam_node)
    ld.add_action(activate_slam_node)

    return ld

