#!/usr/-bin/env python3
"""
Launch file for the SLAM Toolbox.

This launch file starts the slam_toolbox node in asynchronous mode, which is suitable
for creating maps from laser scan data. It is designed to be included in a
higher-level launch file.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            RegisterEventHandler, TimerAction)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """Generate the launch description for the SLAM Toolbox node."""

    # Get the path to this package's share directory
    pkg_slambot_slam = get_package_share_directory('slambot_slam')

    # --- Declare Launch Arguments ---
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_slambot_slam, 'config', 'slam_params.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )

    # This argument allows the parent launch file to specify the robot's namespace
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='slambot',
        description='The namespace of the robot'
    )


    # =================== SLAM Toolbox Node =================== #
 
    # We launch the SLAM node as a LifecycleNode to manage its state
    start_slam_toolbox_node_namespaced = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='', #namespacing doesn't work, so need to just remap topics instead
        output='screen',
        parameters=[
          LaunchConfiguration('slam_params_file'),
          {'use_sim_time': True} # <-- CRITICAL
        ],
        remappings=[
            ('scan', [LaunchConfiguration('robot_name'), '/scan']),
            ('map', [LaunchConfiguration('robot_name'), '/map']),
            ('tf', [LaunchConfiguration('robot_name'), '/tf']),
            ('tf_static', [LaunchConfiguration('robot_name'), '/tf_static'])
        ]
    )

    # --- Lifecycle Management for SLAM Node ---
    # This sequence automates the manual "ros2 lifecycle set" commands.

    # 1. After a 3-second delay, issue the "configure" transition
    configure_slam_node_namespaced = TimerAction(
        period=3.0,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_slam_toolbox_node_namespaced),
                    transition_id=Transition.TRANSITION_CONFIGURE
                )
            )
        ]
    )

    # 2. Once the node transitions to "inactive" (after configuring),
    #    wait another 3 seconds and issue the "activate" transition.
    activate_slam_node_namespaced = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_slam_toolbox_node_namespaced,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                TimerAction(
                    period=3.0,
                    actions=[EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(start_slam_toolbox_node_namespaced),transition_id=Transition.TRANSITION_ACTIVATE))]
                )
            ]
        )
    )

    # --- Create Launch Description ---
    ld = LaunchDescription()

    # Add the launch arguments to the launch description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_robot_name_cmd)

    # Add the slam_toolbox node to the launch description
    ld.add_action(start_slam_toolbox_node_namespaced)
    ld.add_action(configure_slam_node_namespaced)
    ld.add_action(activate_slam_node_namespaced)

    return ld
