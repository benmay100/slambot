#!/usr/bin/env python3
"""Launch RViz on a dev machine while the real robot runs teleop."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
	pkg_slambot_bringup = get_package_share_directory('slambot_bringup')

	rviz_config_path = os.path.join(pkg_slambot_bringup, 'rviz', 'dev_rviz_teleop_config.rviz')
	rviz_config_path_namespaced = os.path.join(pkg_slambot_bringup, 'rviz', 'dev_rviz_teleop_config_namespaced.rviz')

	declare_robot_name_cmd = DeclareLaunchArgument(
		'robot_name',
		default_value='slambot',
		description='Name/namespace used by the real robot.'
	)

	declare_use_sim_time_cmd = DeclareLaunchArgument(
		'use_sim_time',
		default_value='false',
		description='Set to true only when replaying bags with simulated time.'
	)

	declare_using_namespace_cmd = DeclareLaunchArgument(
		'using_namespace',
		default_value='False',
		description='Match the namespace setting used on the robot.'
	)

	declare_using_localization_cmd = DeclareLaunchArgument(
		'using_localization',
		default_value='True',
		description='Select RViz panels that expect localization outputs.'
	)

	declare_rviz_config_file_cmd = DeclareLaunchArgument(
		'rviz_config_file',
		default_value=PythonExpression([
			"('",
			rviz_config_path_namespaced,
			"' if '",
			LaunchConfiguration('using_namespace'),
			"'.lower() == 'true' else '",
			rviz_config_path,
			"')"
		]),
		description='RViz configuration file to mirror the simulation layout.'
	)

	rviz_node = Node(
		condition=UnlessCondition(LaunchConfiguration('using_namespace')),
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		output='screen',
		arguments=['-d', LaunchConfiguration('rviz_config_file')],
		parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
	)

	rviz_node_namespaced = Node(
		condition=IfCondition(LaunchConfiguration('using_namespace')),
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		output='screen',
		arguments=['-d', LaunchConfiguration('rviz_config_file')],
		parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
		remappings=[
			('/tf', [LaunchConfiguration('robot_name'), '/tf']),
			('/tf_static', [LaunchConfiguration('robot_name'), '/tf_static']),
			('/initialpose', [LaunchConfiguration('robot_name'), '/initialpose']),
			('/goal_pose', [LaunchConfiguration('robot_name'), '/goal_pose']),
			('/clicked_point', [LaunchConfiguration('robot_name'), '/clicked_point']),
			('/waypoints', [LaunchConfiguration('robot_name'), '/waypoints'])
		]
	)

	ld = LaunchDescription()
	ld.add_action(declare_robot_name_cmd)
	ld.add_action(declare_use_sim_time_cmd)
	ld.add_action(declare_using_namespace_cmd)
	ld.add_action(declare_using_localization_cmd)
	ld.add_action(declare_rviz_config_file_cmd)
	ld.add_action(rviz_node)
	ld.add_action(rviz_node_namespaced)

	return ld
