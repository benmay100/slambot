#!/usr/bin/env python3
"""Launch RViz on a dev machine while the real robot runs map mode."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
	pkg_slambot_slam = get_package_share_directory('slambot_slam')

	rviz_config_path_slamtoolbox = os.path.join(
		pkg_slambot_slam,
		'rviz',
		'gazebo_rviz_slamtoolbox_config.rviz'
	)
	rviz_config_path_slamtoolbox_namespaced = os.path.join(
		pkg_slambot_slam,
		'rviz',
		'gazebo_rviz_slamtoolbox_config_namespaced.rviz'
	)
	rviz_config_path_cartographer = os.path.join(
		pkg_slambot_slam,
		'rviz',
		'gazebo_rviz_cartographer_config.rviz'
	)
	rviz_config_path_cartographer_namespaced = os.path.join(
		pkg_slambot_slam,
		'rviz',
		'gazebo_rviz_cartographer_config_namespaced.rviz'
	)

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

	declare_slam_type_cmd = DeclareLaunchArgument(
		'slam_type',
		default_value='slamtoolbox',
		description='Select the SLAM stack running on the robot.'
	)

	declare_rviz_config_file_cmd = DeclareLaunchArgument(
		'rviz_config_file',
		default_value=PythonExpression([
			"'",
			rviz_config_path_cartographer_namespaced,
			"' if '",
			LaunchConfiguration('slam_type'),
			"'.lower() == 'cartographer' and '",
			LaunchConfiguration('using_namespace'),
			"'.lower() == 'true' else '",
			rviz_config_path_cartographer,
			"' if '",
			LaunchConfiguration('slam_type'),
			"'.lower() == 'cartographer' else '",
			rviz_config_path_slamtoolbox_namespaced,
			"' if '",
			LaunchConfiguration('slam_type'),
			"'.lower() == 'slamtoolbox' and '",
			LaunchConfiguration('using_namespace'),
			"'.lower() == 'true' else '",
			rviz_config_path_slamtoolbox,
			"'"
		]),
		description='RViz configuration file matching the robot SLAM mode.'
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

	rqt_image_view_node = Node(
		package='rqt_image_view',
		executable='rqt_image_view',
		name='rqt_image_view',
		output='screen'
	)

	ld = LaunchDescription()
	ld.add_action(declare_robot_name_cmd)
	ld.add_action(declare_use_sim_time_cmd)
	ld.add_action(declare_using_namespace_cmd)
	ld.add_action(declare_slam_type_cmd)
	ld.add_action(declare_rviz_config_file_cmd)
	ld.add_action(rviz_node)
	ld.add_action(rviz_node_namespaced)
	ld.add_action(rqt_image_view_node)

	return ld
