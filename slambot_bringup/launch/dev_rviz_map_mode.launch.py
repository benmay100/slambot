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

	# Package paths
	pkg_slambot_bringup = get_package_share_directory('slambot_bringup')
	pkg_slambot_slam = get_package_share_directory('slambot_slam')

	# File paths
	rviz_config_path = os.path.join(pkg_slambot_bringup, 'rviz', 'dev_rviz_teleop_config.rviz')


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

	declare_slam_type_cmd = DeclareLaunchArgument(
		'slam_type',
		default_value='slamtoolbox',
		description='Select the SLAM stack running on the robot.'
	)


	rviz_node = Node(
		condition=UnlessCondition(LaunchConfiguration('using_namespace')),
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		output='screen',
		arguments=['-d', rviz_config_path],
		parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
	)



	ld = LaunchDescription()
	ld.add_action(declare_robot_name_cmd)
	ld.add_action(declare_use_sim_time_cmd)
	ld.add_action(declare_slam_type_cmd)
	ld.add_action(rviz_node)

	return ld
