#!/usr/bin/env python3
"""Launch RViz on a dev machine while the real robot runs nav mode."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

	# Package paths
	pkg_slambot_bringup = get_package_share_directory('slambot_bringup')
	pkg_slambot_nav2 = get_package_share_directory('slambot_nav2')

	# File paths
	rviz_config_path_nav2 = os.path.join(pkg_slambot_nav2, 'rviz', 'nav2_rviz_config.rviz')

	declare_use_sim_time_cmd = DeclareLaunchArgument(
		'use_sim_time',
		default_value='false',
		description='Set to true only when replaying bags with simulated time.'
	)

	rviz_node = Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		output='screen',
		arguments=['-d', rviz_config_path_nav2],
		parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
	)


	ld = LaunchDescription()
	ld.add_action(declare_use_sim_time_cmd)
	ld.add_action(rviz_node)

	return ld
