#!/usr/bin/env python3
"""Launch RViz on a dev machine while the real robot runs teleop."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

	# Package paths
	pkg_slambot_bringup = get_package_share_directory('slambot_bringup')

	# File paths
	rviz_config_path = os.path.join(pkg_slambot_bringup, 'rviz', 'dev_rviz_teleop_config.rviz')


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
		arguments=['-d', rviz_config_path],
		parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
	)


	ld = LaunchDescription()
	ld.add_action(declare_use_sim_time_cmd)
	ld.add_action(rviz_node)

	return ld
