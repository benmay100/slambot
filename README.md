# Slambot

A ROS2 package which includes...

* A slam-enabled, Nav2 enabled, four-wheeled diff-drive robot. 
* The robot has a lidar, camera, and imu sensor. 
* Odometry is filtered via an ekf node and the robot_localization package. 
* The robot can be teleoperated.
* The robot can be launched into a slam_toolbox environment for creating maps
* The robot can also be launched into a ros_cartographer environment for creating maps (which works better!)
* The robot can then be launched into a Nav2 environment and will navigate to destinations sucessfully via RVIZ. 
* The package is designed so that all functionalities can be used in a namespaced or non-namespaced environment depending on the user's desire.

...Enjoy!

## Namespacing

This package is designed to be run within namespaces (e.g. slambot/topic) or without namespaces. By default the package will run without namespacing, so if you want namespacing ensure to add 'using_namespace:=True' to the end of your launch argument.

e.g. 'ros2 launch slambot_bringup sim_with_slam.launch.py using_namespace:=True'


## Usage

```python
# To launch RVIZ only
ros2 launch slambot_description rviz.launch.py
or
ros2 launch slambot_description rviz.launch.py using_namespace:=True

# To launch Gazebo & RVIZ only
ros2 launch slambot_bringup sim_only.launch.py
or
ros2 launch slambot_bringup sim_only.launch.py using_namespace:=True

# To launch Gazebo, RVIZ and slam_toolbox for mapping
ros2 launch slambot_bringup sim_with_slamtoolbox.launch.py
or
ros2 launch slambot_bringup sim_with_slamtoolbox.launch.py using_namespace:=True

# To launch Gazebo, RVIZ and ros_cartographer for BETTER mapping
ros2 launch slambot_bringup sim_with_slamtoolbox.launch.py
or
ros2 launch slambot_bringup sim_with_slamtoolbox.launch.py using_namespace:=True

```
## To Teleop

```python

#If package is namespaced then topic will be 'slambot/cmd_vel' so use:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/slambot/cmd_vel

#If package is not namespaced then topic will be 'cmd_vel' so use:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Saving Maps (When Using SlamToolbox or Cartographer)

If you've used slam and want to save the map use the in-built map saver node. Open a new terminal, source it correctly and enter:

```python

#If not namespacing...
ros2 launch slambot_slam map_saver.launch.py map_name:=my_maze_map  #(or whatever you want to call it!)

#If namespacing...
ros2 launch slambot_slam map_saver.launch.py using_namespace:=True map_name:=my_maze_map  #(or whatever you want to call it!)
```

After you run the command, you will see a confirmation in the terminal:
* Saving map from '/map' topic to '/home/<your-username>/nav2_maps/my_maze_map' file

Note... if you then want the maps in the /src directory, you need to copy them from the /nav2_maps directory in your home folder into it! 
