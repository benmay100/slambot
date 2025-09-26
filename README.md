# Slambot (Four-Wheeled Robot)

A ROS2 robot package with a slam-enabled diff-drive robot which launches into Gazebo and RVIZ and has a lidar and camera, and can be teleoperated. This robot is a four-wheeled robot and is more stable than the two-wheeled version. 

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
ros2 launch slambot_bringup sim_no_slam.launch.py
or
ros2 launch slambot_bringup sim_no_slam.launch.py using_namespace:=True

# To launch Gazebo, RVIZ and SLAM
ros2 launch slambot_bringup sim_with_slam.launch.py
or
ros2 launch slambot_bringup sim_with_slam.launch.py using_namespace:=True

```
## To Teleop

```python

#If package is namespaced then topic will be 'slambot/cmd_vel' so use:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/slambot/cmd_vel

#If package is not namespaced then topic will be 'cmd_vel' so use:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Saving Maps (When Using Slam)

If you've used slam and want to save the map use the in-built map saver node. Open a new terminal, source it correctly and enter:

```python

ros2 launch slambot_slam map_saver.launch.py map_name:=my_maze_map  (or whatever you want to call it!)

```

After you run the command, you will see a confirmation in the terminal:
* slambot_ws2/install/slambot_slam/share/slambot_slam/maps/my_maze_map.pgm
* slambot_ws2/install/slambot_slam/share/slambot_slam/maps/my_maze_map.yaml

Note... if you then want the maps in the /src directory, you need to copy them from the /share directory into it! Note that both the slambot_slam package, and the slambot_nav2 package have /maps directories. 
