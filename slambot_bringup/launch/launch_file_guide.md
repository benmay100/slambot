# Slambot Launch File Guide

There are a large number of launch files in the slambot program. Unless running specific tests, you should only need to use launch files from within the 'slambot_bringup' package, as these are the top level launch files. 

‼️Be careful about editing launch files within other packages, as these are called upon by the top level launch files in the bringup package!‼️

## Launch File Descriptions

### sim_only.launch.py: 
Launches Gazebo and RVIZ and Localization (ekf node). Once launched can teleop via teleop_twist_keyboard.
- Optional Launch Arguments...
  - using_namespace | e.g. true / false (default is false)
  - world | e.g. indoor_world_1.sdf
  - headless | e.g. true / false (default is false)
  - jsp_gui | e.g. true / false (default is false)
  - using_localization | e.g. true / false (default is true)

```python
#Example
ros2 launch slambot_bringup sim_only.launch.py using_namespace:=true world:=indoor_world_1.sdf using_localization:=false

#Teleop in another terminal (if namespacing)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/slambot/cmd_vel
```



### sim_with_slam.launch.py: 
Launches Gazebo, RVIZ, Localization (ekf node), and SLAM (either slam_toolbox or cartographer_ros depending on your preference and on what slam type best suits the environment you're driving around in.
- Optional Launch Arguments...
  - using_namespace | e.g. true / false (default is false)
  - world | e.g. indoor_world_1.sdf
  - headless | e.g. true / false (default is false)
  - jsp_gui | e.g. true / false (default is false)
  - slam_type | e.g. cartographer / slamtoolbox (default is cartographer)


```python
#Example
ros2 launch slambot_bringup sim_with_slam.launch.py slam_type:=slamtoolbox

#Teleop in another terminal (if NOT namespacing)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


### sim_with_slam_and_qr_codes.launch.py: 
Launches Gazebo, RVIZ, Localization (ekf node), and SLAM (either slam_toolbox or cartographer_ros depending on your preference and on what slam type best suits the environment you're driving around in. Where this launch file is different is it also runs another node within the package called 'qr_code_reader'. This is designed to be used in an indoor environment with QRcodes dotted about, so when you are slam mapping, you can also stop at each QRcode and the qr_code_reader node will save the robot's position. These positions are then saved so you can utilise them as waypoints for the Nav2 programs.
- Optional Launch Arguments...
  - using_namespace | e.g. true / false (default is false)
  - world | e.g. indoor_world_with_qr_codes.sdf
  - headless | e.g. true / false (default is false)
  - jsp_gui | e.g. true / false (default is false)
  - slam_type | e.g. cartographer / slamtoolbox (default is cartographer)


```python
#Example
ros2 launch slambot_bringup sim_with_slam_and_qr_codes.launch.py using_namespace:=true world:=warehouse_with_qr_codes.sdf

#Teleop in another terminal (if namespacing)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/slambot/cmd_vel
```

### sim_with_nav2.launch.py: 
Launch file to start the Gazebo and RVIZ with Nav2. This only currently works using a NON namespaced environment. Work is under way to try and get it working with namespaced environment too...
- Optional Launch Arguments...
  - using_namespace | false (MUST remain as false)
  - world | e.g. indoor_world_with_qr_codes.sdf
  - headless | e.g. true / false (default is false)
  - jsp_gui | e.g. true / false (default is false)
  - map | e.g. "/home/<user>/nav2_maps/indoor_map_cartographed.yaml"


```python
#Example
ros2 launch slambot_bringup sim_with_nav2.launch.py map:="/home/<user>/nav2_maps/indoor_map_cartographed.yaml"
```