# Slambot

A Custom Autonomous Mapping & Navigation Robot (Slambot) Which Can Be Run In Real Life And In Sim...

* A slam-enabled, Nav2 enabled, four-wheeled diff-drive robot. 
* The robot has four Motors (with encoders), a Lidar, Camera, and an IMU sensor. 
* Odometry is filtered via an ekf node and the robot_localization package. 
* The robot can be teleoperated (ideally using a Logitech F710 controller or similar)
* The robot can be launched into a slam_toolbox or ros_cartographer (your choice!) environment for creating maps
* The robot can then be launched into a Nav2 environment and will navigate to destinations sucessfully via RVIZ. 
* The package is designed so that all functionalities can be used in a namespaced or non-namespaced environment depending on the user's desire.
* The package is also fully capable of operating a real robot, if you want the CAD files and instructions to create the robot visit www.benmay.co.uk/portfolio-slambot-real

...Enjoy!

## For REAL robot operation you'll need to install the following first!

### Suggested file structure...
```python
# Up to you, but this is the suggested structure (NOTE: 'ldlidar_ws' folder only needed on the RaspberryPi, not the dev machine)
mkdir -p ~/ros_workspace/slambot_ws/src # Where main slambot program will live
mkdir -p ~/ros_workspace/ldlidar_ws/src # A necessary third party lidar driver 
mkdir -p ~/ros_workspace/microros_agent/src # Necessary to ensure MicroROS is functioning on both the dev machine AND the embedded Raspberry Pi

```

### Initial installs -> Micro ROS Agent (Dev Machine AND RaspberryPi)
```python
#Clone it...
cd ~/ros_workspace/microros_agent/src
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git
# Rosdep it...
cd ~/ros_workspace/microros_agent
rosdep update && rosdep install --from-paths src --ignore-src -y
# Build it...
colcon build
source install/local_setup.bash
# Create and built the agent...
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
#Test it...
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 baudrate=115200

```

### Initial installs -> LDLidar Driver (Raspberry Pi Only)
```python
#Clone it...
cd ~/ros_workspace/ldlidar_ws/src
git clone https://github.com/ldrobotSensorTeam/ldlidar_ros2.git

#Make sure to do this or /sdk folder won't have any contents in it...
cd ~/ros_workspace/ldlidar_ws/src/ldlidar_ros2
git submodule update --init --recursive

# IMPORTANT!!!
# Now edit the launch file (for my version of slambot I was using LD06 lidar) so...
# Find this file and open it... /ros_workspace/ldlidar_ws/src/ldlidar_ros2/launch/ld06.launch.py
# >>>>> 1. Change 'port_name' to '/dev/ttyAMA0'
# >>>>> 2. Change 'frame_id' to 'lidar_link'
# >>>>> 3. Comment out the entire base_link to base_laser tf node (lines 47 - 53)
# >>>>> 4. Comment out the ld.add_action(base_link_to_laser_tf_node) (line 60)

# Now edit the log_module.cpp file...
# Find this file and open it... /ros_workspace/ldlidar_ws/src/ldlidar_ros2/sdk/src/log_module.cpp
# Add '#include <pthread.h>' just underneath where it says '#include <string.h>'

# Now RosDep and Build it...
cd ~/ros_workspace/ldlidar_ws/
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build
source install/local_setup.bash

# Now add this to your .bashrc
source ~/ros_workspaces/ldlidar_ros2_ws/install/setup.bash

#When setting up on the Raspberry Pi, make sure to provide necessary permissions with...
sudo usermod -aG dialout $USER

```

### Other necessary setup (for the Raspberry Pi)
```python
# We must activate the GPIO and PWM pins which the lidar will use...
sudo nano /boot/firmware/config.txt
# Scroll to the bottom and add these exact lines to the end of the file...

# Force high current USB even if power supply is weak
usb_max_current_enable=1
# Force enable UART for lidar on GPIO 14/15 (pins 8/10)
enable_uart=1
dtparam=uart0=on
# Force enable PWM pin for lidar on Pin 12 (GPIO 18)
dtoverlay=pwm-2chan,pin=18,func=2
# And reboot
sudo reboot

```



## Now you are ready to clone and build THIS repository!
```python
# Navigate to correct directory...
cd ~/ros_workspace/slambot_ws/src
git clone https://github.com/benmay100/slambot.git
cd ~/ros_workspace/slambot_ws/
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build

```


## A Note On Namespacing

This package is designed to be run within namespaces (e.g. slambot/topic) or without namespaces. By default the package will run without namespacing, so if you want namespacing ensure to add 'using_namespace:=True' to the end of your launch argument.

e.g. 'ros2 launch slambot_bringup sim_with_slam.launch.py using_namespace:=True'


# Slambot Launch File Guide

There are a large number of launch files in the slambot program, and each package tends to contain its own specific launch files. Unless you are running specific tests, you should ONLY need to use launch files from within the 'slambot_bringup' package, as these are the top level launch files. 

‼️Be careful about editing launch files within other packages, as these are called upon by the top level launch files in the bringup package!‼️


## Launch File Descriptions (RUNNING IN SIMULATION)

### sim_teleop_only.launch.py: 
Launches Gazebo and RVIZ and Localization (ekf node). Once launched can teleop via teleop_twist_keyboard.
- Optional Launch Arguments...
  - using_namespace | e.g. true / false (default is false)
  - world | e.g. indoor_world_1.sdf
  - headless | e.g. true / false (default is false)
  - jsp_gui | e.g. true / false (default is false)
  - using_localization | e.g. true / false (default is true)
  - using_joy | set to true by default, and assumes you have a Logitech F710 controller or similar plugged into the PC (if not, set to false and use the teleop_twist_keyboard)

```python
#Example
ros2 launch slambot_bringup sim_teleop_only.launch.py using_namespace:=true world:=indoor_world_1.sdf

#Teleop in another terminal (if namespacing)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/slambot/cmd_vel
```



### sim_map_mode.launch.py: 
Launches Gazebo, RVIZ, Localization (ekf node), and SLAM (either slam_toolbox or cartographer_ros depending on your preference and on what slam type best suits the environment you're driving around in).
- Optional Launch Arguments...
  - using_namespace | e.g. true / false (default is false)
  - world | e.g. indoor_world_1.sdf
  - headless | e.g. true / false (default is false)
  - jsp_gui | e.g. true / false (default is false)
  - slam_type | e.g. cartographer / slamtoolbox (default is slamtoolbox)
  - using_joy | set to true by default, and assumes you have a Logitech F710 controller or similar plugged into the PC (if not, set to false and use the teleop_twist_keyboard)


```python
#Example
ros2 launch slambot_bringup sim_map_mode.launch.py slam_type:=cartographer

#Teleop in another terminal (if NOT namespacing)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


### sim_map_mode_and_qrcodes.launch.py: 
Launches Gazebo, RVIZ, Localization (ekf node), and SLAM (either slam_toolbox or cartographer_ros depending on your preference and on what slam type best suits the environment you're driving around in). Where this launch file is different is it also runs another node within the package called 'qr_code_reader'. This is designed to be used in an indoor environment with QRcodes dotted about, so when you are slam mapping, you can also stop at each QRcode and the qr_code_reader node will save the robot's position. These positions are then saved so you can utilise them as waypoints for the Nav2 programs.
- Optional Launch Arguments...
  - using_namespace | e.g. true / false (default is false)
  - world | e.g. indoor_world_with_qr_codes.sdf
  - headless | e.g. true / false (default is false)
  - jsp_gui | e.g. true / false (default is false)
  - slam_type | e.g. cartographer / slamtoolbox (default is slamtoolbox)
  - using_joy | set to true by default, and assumes you have a Logitech F710 controller or similar plugged into the PC (if not, set to false and use the teleop_twist_keyboard)


```python
#Example
ros2 launch slambot_bringup sim_map_mode_and_qrcodes.launch.py using_namespace:=true world:=warehouse_with_qr_codes.sdf

#Teleop in another terminal (if namespacing)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/slambot/cmd_vel
```

### sim_nav_mode.launch.py: 
Launch file to start the Gazebo and RVIZ with Nav2. This only currently works using a NON namespaced environment. Work is under way to try and get it working with namespaced environment too...
- Optional Launch Arguments...
  - using_namespace | false (MUST remain as false)
  - world | e.g. indoor_world_with_qr_codes.sdf
  - headless | e.g. true / false (default is false)
  - jsp_gui | e.g. true / false (default is false)
  - map | e.g. "/home/<user>/nav2_maps/indoor_map_cartographed.yaml"
  - using_joy | set to true by default, and assumes you have a Logitech F710 controller or similar plugged into the PC (if not, set to false and use the teleop_twist_keyboard)


```python
#Example
ros2 launch slambot_bringup sim_nav_mode.launch.py map:="/home/<user>/nav2_maps/indoor_map_cartographed.yaml"
```


## Launch File Descriptions (RUNNING REAL ROBOT)
When running the real robot, it is likely you'll have a dev machine on hand for visualisation purposes, so these launch files include the real_... and dev_... files

### dev_rviz_teleop_only.launch.py: 
Launches Rviz for visualisation purposes. Assumes no slam or nav2 running so the config should be "out of the box"

### dev_rviz_map_mode.launch.py
Launches Rviz for visualisation purposes. Assumes slam is running so will provide the necessary config to view this. 

### More to be added.... TBC

##

# Other Functionality & Features...

## 1. Saving Maps (When Using SlamToolbox or Cartographer)

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
