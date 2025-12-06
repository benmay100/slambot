# Slambot

A Custom Autonomous Mapping & Navigation Robot (Slambot) Which Can Be Run In Real Life And In Simulation.

* A slam-enabled, Nav2 enabled, four-wheeled diff-drive robot. 
* The robot has four Motors (with encoders), a Lidar, Camera, and an IMU sensor. 
* Odometry is filtered via an ekf node and the robot_localization package. 
* The robot can be teleoperated (ideally using a Logitech F710 controller or similar)
* The robot can be launched into a slam_toolbox or ros_cartographer (your choice!) environment for creating maps
* The robot can then be launched into a Nav2 environment and will navigate to destinations sucessfully via RVIZ. 
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
usb_max_current_enable=1
enable_uart=1
dtparam=uart0=on
dtoverlay=pwm-2chan,pin=18,func=2

# Once done you can reboot
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



# Slambot Launch File Guide

For a comprehensive launch file guide see 'LAUNCH_FILES_GUIDE.md' in slambot_bringup/launch


# Other Functionality & Features...

## 1. Saving Maps (When Using SlamToolbox or Cartographer)

If you've used slam and want to save the map use the in-built map saver node. Open a new terminal, source it correctly and enter:

```python

ros2 launch slambot_slam map_saver.launch.py map_name:=my_maze_map  #(or whatever you want to call it!)

```

After you run the command, you will see a confirmation in the terminal:
* Saving map from '/map' topic to '/home/<your-username>/nav2_maps/my_maze_map' file

Note... if you then want the maps in the /src directory, you need to copy them from the /nav2_maps directory in your home folder into it! 
