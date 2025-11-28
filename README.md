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

___

# Getting Started... (Dev Machine)
Up to you, but this is the suggested structure (NOTE: The 'ldlidar_ws' directory is only needed on the RaspberryPi, not the dev machine)

```python
mkdir -p ~/slambot_ws/src
cd /slambot_ws/src
git clone https://github.com/benmay100/slambot.git
```

You should now have the following directory structure:

```python
/home/user/slambot_ws/     
└── src/
    └── slambot/
        ├── hardware/
        │   ├── cad/
        │   └── electronics/
        ├── firmware/
        │   └── pico_code/           # C motor driver and IMU driver code for the Pico
        ├── ros_packages/            
        │   ├── slambot_bringup/
        │   ├── slambot_description/
        │   ├── slambot_gazebo/
        │   ├── slambot_hardware/
        │   ├── slambot_localization/
        │   ├── slambot_nav2/
        │   ├── slambot_slam/
        │   └── slambot_scripts/ # *Some QR code following scripts (optional)
        ├── scripts/
        │   └── raspberry_pi_setup/            # boot scripts and setup shell scripts for the Raspberry Pi 5
        ├── docs/
        │   ├── bill_of_materials.md # Materials List
        │   ├── assembly_guide.md # Instructions on how to build slambot
        │   └── Other PDF documents / guides    # Various helper guides and documents
        └── README.md                
```

## Installation (Dev Machine)

Everything you see above, is everything you need for the dev machine. You just need to install dependencies...

```bash
cd ~/slambot_ws
rosdep install --from-paths src --ignore-src -r -y
```
...then build it.

```bash
colcon build
```
...and finally source it.

```bash
source install/setup.bash
# Note you may want to add the sourcing of the workspace to your ~/.bashrc file for ease of use
```

Your file structure should now look like this...

```python
/home/user/slambot_ws/     
├── build/
├── install/
├── log/
└── src/
    └── slambot/
        ├── hardware/
        │   ├── cad/
        │   └── electronics/
        ├── firmware/
        │   └── pico_code/           # C motor driver and IMU driver code for the Pico
        ├── ros_packages/            
        │   ├── slambot_bringup/
        │   ├── slambot_description/
        │   ├── slambot_gazebo/
        │   ├── slambot_hardware/
        │   ├── slambot_localization/
        │   ├── slambot_nav2/
        │   ├── slambot_slam/
        │   └── slambot_scripts/ # *Some QR code following scripts (optional)
        ├── scripts/
        │   └── raspberry_pi_setup/            # boot scripts and setup shell scripts for the Raspberry Pi 5
        ├── docs/
        │   ├── bill_of_materials.md # Materials List
        │   ├── assembly_guide.md # Instructions on how to build slambot
        │   └── Other PDF documents / guides    # Various helper guides and documents
        └── README.md                
```

## Run a Test (Simulation)

```python
cd ~/slambot_ws
source install/setup.bash
ros2 launch slambot_bringup sim_teleop_only.launch.py

# Note you'll need a logitech F710 (or similar) controller, or you can use the keyboard with...

ros2 run teleop_twist_keyboard teleop_twist_keyboard

```

# CRITICAL Setup Steps (Raspberry Pi 5)
It is critical the Raspberry Pi 5 is setup correctly. Please see below the steps required:

### Step 1: Flash Ubuntu Terminal 24.04.3 (LTS) to the SD card
### Step 2: Install Ubuntu Terminal, make sure to allow SSH and setup to connect your home wifi
Up to you, but suggest using these credentials as these will match the config files in this package
- user: slambot-rpi@slambot-rpi.local

### Step 3: Once Pi boots up, SSH into it and install ROS Jazzy (Bare Bones)
### Step 4: Create necessary directories and clone package...

```python
mkdir -p ~/slambot_pi_ws/src
cd /slambot_pi_ws/src
git clone https://github.com/benmay100/slambot.git
cd /slambot_pi_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build

# Optional: If you want to remove unwanted folders from the Pi...

rm -rf /docs /firmware /hardware
```

### Step 5: Create an additional directory in the Pi, and set up the Lidar Module

```python
cd 
mkdir -p ldlidar_ws/src
cd ~/ldlidar_ws/src
git clone https://github.com/ldrobotSensorTeam/ldlidar_ros2.git

#Make sure to do this or /sdk folder won't have any contents in it...
cd ~/ldlidar_ws/src/ldlidar_ros2
git submodule update --init --recursive

# IMPORTANT!!!
# Now edit the launch file (for the LD06 lidar) so...
# Find this file and open it... /ldlidar_ws/src/ldlidar_ros2/launch/ld06.launch.py
# >>>>> 1. Change 'port_name' to '/dev/ttyAMA0'
# >>>>> 2. Change 'frame_id' to 'lidar_link'
# >>>>> 3. Comment out the entire base_link to base_laser tf node (lines 47 - 53)
# >>>>> 4. Comment out the ld.add_action(base_link_to_laser_tf_node) (line 60)

# Now edit the log_module.cpp file...
# Find this file and open it... /ldlidar_ws/src/ldlidar_ros2/sdk/src/log_module.cpp
# Add '#include <pthread.h>' just underneath where it says '#include <string.h>'

# Now RosDep and Build it...
cd ~/ldlidar_ws/
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build
source install/local_setup.bash

# Now add this to your .bashrc (optional)
source ~/ros_workspaces/ldlidar_ros2_ws/install/setup.bash

#When setting up on the Raspberry Pi, make sure to provide necessary permissions with...
sudo usermod -aG dialout $USER
```
Now.... CRITICALLY important for the lidar setup, we need to stop the Raspberry Pi screaming rubbish down the serial during boot as this can crash the lidar driver... so do this:

```python
sudo nano /boot/firmware/cmdline.txt

# And now delete the line that is there and replace it with this:
multipath=off dwc_otg.lpm_enable=0 console=tty1 root=LABEL=writable rootfstype=ext4 rootwait fixrtc cfg80211.ieee80211_regdom=GB

# Now save and reboot!
```


### Step 6: Create an additional directory in the Pi, and set up the MicroROS Agent

```python
cd
mkdir -p ~/microros_agent/src
cd /microros_agent/src

#Clone it...
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git

# Rosdep it...
cd ~/microros_agent
rosdep update && rosdep install --from-paths src --ignore-src -y

# Build it...
colcon build
source install/local_setup.bash

# Create and built the agent...
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash

#Test it... (optional)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 baudrate=115200

```
### Step 7: Set up Raspberry Pi 5 (running Ubuntu) to accept the PiCam3
This is a more complex operation when running Ubuntu, so see the full guide in [GetPiCamWorkingOnRaspberryPi_WithUbuntu.pdf](docs/GetPiCamWorkingOnRaspberryPi_WithUbuntu.pdf)

### Step 8: Edit the config.txt file on the Raspberry Pi
See the file with all necessary changes [HERE](scripts/config.txt)

### Step 9: Create a boot script to allow you to boot into different modes

The good news is, the script is already here [robot_mode_boot.py](scripts/robot_mode_boot.py)
So just do this:
```python
cp ~/slambot_pi_ws/src/scripts/robot_mode_boot.py ~/
```
Now, critically important to do this...
```python
chmod +x ~/robot_mode_boot.py
sudo nano /etc/systemd/system/robot_boot.service
```
And in the robot_boot.service file, add this and save:
```python
[Unit]
Description=Robot Boot Mode Selector
Wants=network-online.target
After=network.target network-online.target time-sync.target

[Service]
Type=simple
# CHANGED: Updated to your specific user
User=slambot-rpi
Environment="DISPLAY=:0"
# CHANGED: Updated to your specific home directory
Environment="HOME=/home/slambot-rpi"
# ADDED: 
WorkingDirectory=/home/slambot-rpi
# CHANGED: Ensure this path matches where you saved the python script
ExecStart=/usr/bin/python3 /home/slambot-rpi/robot_mode_boot.py

Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```
Now you'll need to ensure this library is installed...
```python
sudo apt install python3-lgpio
```

And finally, start the reload, enable and start the deamon...
```python
sudo systemctl daemon-reload
sudo systemctl enable robot_boot.service
sudo systemctl start robot_boot.service
```

# CRITICAL Setup Steps (Raspberry Pi Pico)
Just like the Raspberry Pi 5, the Pico has its own setup process which can be complex if not understood. Below are the necessary steps:
### Step 1:  TBC...
### Step 2:  TBC...
### Step 3:  TBC...

---

# Now you're ready to launch some files!
See Launch File Guide [HERE](ros_packages/slambot_bringup/launch/LAUNCH_FILES_GUIDE.md)