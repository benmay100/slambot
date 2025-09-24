# Slambot (Four-Wheeled Robot)

A ROS2 robot package with a slam-enabled diff-drive robot which launches into Gazebo and RVIZ and has a lidar and camera, and can be teleoperated. This robot is a four-wheeled robot and is more stable than the two-wheeled version. 

## Usage

```python
# To launch RVIZ only
ros2 launch slambot_description rviz.launch.py

# To launch Gazebo & RVIZ only
ros2 launch slambot_bringup sim_no_slam.launch.py

# To launch Gazebo, RVIZ and SLAM
ros2 launch slambot_bringup sim_with_slam.launch.py

```
## To Teleop

```python
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```


