# Slambot Launch File Guide

There are a large number of launch files in the slambot program, and each package tends to contain its own specific launch files. Unless you are running specific tests, you should ONLY need to use launch files from within the `slambot_bringup` package, as these are the top level launch files.

‼️Be careful about editing launch files within other packages, as these are called upon by the top level launch files in the bringup package!‼️


## Launch File Descriptions (RUNNING IN SIMULATION)

### sim_teleop_only.launch.py
Launches Gazebo, RVIZ, and the EKF localization stack for pure teleoperation. Twist_mux, joystick (optional), and teleop nodes are started so you can drive the robot manually.
- Optional Launch Arguments...
  - world | e.g. `indoor_world_1.sdf`
  - robot_name | Gazebo entity name (default `slambot`)
  - headless | true / false (default false)
  - use_sim_time | true / false (default true)
  - jsp_gui | true / false (default false)
  - using_joy | true by default; set false if no joystick is connected

```python
#Example
ros2 launch slambot_bringup sim_teleop_only.launch.py world:=indoor_world_1.sdf

#Teleop in another terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


### sim_map_mode.launch.py
Launches Gazebo, RVIZ, the EKF localization stack, and SLAM (slam_toolbox or cartographer). Use this when generating a map in simulation.
- Optional Launch Arguments...
  - world | e.g. `indoor_world_1.sdf`
  - robot_name | Gazebo entity name (default `slambot`)
  - headless | true / false (default false)
  - use_sim_time | true / false (default true)
  - jsp_gui | true / false (default false)
  - slam_type | `cartographer` or `slamtoolbox` (default `slamtoolbox`)
  - slam_params_file | alternate slam_toolbox parameter file
  - configuration_directory | cartographer config directory override
  - cartographer_config_file | cartographer Lua file override
  - rviz_config_file | RVIZ config override (auto-selected by default)
  - using_joy | true by default; set false if no joystick is connected

```python
#Example
ros2 launch slambot_bringup sim_map_mode.launch.py slam_type:=cartographer

#Teleop in another terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


### sim_map_mode_and_qrcodes.launch.py
Extends `sim_map_mode` by launching the `slambot_scripts/qr_code_reader` node. Use this in QR-coded environments to capture waypoint positions while mapping.
- Optional Launch Arguments...
  - world | e.g. `indoor_world_with_qr_codes.sdf`
  - robot_name | Gazebo entity name (default `slambot`)
  - headless | true / false (default false)
  - use_sim_time | true / false (default true)
  - jsp_gui | true / false (default false)
  - slam_type | `cartographer` or `slamtoolbox` (default `slamtoolbox`)
  - slam_params_file | alternate slam_toolbox parameter file
  - configuration_directory | cartographer config directory override
  - cartographer_config_file | cartographer Lua file override
  - rviz_config_file | RVIZ config override (auto-selected by default)
  - using_joy | true by default; set false if no joystick is connected

```python
#Example
ros2 launch slambot_bringup sim_map_mode_and_qrcodes.launch.py world:=warehouse_with_qr_codes.sdf

#Teleop in another terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


### sim_nav_mode.launch.py
Launches Gazebo, RVIZ, EKF localization, and the Nav2 stack with a supplied map. Ideal for testing autonomous navigation in simulation.
- Optional Launch Arguments...
  - world | e.g. `indoor_world_with_qr_codes.sdf`
  - robot_name | Gazebo entity name (default `slambot`)
  - headless | true / false (default false)
  - use_sim_time | true / false (default true)
  - jsp_gui | true / false (default false)
  - map | e.g. `/home/<user>/nav2_maps/indoor_map_cartographer.yaml`
  - using_joy | true by default; set false if no joystick is connected

```python
ros2 launch slambot_bringup sim_nav_mode.launch.py map:=/home/<user>/nav2_maps/indoor_map_cartographer.yaml
```


## Launch File Descriptions (RUNNING REAL ROBOT)
When running the real robot, you will typically pair one of the real_* launch files with an RVIZ view on a development machine using the dev_* launch files.

### dev_rviz_teleop_only.launch.py
Launches RVIZ on a development machine for visualising the robot while the teleop stack is running (no SLAM or Nav2).
- Optional Launch Arguments...
  - use_sim_time | true only when replaying recorded bags (default false)

### dev_rviz_map_mode.launch.py
Launches RVIZ on a development machine for visualising the robot while SLAM is running. Chooses the RVIZ config based on `slam_type`.
- Optional Launch Arguments...
  - use_sim_time | true only when replaying recorded bags (default false)
  - slam_type | `cartographer` or `slamtoolbox` (default `slamtoolbox`)

### real_teleop_only.launch.py
Brings up the real robot for teleoperation with sensor fusion. Starts micro-ROS, controllers, EKF, lidar, camera, and optional joystick nodes.
- Optional Launch Arguments...
  - use_sim_time | must remain `false` for hardware (default false)
  - using_joy | true by default; set false if no joystick is connected

### real_map_mode.launch.py
End-to-end bringup for the real robot performing SLAM (slam_toolbox or cartographer) alongside localization and sensor fusion. Includes micro-ROS, sensors, controllers, SLAM, and optional joystick nodes.
- Optional Launch Arguments...
  - use_sim_time | must remain `false` for hardware (default false)
  - slam_type | `cartographer` or `slamtoolbox` (default `slamtoolbox`)
  - slam_params_file | alternate slam_toolbox parameter file
  - configuration_directory | cartographer config directory override
  - cartographer_config_file | cartographer Lua file override
  - using_joy | true by default; set false if no joystick is connected

