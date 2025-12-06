cmake_minimum_required(VERSION 3.8)
project(slambot_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(cmd_vel_watchdog
  src/cmd_vel_watchdog.cpp
)

ament_target_dependencies(cmd_vel_watchdog
  action_msgs
  geometry_msgs
  rclcpp
)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS cmd_vel_watchdog
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
