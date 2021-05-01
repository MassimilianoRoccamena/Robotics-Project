cmake_minimum_required(VERSION 3.0.2)
project(project1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
  message_filters
  tf
  dynamic_reconfigure
  robotics_hw1
)

# Messages, services and actions

add_message_files( FILES
  CustomOdometry.msg
  Correction.msg
)

add_service_files( FILES
  ResetPose.srv
  SetPose.srv
)

generate_messages( DEPENDENCIES
  std_msgs
  nav_msgs
)

# Dynamic reconfigure parameters

generate_dynamic_reconfigure_options(
  cfg/robot.cfg
)

# Catkin specific configuration

catkin_package( CATKIN_DEPENDS
  roscpp rospy
  std_msgs
  geometry_msgs
  nav_msgs
  message_runtime
  message_filters
  robotics_hw1
)

# Build

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot src/robot.cpp)
target_link_libraries(robot ${catkin_LIBRARIES})

add_executable(calibration src/calibration.cpp)
target_link_libraries(calibration ${catkin_LIBRARIES})