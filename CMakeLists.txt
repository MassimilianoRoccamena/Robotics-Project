cmake_minimum_required(VERSION 3.0.2)
project(project2)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES project2
   CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(fix_scout_odom src/fix_scout_odom.cpp)
target_link_libraries(fix_scout_odom ${catkin_LIBRARIES})

add_executable(fix_camera_odom src/fix_camera_odom.cpp)
target_link_libraries(fix_camera_odom ${catkin_LIBRARIES})

#############
## Install ##
#############

#############
## Testing ##
#############
