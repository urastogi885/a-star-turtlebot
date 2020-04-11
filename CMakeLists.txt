cmake_minimum_required(VERSION 2.8.3)
project(a-star-turtlebot)

# Compile as C++11, supported in ROS Kinetic
# add_compile_options(-std=c++1)

# Find catkin and any catkin packages
find_package(
  catkin REQUIRED COMPONENTS 
  rospy
  urdf
  std_msgs
  sensor_msgs
  geometry_msgs
)

# Declare a catkin package
catkin_package(
	INCLUDE_DIRS
	CATKIN_DEPENDS rospy geometry_msgs
	DEPENDS
)

# Build target library
include_directories( 
  include
  ${catkin_INCLUDE_DIRS})

install(PROGRAMS
  scripts/planner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Build algo and main
# add_executable(a-star-turtlebot scripts/keyboard.cpp)
# target_link_libraries(a-star-turtlebot ${catkin_LIBRARIES})
