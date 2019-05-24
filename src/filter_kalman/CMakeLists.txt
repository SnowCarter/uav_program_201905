cmake_minimum_required(VERSION 2.8.3)
#project(ReadBagToKalman)
project(filter_kalman)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  sensor_msgs
  pcl_ros
  grid_map_ros
  tf
  tf_conversions
  #kindr_ros
  #odometry_tf

)
find_package(kindr)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES filter_kalman
  CATKIN_DEPENDS roscpp rospy std_msgs #odometry_tf
  DEPENDS  system_lib

)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${kindr_INCLUDE_DIRS}
)




## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
#include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(filter_kalman src/filter_kalman_node.cpp
    src/filter_kalman.cpp
    include/filter_kalman.h)
target_link_libraries(filter_kalman  ${catkin_LIBRARIES})
add_dependencies(filter_kalman filter_kalman_generate_messages_cpp)
