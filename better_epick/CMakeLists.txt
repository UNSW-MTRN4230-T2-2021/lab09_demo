cmake_minimum_required(VERSION 3.5.1)

project(better_epick)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  message_generation
  gazebo_msgs
  roscpp
  rospy
  std_srvs
  geometry_msgs
  urdf
  tf
  tf2_ros
  std_msgs
  visualization_msgs
)

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)


add_library(better_epick SHARED src/better_epick.cpp)
target_link_libraries(better_epick ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
