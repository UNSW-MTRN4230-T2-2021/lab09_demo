cmake_minimum_required(VERSION 3.0.2)
project(lab09_gazebo)

add_compile_options(-std=c++17)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
      moveit_core
  #    moveit_visual_tools
      moveit_ros_planning
      moveit_ros_planning_interface
  #    pluginlib
  #    geometric_shapes
  #    pcl_ros
  #    pcl_conversions
  #    rosbag
  #    tf2_ros
  #    tf2_eigen
  #    tf2_geometry_msgs
  roslaunch
)

#find_package(Eigen3 REQUIRED)
#find_package(Boost REQUIRED system filesystem date_time thread)

# Perform checks on launch files too
roslaunch_add_file_check(launch)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
      moveit_core
  #    moveit_visual_tools
      moveit_ros_planning_interface
  #    tf2_geometry_msgs
  #  DEPENDS
  #    EIGEN3
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

# ${PROJECT_NAME} expands to lab09_gazebo (or whatever the project name is on line 2 of this file)
add_executable(${PROJECT_NAME}_moveit src/ur5e_epick_move_group_interface.cpp)
target_link_libraries(${PROJECT_NAME}_moveit ${catkin_LIBRARIES} ${Boost_LIBRARIES})

