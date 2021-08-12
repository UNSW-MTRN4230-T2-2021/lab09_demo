cmake_minimum_required(VERSION 3.0.2)
project(lab09_gazebo)

add_compile_options(-std=c++17)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
      moveit_core
      moveit_ros_planning
      moveit_ros_planning_interface
      roslaunch
)

# Perform checks on launch files too
roslaunch_add_file_check(launch)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
      moveit_core
      moveit_ros_planning_interface
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

# ${PROJECT_NAME} expands to lab09_gazebo (or whatever the project name is on line 2 of this file)
add_executable(${PROJECT_NAME}_moveit src/ur5e_epick_move_group_interface.cpp)
target_link_libraries(${PROJECT_NAME}_moveit ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(cartesian_movement src/cartesian_movement.cpp)
target_link_libraries(cartesian_movement ${catkin_LIBRARIES} ${Boost_LIBRARIES})

