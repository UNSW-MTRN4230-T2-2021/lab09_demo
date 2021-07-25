cmake_minimum_required(VERSION 2.8.3)

project(lab08_package)


find_package(catkin REQUIRED
  COMPONENTS
    std_srvs
    gazebo_msgs
    geometry_msgs
    tf2
    roscpp
)

catkin_package(
  # INCLUDE_DIRS
  #   include
  # LIBRARIES
  CATKIN_DEPENDS  
    std_srvs
    gazebo_msgs
    geometry_msgs
    tf2
    roscpp  
  # DEPENDS
  
    
)

include_directories(
  lab08_cpp  
  ${catkin_INCLUDE_DIRS}
)

add_executable(lab08_cpp nodes/lab08_cpp.cpp)
target_link_libraries(lab08_cpp ${catkin_LIBRARIES})



install(DIRECTORY launch controller DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

