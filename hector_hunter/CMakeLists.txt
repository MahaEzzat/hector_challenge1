cmake_minimum_required(VERSION 2.8.3)
project(hector_hunter)
find_package(catkin REQUIRED COMPONENTS actionlib geometry_msgs hector_quadrotor_interface roscpp tf2 tf2_geometry_msgs hector_uav_msgs std_msgs)
  
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES 
    CATKIN_DEPENDS roscpp
    DEPENDS
)

add_executable(${PROJECT_NAME} 
  src/hector_hunter_node.cpp
  src/HectorHunter.cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)