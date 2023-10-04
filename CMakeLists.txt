cmake_minimum_required(VERSION 3.0.2)
project(robot_info)


find_package(catkin REQUIRED COMPONENTS
  robotinfo_msgs
  roscpp
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_info
#  CATKIN_DEPENDS robotinfo_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/robot_info_class.cpp
  src/agv_robot_info_node.cpp
)

add_executable(robot_info_node
	src/robot_info_class.cpp
	src/robot_info_main.cpp)
target_link_libraries(robot_info_node ${catkin_LIBRARIES})

add_executable(agv_robot_info_node
	src/robot_info_class.cpp
	src/agv_robot_info_node.cpp)
target_link_libraries(agv_robot_info_node ${catkin_LIBRARIES})