cmake_minimum_required(VERSION 3.0.2)
project(param_tuner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  vision_common
  vision_msgs
)

generate_dynamic_reconfigure_options(
  cfg/ParamTuner.cfg
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES param_tuner
 CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  dynamic_reconfigure
  vision_common
  vision_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## CMake Setup
# Build in Release mode if nothing is specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(param_tuner_node src/param_tuner_node.cpp
                                    src/param_tuner.cpp)
add_dependencies(param_tuner_node ${PROJECT_NAME}_gencfg)

target_link_libraries(
  param_tuner_node
  PUBLIC
  ${catkin_LIBRARIES}
)
