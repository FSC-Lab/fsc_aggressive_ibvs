cmake_minimum_required(VERSION 3.0.2)
project(autopilot_monitor)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  vision_common
  vision_msgs
  mavros
  mavros_extras
  mavros_msgs
)

generate_dynamic_reconfigure_options(
  cfg/BasicTune.cfg
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES autopilot_monitor
 CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  dynamic_reconfigure
  vision_common
  vision_msgs
  mavros
  mavros_extras
  mavros_msgs
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

# add_library(autopilot_monitor src/autopilot_monitor.cpp)
# target_link_libraries(autopilot_monitor ${catkin_LIBRARIES})
# add_dependencies(autopilot_monitor ${PROJECT_NAME}_gencfg)

add_executable(autopilot_monitor_node src/autopilot_monitor_node.cpp
                                    src/autopilot_monitor.cpp)
add_dependencies(autopilot_monitor_node ${PROJECT_NAME}_gencfg)

target_link_libraries(
  autopilot_monitor_node
  ${catkin_LIBRARIES}
)
