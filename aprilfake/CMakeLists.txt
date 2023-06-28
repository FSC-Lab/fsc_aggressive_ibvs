cmake_minimum_required(VERSION 3.0.2)
project(aprilfake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  image_transport
  image_geometry
  nodelet
  vision_common
  vision_msgs
  trajectory_generation_helper
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES aprilfake
 CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  cv_bridge
  image_transport
  image_geometry
  vision_common
  vision_msgs
  trajectory_generation_helper
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

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_library(aprilfake_nodelet
  src/aprilfake_nodelet.cpp)
add_dependencies(aprilfake_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(aprilfake_nodelet
  PUBLIC
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_executable(aprilfake_node src/aprilfake_node_main.cpp)
target_link_libraries(aprilfake_node
  PUBLIC
  aprilfake_nodelet
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
