cmake_minimum_required(VERSION 3.0.2)
project(circle_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build in Release mode if nothing is specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  vision_common
  vision_msgs
  trajectory_generation_helper
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fsc_autopilot
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    vision_common
    vision_msgs
    trajectory_generation_helper
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(circle_planner
  src/circle_planner.cpp)
target_link_libraries(circle_planner
  ${catkin_LIBRARIES})

  add_executable(circle_planner_node src/circle_planner_node.cpp)
  target_link_libraries(circle_planner_node
    circle_planner
    ${catkin_LIBRARIES}
  )