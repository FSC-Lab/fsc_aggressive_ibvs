cmake_minimum_required(VERSION 3.0.2)
project(flight_system)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## CMake Setup
# Build in Release mode if nothing is specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  # mav_msgs
  vision_common
  # mavros
  mavros_extras
  mavros_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES flight_system
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    # mav_msgs
    vision_common
    mavros
    mavros_extras
    mavros_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

list(APPEND SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/flight_system.cpp
    ${PROJECT_SOURCE_DIR}/src/rotors_system.cpp
    ${PROJECT_SOURCE_DIR}/src/pixhawk_system.cpp
)

add_library(flight_system ${SOURCE_FILES})
add_dependencies(flight_system ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(flight_system
  ${catkin_LIBRARIES}
)