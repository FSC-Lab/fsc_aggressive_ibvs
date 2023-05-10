cmake_minimum_required(VERSION 3.0.2)
project(fsc_autopilot)

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
  flight_system
  state_predictor
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
    flight_system
    state_predictor
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(mpc_solver
  externals/qpoases/SRC/Bounds.cpp
  externals/qpoases/SRC/Constraints.cpp
  externals/qpoases/SRC/CyclingManager.cpp
  externals/qpoases/SRC/Indexlist.cpp
  externals/qpoases/SRC/MessageHandling.cpp
  externals/qpoases/SRC/QProblem.cpp
  externals/qpoases/SRC/QProblemB.cpp
  externals/qpoases/SRC/SubjectTo.cpp
  externals/qpoases/SRC/Utils.cpp
  externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  model/pose_track_mpc/acado_qpoases_interface.cpp
  model/pose_track_mpc/acado_integrator.c
  model/pose_track_mpc/acado_solver.c
  model/pose_track_mpc/acado_auxiliary_functions.c
)
target_include_directories(mpc_solver PUBLIC
  model/pose_track_mpc/
  externals/qpoases
  externals/qpoases/INCLUDE
  externals/qpoases/SRC
  ${catkin_INCLUDE_DIRS}
)

add_library(mpc_wrapper
  src/mpc/mpc_wrapper.cpp)
target_link_libraries(mpc_wrapper
  mpc_solver
  ${catkin_LIBRARIES})
  
add_library(mpc_controller
  src/mpc/mpc_controller.cpp)
target_link_libraries(mpc_controller 
  mpc_wrapper
  ${catkin_LIBRARIES})

add_executable(fsc_pixhawk_node
  src/node/fsc_pixhawk_node.cpp
  src/distparam_trajectory.cpp)
target_link_libraries(fsc_pixhawk_node
  PUBLIC
  mpc_solver
  mpc_wrapper
  mpc_controller
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(fsc_rotors_node
  src/node/fsc_rotors_node.cpp
  src/distparam_trajectory.cpp)
target_link_libraries(fsc_rotors_node
  PUBLIC
  mpc_solver
  mpc_wrapper
  mpc_controller
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(goal_broadcast_node src/node/goal_broadcast_node.cpp)
target_link_libraries(goal_broadcast_node
  ${catkin_LIBRARIES}
)

add_executable(flight_mode_publisher_node src/node/flight_mode_publisher_node.cpp)
target_link_libraries(flight_mode_publisher_node
  ${catkin_LIBRARIES}
)

