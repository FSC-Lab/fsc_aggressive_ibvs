cmake_minimum_required(VERSION 3.0.2)
project(vision_common)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-O3)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

cs_add_library(${PROJECT_NAME} 
  src/math_common.cpp
  src/parameter_helper.cpp
  src/normal_vector.cpp
  src/visual_feature.cpp
  src/visual_features.cpp
  src/state_estimate.cpp
  src/trajectory_point.cpp
  src/trajectory.cpp
  src/trajectory_segment.cpp
  src/racing_trajectory.cpp
  src/control_command.cpp
  src/gate_feature.cpp
  src/gate_feature_array.cpp
  src/cubic_spline.cpp
  src/polynomial_curve.cpp
  src/polynomial_trajectory.cpp
  src/cubic_spline3d.cpp
  src/distparam_trajectory.cpp
  src/local_reference.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# add_executable(test_vision_common 
#   test/test_vision_common.cpp)
# target_link_libraries(test_vision_common ${catkin_LIBRARIES} ${PROJECT_NAME})

cs_install()
cs_export()
