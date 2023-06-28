cmake_minimum_required(VERSION 3.0.2)
project(trajectory_generation_helper)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-O3)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME} src/polynomial_trajectory_helper.cpp
	src/heading_trajectory_helper.cpp src/circle_trajectory_helper.cpp)

cs_install()
cs_export()
