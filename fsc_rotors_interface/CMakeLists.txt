cmake_minimum_required(VERSION 2.8.3)
project(fsc_rotors_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-O3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_executable(fsc_rotors_interface src/fsc_rotors_interface.cpp)

cs_install()
cs_export()
