cmake_minimum_required(VERSION 3.8)
project(px4_offboard_bridge)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(fsc_vision_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(px4_offboard_bridge src/px4_offboard_bridge.cpp)
target_include_directories(px4_offboard_bridge PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(px4_offboard_bridge
  rclcpp std_msgs nav_msgs fsc_vision_msgs px4_msgs Eigen3
)

install(TARGETS px4_offboard_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
