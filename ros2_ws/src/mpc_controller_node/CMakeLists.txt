cmake_minimum_required(VERSION 3.8)
project(mpc_controller_node)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(fsc_vision_msgs REQUIRED)
find_package(mpc_core REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(mpc_controller_node src/mpc_controller_node.cpp)
ament_target_dependencies(mpc_controller_node
  rclcpp std_msgs geometry_msgs nav_msgs fsc_vision_msgs mpc_core Eigen3
)

install(TARGETS mpc_controller_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
