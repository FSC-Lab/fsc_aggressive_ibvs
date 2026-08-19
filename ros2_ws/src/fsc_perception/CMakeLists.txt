cmake_minimum_required(VERSION 3.8)
project(fsc_perception)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(fsc_vision_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(target_bearing_node src/target_bearing_node.cpp)
ament_target_dependencies(target_bearing_node
  rclcpp std_msgs geometry_msgs nav_msgs fsc_vision_msgs
)
target_link_libraries(target_bearing_node Eigen3::Eigen)

install(TARGETS target_bearing_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config gz DESTINATION share/${PROJECT_NAME})

ament_package()
