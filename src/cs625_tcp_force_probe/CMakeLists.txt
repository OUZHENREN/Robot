cmake_minimum_required(VERSION 3.8)
project(cs625_tcp_force_probe)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(tcp_force_probe_node src/tcp_force_probe_node.cpp)

ament_target_dependencies(tcp_force_probe_node
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS
  tcp_force_probe_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
