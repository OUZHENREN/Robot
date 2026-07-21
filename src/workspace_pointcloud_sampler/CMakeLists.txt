cmake_minimum_required(VERSION 3.8)
project(workspace_pointcloud_sampler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cs625_trajectory_tools REQUIRED)
find_package(cs625_state_monitor REQUIRED)
find_package(pointcloud_workflow_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(
  include
)

add_executable(workspace_pointcloud_sampler_node
  src/workspace_pointcloud_sampler_node.cpp
)

ament_target_dependencies(workspace_pointcloud_sampler_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cs625_trajectory_tools
  cs625_state_monitor
  pointcloud_workflow_interfaces
  nav_msgs
)

install(TARGETS
  workspace_pointcloud_sampler_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()
