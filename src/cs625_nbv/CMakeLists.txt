cmake_minimum_required(VERSION 3.8)
project(cs625_nbv)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ROS2 core
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# PCL
find_package(PCL REQUIRED COMPONENTS common io registration filters surface)
find_package(pcl_conversions REQUIRED)

# Eigen
find_package(Eigen3 REQUIRED)

# TF2
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

# Workspace packages
find_package(cs625_kinematics REQUIRED)
find_package(cs625_state_monitor REQUIRED)
find_package(pointcloud_workflow_interfaces REQUIRED)

# ---------------------------------------------------------------------------
# Message / Service / Action definitions
# ---------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ViewpointCandidate.msg"
  "msg/EpisodeResult.msg"
  "srv/RunNbvEpisode.srv"
  "srv/GetExperimentResults.srv"
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

# ---------------------------------------------------------------------------
# Library: NBV core
# ---------------------------------------------------------------------------
set(NBV_CORE_SOURCES
  src/viewpoint_sampler.cpp
  src/pose_estimator.cpp
  src/covariance_estimator.cpp
  src/information_gain.cpp
  src/nbv_orchestrator.cpp
  src/baseline_strategies.cpp
  src/experiment_logger.cpp
)

add_library(cs625_nbv_core STATIC ${NBV_CORE_SOURCES})
target_include_directories(cs625_nbv_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(cs625_nbv_core
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)
target_compile_definitions(cs625_nbv_core PRIVATE ${PCL_DEFINITIONS})
ament_target_dependencies(cs625_nbv_core
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  moveit_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_conversions
  cs625_kinematics
  cs625_state_monitor
)
# Link to generated message/service interfaces from this package
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(cs625_nbv_core "${cpp_typesupport_target}")

# ---------------------------------------------------------------------------
# Nodes
# ---------------------------------------------------------------------------
# Main NBV server node
add_executable(nbv_server_node src/nbv_server_node.cpp)
target_link_libraries(nbv_server_node cs625_nbv_core)
ament_target_dependencies(nbv_server_node
  rclcpp
  geometry_msgs
  cs625_kinematics
  cs625_state_monitor
)

# Viewpoint visualization publisher
add_executable(viewpoint_publisher_node src/viewpoint_publisher_node.cpp)
target_link_libraries(viewpoint_publisher_node cs625_nbv_core)
ament_target_dependencies(viewpoint_publisher_node
  rclcpp
  visualization_msgs
  geometry_msgs
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS cs625_nbv_core nbv_server_node viewpoint_publisher_node
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS cs625_nbv_core nbv_server_node viewpoint_publisher_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(nbv_server_node "${cpp_typesupport_target}")
target_link_libraries(viewpoint_publisher_node "${cpp_typesupport_target}")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  PCL
  Eigen3
  geometry_msgs
  sensor_msgs
)
ament_package()
