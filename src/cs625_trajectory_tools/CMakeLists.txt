cmake_minimum_required(VERSION 3.8)
project(cs625_trajectory_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

set(SRV_FILES
  "srv/NearestPathPoint.srv"
  "srv/PlanToFrame.srv"
  "srv/PlanToPose.srv"
  "srv/PlanToTarget.srv"
  "srv/ExecuteLastPlan.srv"
  "srv/SetNamedPose.srv"
  "srv/SetNamedPoseFromCurrent.srv"
  "srv/SetNamedTarget.srv"
  "srv/SetNamedTargetFromCurrent.srv"
  "srv/GetNamedPose.srv"
  "srv/GetNamedTarget.srv"
  "srv/CartesianPlanToFrame.srv"
  "srv/CartesianForceCompensatedToFrame.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${SRV_FILES}
  DEPENDENCIES geometry_msgs sensor_msgs trajectory_msgs
)

include_directories(
  include
)

add_executable(planned_tcp_path_node
  src/planned_tcp_path_node.cpp
)

ament_target_dependencies(planned_tcp_path_node
  rclcpp
  nav_msgs
  geometry_msgs
  trajectory_msgs
  moveit_ros_planning_interface
  moveit_core
)

add_executable(moveit_planner_client_node
  src/moveit_planner_client_node.cpp
)

ament_target_dependencies(moveit_planner_client_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  tf2_ros
  tf2
)

add_executable(display_trajectory_bridge_node
  src/display_trajectory_bridge_node.cpp
)

ament_target_dependencies(display_trajectory_bridge_node
  rclcpp
  trajectory_msgs
  moveit_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

target_link_libraries(planned_tcp_path_node
  ${cpp_typesupport_target}
)

target_link_libraries(moveit_planner_client_node
  ${cpp_typesupport_target}
)

target_link_libraries(display_trajectory_bridge_node
  ${cpp_typesupport_target}
)

install(TARGETS
  planned_tcp_path_node
  moveit_planner_client_node
  display_trajectory_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
