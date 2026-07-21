cmake_minimum_required(VERSION 3.5)
project(elite_dashboard_rviz_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

find_package(eli_common_interface REQUIRED)
find_package(eli_dashboard_interface REQUIRED)
find_package(std_srvs REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(cs625_kinematics REQUIRED)
find_package(cs625_trajectory_tools REQUIRED)
find_package(cs625_task_manager REQUIRED)
find_package(cs625_compliant_placement REQUIRED)
find_package(cs625_state_monitor REQUIRED)
find_package(pointcloud_workflow_interfaces REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(
  include
  ${Qt5Widgets_INCLUDE_DIRS}
)

set(MOC_HEADERS
  include/elite_dashboard_rviz_plugin/elite_dashboard_panel.hpp
  include/elite_dashboard_rviz_plugin/tcp_client_panel.hpp
  include/elite_dashboard_rviz_plugin/goal_planner_panel.hpp
  include/elite_dashboard_rviz_plugin/task_execution_panel.hpp
  include/elite_dashboard_rviz_plugin/state_monitor_panel.hpp
  include/elite_dashboard_rviz_plugin/pointcloud_workflow_panel.hpp
  include/elite_dashboard_rviz_plugin/multi_config_planning_panel.hpp
)

qt5_wrap_cpp(MOC_SOURCES ${MOC_HEADERS})

add_library(${PROJECT_NAME} SHARED
  src/elite_dashboard_panel.cpp
  src/tcp_client_panel.cpp
  src/goal_planner_panel.cpp
  src/task_execution_panel.cpp
  src/state_monitor_panel.cpp
  src/pointcloud_workflow_panel.cpp
  src/multi_config_planning_panel.cpp
  ${MOC_SOURCES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  pluginlib
  eli_common_interface
  eli_dashboard_interface
  std_srvs
  tf2_ros
  tf2
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  cs625_kinematics
  cs625_trajectory_tools
  cs625_task_manager
  cs625_compliant_placement
  cs625_state_monitor
  pointcloud_workflow_interfaces
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)

set_target_properties(${PROJECT_NAME}
  PROPERTIES
    OUTPUT_NAME "elite_dashboard_rviz_plugin"
)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rviz_common
  pluginlib
  eli_common_interface
  eli_dashboard_interface
  std_srvs
  tf2_ros
  tf2
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  cs625_kinematics
  cs625_trajectory_tools
  cs625_task_manager
  cs625_compliant_placement
  cs625_state_monitor
  pointcloud_workflow_interfaces
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_package()
