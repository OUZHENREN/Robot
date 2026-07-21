cmake_minimum_required(VERSION 3.8)
project(eli_cs_robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# Global RPATH policy
# =========================
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)

# =========================
# Force Elite SDK from /opt only
# =========================
set(ELITE_SDK_ROOT "/opt/elite-sdk-custom")
set(ELITE_SDK_INCLUDE_HINT "${ELITE_SDK_ROOT}/include")
set(ELITE_SDK_LIBRARY_HINT "${ELITE_SDK_ROOT}/lib")
set(elite-cs-series-sdk_DIR "${ELITE_SDK_ROOT}/lib/cmake/elite-cs-series-sdk")

message(STATUS "ELITE_SDK_ROOT=${ELITE_SDK_ROOT}")
message(STATUS "elite-cs-series-sdk_DIR=${elite-cs-series-sdk_DIR}")

find_path(ELITE_SDK_INCLUDE_DIR
  NAMES Elite/EliteDriver.hpp
  PATHS ${ELITE_SDK_INCLUDE_HINT}
  NO_DEFAULT_PATH
)

find_library(ELITE_SDK_LIBRARY
  NAMES elite-cs-series-sdk
  PATHS ${ELITE_SDK_LIBRARY_HINT}
  NO_DEFAULT_PATH
)

if(NOT ELITE_SDK_INCLUDE_DIR)
  message(FATAL_ERROR "Elite SDK headers not found in ${ELITE_SDK_INCLUDE_HINT}")
endif()

if(NOT ELITE_SDK_LIBRARY)
  message(FATAL_ERROR "Elite SDK library not found in ${ELITE_SDK_LIBRARY_HINT}")
endif()

message(STATUS "ELITE_SDK_INCLUDE_DIR=${ELITE_SDK_INCLUDE_DIR}")
message(STATUS "ELITE_SDK_LIBRARY=${ELITE_SDK_LIBRARY}")

find_package(elite-cs-series-sdk 1.2.0 REQUIRED NO_DEFAULT_PATH
  PATHS ${elite-cs-series-sdk_DIR}
)

# =========================
# Find dependencies
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(eli_dashboard_interface REQUIRED)
find_package(eli_common_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(backward_ros REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(
  THIS_PACKAGE_DEPENDENCIES
  eli_common_interface
  eli_dashboard_interface
  hardware_interface
  rclcpp_lifecycle
  pluginlib
  tf2_geometry_msgs
  std_srvs
  rclcpp
  rclcpp_components
  controller_manager_msgs
  rclcpp_action
  control_msgs
  sensor_msgs
  trajectory_msgs
)

# =========================
# Helper function:
# bind target to custom SDK only
# =========================
function(configure_elite_sdk_target target_name)
  target_include_directories(
    ${target_name}
    BEFORE PRIVATE
    ${ELITE_SDK_INCLUDE_DIR}
    include
  )

  # IMPORTANT:
  # Use plain signature here to stay compatible with ament_target_dependencies(),
  # which also calls target_link_libraries() internally.
  target_link_libraries(
    ${target_name}
    ${ELITE_SDK_LIBRARY}
  )

  set_target_properties(${target_name} PROPERTIES
    BUILD_RPATH "${ELITE_SDK_LIBRARY_HINT}"
    INSTALL_RPATH "${ELITE_SDK_LIBRARY_HINT}"
  )
endfunction()

# =========================
# Dashboard client component
# =========================
add_library(dashboard_client_component SHARED src/dashboard_client.cpp)
configure_elite_sdk_target(dashboard_client_component)
ament_target_dependencies(dashboard_client_component ${THIS_PACKAGE_DEPENDENCIES})

rclcpp_components_register_node(
  dashboard_client_component
  PLUGIN "ELITE_CS_ROBOT_ROS_DRIVER::DashboardClient"
  EXECUTABLE dashboard_client
)

ament_export_targets(export_dashboard_client_component)

install(
  TARGETS dashboard_client_component
  EXPORT export_dashboard_client_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# =========================
# Script sender component
# =========================
add_library(script_node_component SHARED src/script_node.cpp)
configure_elite_sdk_target(script_node_component)
ament_target_dependencies(script_node_component ${THIS_PACKAGE_DEPENDENCIES})

rclcpp_components_register_node(
  script_node_component
  PLUGIN "ELITE_CS_ROBOT_ROS_DRIVER::ScriptNode"
  EXECUTABLE script_node
)

ament_export_targets(export_script_node_component)

install(
  TARGETS script_node_component
  EXPORT export_script_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# =========================
# Components loader
# =========================
add_executable(
  eli_components_loader
  src/load_components.cpp
)

target_include_directories(
  eli_components_loader
  BEFORE PRIVATE
  ${ELITE_SDK_INCLUDE_DIR}
  include
)

target_link_libraries(
  eli_components_loader
  script_node_component
  dashboard_client_component
  ${ELITE_SDK_LIBRARY}
)

set_target_properties(eli_components_loader PROPERTIES
  BUILD_RPATH "${ELITE_SDK_LIBRARY_HINT}"
  INSTALL_RPATH "${ELITE_SDK_LIBRARY_HINT}"
)

ament_target_dependencies(eli_components_loader ${THIS_PACKAGE_DEPENDENCIES})

install(
  TARGETS eli_components_loader
  DESTINATION lib/${PROJECT_NAME}
)

# =========================
# Hardware interface plugin
# =========================
add_library(
  eli_cs_hardware_interface_plugin
  SHARED
  src/hardware_interface.cpp
)

configure_elite_sdk_target(eli_cs_hardware_interface_plugin)

ament_target_dependencies(
  eli_cs_hardware_interface_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_DEPENDENCIES}
)

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

install(
  TARGETS eli_cs_hardware_interface_plugin
  DESTINATION lib
)

# =========================
# Controller stopper node
# =========================
add_executable(
  controller_stopper_node
  src/controller_stopper.cpp
  src/controller_stopper_node.cpp
)

target_include_directories(
  controller_stopper_node
  PRIVATE
  include
)

ament_target_dependencies(
  controller_stopper_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_DEPENDENCIES}
)

install(
  TARGETS controller_stopper_node
  DESTINATION lib/${PROJECT_NAME}
)

# =========================
# eli_ros2_control_node
# =========================
add_executable(
  eli_ros2_control_node
  src/control_node.cpp
)

ament_target_dependencies(
  eli_ros2_control_node
  controller_manager
  rclcpp
)

install(
  TARGETS eli_ros2_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# =========================
# Install resources
# =========================
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY resources
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# =========================
# Install examples
# =========================
install(
  DIRECTORY example/python example/config/
  DESTINATION share/${PROJECT_NAME}
)

add_executable(
  example_safe_joint_move
  example/cpp/example_safe_joint_move.cpp
)

target_include_directories(
  example_safe_joint_move
  BEFORE PRIVATE
  ${ELITE_SDK_INCLUDE_DIR}
)

target_link_libraries(
  example_safe_joint_move
  ${ELITE_SDK_LIBRARY}
)

set_target_properties(example_safe_joint_move PROPERTIES
  BUILD_RPATH "${ELITE_SDK_LIBRARY_HINT}"
  INSTALL_RPATH "${ELITE_SDK_LIBRARY_HINT}"
)

ament_target_dependencies(
  example_safe_joint_move
  ${THIS_PACKAGE_DEPENDENCIES}
)

install(
  TARGETS example_safe_joint_move
  DESTINATION lib/${PROJECT_NAME}
)

# =========================
# Export
# =========================
ament_export_include_directories(
  include
)

ament_export_libraries(
  eli_cs_hardware_interface_plugin
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ${THIS_PACKAGE_DEPENDENCIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
