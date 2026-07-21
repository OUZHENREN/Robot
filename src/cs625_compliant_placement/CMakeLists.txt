cmake_minimum_required(VERSION 3.8)
project(cs625_compliant_placement)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cs625_state_monitor REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CompliantPlacementState.msg"
  "srv/GetCompliantPlacementState.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

include_directories(include)

add_executable(compliant_placement_node
  src/main.cpp
  src/contact_detector.cpp
  src/compliant_placement_node.cpp
)

ament_target_dependencies(compliant_placement_node
  rclcpp
  std_msgs
  builtin_interfaces
  cs625_state_monitor
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(compliant_placement_node "${cpp_typesupport_target}")

install(
  TARGETS
    compliant_placement_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
