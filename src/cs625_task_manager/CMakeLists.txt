cmake_minimum_required(VERSION 3.8)
project(cs625_task_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(eli_common_interface REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TaskState.msg"
  "srv/TriggerStep.srv"
  "srv/ConfirmAction.srv"
)

include_directories(
  include
)

add_executable(cs625_task_manager_node
  src/cs625_task_manager_node.cpp
  src/task_manager_main.cpp
)

ament_target_dependencies(cs625_task_manager_node
  rclcpp
  std_msgs
  eli_common_interface
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

target_link_libraries(cs625_task_manager_node
  ${cpp_typesupport_target}
)

install(TARGETS
  cs625_task_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
