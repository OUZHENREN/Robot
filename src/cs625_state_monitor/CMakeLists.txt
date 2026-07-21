cmake_minimum_required(VERSION 3.8)
project(cs625_state_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CS625State.msg"
  "srv/StateLoggingControl.srv"
  DEPENDENCIES std_msgs
)

install(
  DIRECTORY cs625_state_monitor
  DESTINATION lib/python3.12/site-packages
  PATTERN "__pycache__" EXCLUDE
)

install(
  PROGRAMS
    cs625_state_monitor/raw_state_receiver_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME raw_state_receiver_node
)

install(
  PROGRAMS
    cs625_state_monitor/state_csv_logger_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME state_csv_logger_node
)

install(
  PROGRAMS
    cs625_state_monitor/sim_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sim_state_publisher
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
