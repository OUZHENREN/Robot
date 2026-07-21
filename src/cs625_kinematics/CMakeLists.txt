cmake_minimum_required(VERSION 3.8)
project(cs625_kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

find_package(std_msgs REQUIRED)

set(msg_files
  "msg/IkSolution.msg"
)

set(srv_files
  "srv/SolveIK.srv"
  "srv/SolveIKAll.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES geometry_msgs sensor_msgs trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

add_library(${PROJECT_NAME}_core
  src/fk_cs625.cpp
  src/ik_solver.cpp
  src/config_classifier.cpp
  src/ros_conversions.cpp
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  Eigen3
  rosidl_default_runtime
  rosidl_typesupport_cpp
)

set(interfaces_target "")
foreach(t
  ${PROJECT_NAME}__cpp
  ${PROJECT_NAME}__rosidl_typesupport_cpp
  ${PROJECT_NAME}_generate_interfaces_cpp
)
  if(TARGET ${t})
    set(interfaces_target ${t})
    break()
  endif()
endforeach()

if(interfaces_target)
  add_dependencies(${PROJECT_NAME}_core ${interfaces_target})
endif()

set(typesupport_cpp_target "")
if(TARGET ${PROJECT_NAME}__rosidl_typesupport_cpp)
  set(typesupport_cpp_target ${PROJECT_NAME}__rosidl_typesupport_cpp)
endif()

add_executable(ik_server_node
  src/ik_server_node.cpp
)

add_executable(preview_joint_state_relay
  src/preview_joint_state_relay.cpp
)

ament_target_dependencies(preview_joint_state_relay
  rclcpp
  sensor_msgs
)

add_executable(preview_robot_description_publisher
  src/preview_robot_description_publisher.cpp
)

ament_target_dependencies(preview_robot_description_publisher
  rclcpp
  std_msgs
)

install(TARGETS
  preview_joint_state_relay
  preview_robot_description_publisher
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(ik_server_node
  ${PROJECT_NAME}_core
)

ament_target_dependencies(ik_server_node
  rclcpp
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  rosidl_default_runtime
  rosidl_typesupport_cpp
)

if(interfaces_target)
  add_dependencies(ik_server_node ${interfaces_target})
endif()

if(typesupport_cpp_target)
  target_link_libraries(ik_server_node ${typesupport_cpp_target})
endif()

if(BUILD_TESTING)
  ament_add_gtest(test_fk_vs_matlab
    test/test_fk_vs_matlab.cpp
  )
  if(TARGET test_fk_vs_matlab)
    target_link_libraries(test_fk_vs_matlab ${PROJECT_NAME}_core)
    ament_target_dependencies(test_fk_vs_matlab
      Eigen3
      geometry_msgs
      sensor_msgs
      trajectory_msgs
      rosidl_default_runtime
      rosidl_typesupport_cpp
    )
    if(interfaces_target)
      add_dependencies(test_fk_vs_matlab ${interfaces_target})
    endif()
    if(typesupport_cpp_target)
      target_link_libraries(test_fk_vs_matlab ${typesupport_cpp_target})
    endif()
  endif()

  ament_add_gtest(test_ik_single
    test/test_ik_single.cpp
  )
  if(TARGET test_ik_single)
    target_link_libraries(test_ik_single ${PROJECT_NAME}_core)
    ament_target_dependencies(test_ik_single
      Eigen3
      geometry_msgs
      sensor_msgs
      trajectory_msgs
      rosidl_default_runtime
      rosidl_typesupport_cpp
    )
    if(interfaces_target)
      add_dependencies(test_ik_single ${interfaces_target})
    endif()
    if(typesupport_cpp_target)
      target_link_libraries(test_ik_single ${typesupport_cpp_target})
    endif()
  endif()

  ament_add_gtest(test_ik_all
    test/test_ik_all.cpp
  )
  if(TARGET test_ik_all)
    target_link_libraries(test_ik_all ${PROJECT_NAME}_core)
    ament_target_dependencies(test_ik_all
      Eigen3
      geometry_msgs
      sensor_msgs
      trajectory_msgs
      rosidl_default_runtime
      rosidl_typesupport_cpp
    )
    if(interfaces_target)
      add_dependencies(test_ik_all ${interfaces_target})
    endif()
    if(typesupport_cpp_target)
      target_link_libraries(test_ik_all ${typesupport_cpp_target})
    endif()
  endif()
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(TARGETS
  ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ik_server_node
  DESTINATION lib/${PROJECT_NAME}
  
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)

ament_package()
