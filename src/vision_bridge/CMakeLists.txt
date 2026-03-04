cmake_minimum_required(VERSION 3.8)
project(vision_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ===================== 添加我们自己的编译规则 =====================
# 添加可执行文件定义
add_executable(tcp_pose_receiver src/tcp_pose_receiver.cpp)

# 链接依赖库
ament_target_dependencies(
  tcp_pose_receiver
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 安装可执行文件，这样才能通过 ros2 run 找到它
install(TARGETS
  tcp_pose_receiver
  DESTINATION lib/${PROJECT_NAME}
)
# ====================================================================

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  ament_lint_auto(
#    LINT_EXCLUDE
#      "src/tcp_pose_receiver.cpp"
#  )
#endif()

install(
  FILES
  "package.xml"
  DESTINATION "share/${PROJECT_NAME}"
)

ament_package()

