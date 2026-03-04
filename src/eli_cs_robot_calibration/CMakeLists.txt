cmake_minimum_required(VERSION 3.8)
project(eli_cs_robot_calibration)

# 设置 C++ 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)

# 创建库目标
add_library(calibration SHARED
  src/calibration_library.cpp
)

target_include_directories(calibration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 为目标添加依赖
ament_target_dependencies(calibration
  rclcpp
  ament_index_cpp
)

# 创建可执行文件
ament_package()
