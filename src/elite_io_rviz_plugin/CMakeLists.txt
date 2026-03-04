cmake_minimum_required(VERSION 3.5)
project(elite_io_rviz_plugin)

# 默认 C++14 或以上
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(pluginlib REQUIRED)
find_package(eli_common_interface REQUIRED)

# 让 Qt 的 MOC 系统工作
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(
  include
  ${Qt5Widgets_INCLUDE_DIRS}
)

# =============== ① 显式指定需要 MOC 的头文件 =================
set(MOC_HEADERS
  include/elite_io_rviz_plugin/io_control_panel.hpp
)

# =============== ② 调用 qt5_wrap_cpp 生成 moc 源文件 ==========
qt5_wrap_cpp(MOC_SOURCES ${MOC_HEADERS})

# =============== ③ 把 moc 源文件加入库 ========================
add_library(${PROJECT_NAME} SHARED
  src/io_control_panel.cpp
  ${MOC_SOURCES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  pluginlib
  eli_common_interface
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)

# 插件库名字与 plugin_description.xml 的 library path 对应
set_target_properties(${PROJECT_NAME}
  PROPERTIES
    OUTPUT_NAME "elite_io_rviz_plugin"
)

# 安装目标
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
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_package()
