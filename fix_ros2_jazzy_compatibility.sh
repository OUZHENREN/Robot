#!/bin/bash

# 修复硬件接口可见性控制头文件
SOURCE_FILE="src/eli_cs_robot_driver/include/eli_cs_robot_driver/hardware_interface.hpp"
if [ -f "$SOURCE_FILE" ]; then
    sed -i 's|#include <hardware_interface/visibility_control.h>|// #include <hardware_interface/visibility_control.h>|g' "$SOURCE_FILE"
    echo "Fixed visibility_control.h include in $SOURCE_FILE"
fi

# 修复 speed_scaling_state_broadcaster.cpp 中的 qos_event.hpp 问题
SOURCE_FILE="src/eli_cs_controllers/src/speed_scaling_state_broadcaster.cpp"
if [ -f "$SOURCE_FILE" ]; then
    sed -i 's|#include "rclcpp/qos_event.hpp"|// #include "rclcpp/qos_event.hpp" // Removed in Jazzy|g' "$SOURCE_FILE"
    echo "Fixed qos_event.hpp include in $SOURCE_FILE"
fi

# 修复 scaled_joint_trajectory_controller.cpp 中的 API 问题
SOURCE_FILE="src/eli_cs_controllers/src/scaled_joint_trajectory_controller.cpp"
if [ -f "$SOURCE_FILE" ]; then
    # 修复 get_state() 调用
    sed -i 's/get_state()/get_current_state()/g' "$SOURCE_FILE"
    
    # 修复轨迹控制器 API 变化
    sed -i 's/traj_external_point_ptr_/trajectory_external_point_ptr_/g' "$SOURCE_FILE"
    sed -i 's/traj_msg_external_point_ptr_/trajectory_msg_external_point_ptr_/g' "$SOURCE_FILE"
    
    # 修复 publish_state 调用
    sed -i 's|publish_state(state_desired_, state_current_, state_error_);|publish_state(rt_clock_.now(), state_desired_, state_current_, state_error_);|g' "$SOURCE_FILE"
    
    echo "Fixed API issues in $SOURCE_FILE"
fi

# 修复所有弃用的 get_value() 调用，替换为 get_optional()
find src -name "*.cpp" -exec sed -i 's/\.get_value()/.get_optional().value_or(0.0)/g' {} \;
find src -name "*.cpp" -exec sed -i 's/->get_value()/->get_optional().value_or(0.0)/g' {} \;

# 修复数据类型问题（float 相关的 variant 错误）
SOURCE_FILE="src/eli_cs_controllers/src/gpio_controller.cpp"
if [ -f "$SOURCE_FILE" ]; then
    # 将 float 类型的 mass 改为 double
    sed -i 's/req->mass/static_cast<double>(req->mass)/g' "$SOURCE_FILE"
    echo "Fixed float to double cast in $SOURCE_FILE"
fi

echo "ROS2 Jazzy compatibility fixes applied!"
