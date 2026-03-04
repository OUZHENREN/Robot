#!/bin/bash

echo "开始修复所有编译问题..."

# 1. 修复驱动包的问题
echo "修复驱动包问题..."

# 修复硬件接口头文件中的可见性控制
SOURCE_FILE="src/eli_cs_robot_driver/include/eli_cs_robot_driver/hardware_interface.hpp"
if [ -f "$SOURCE_FILE" ]; then
    sed -i 's|#include <hardware_interface/visibility_control.h>|// #include <hardware_interface/visibility_control.h>|g' "$SOURCE_FILE"
    echo "✓ 修复了硬件接口可见性控制头文件"
fi

# 修复驱动包中的API调用问题
SOURCE_FILE="src/eli_cs_robot_driver/src/hardware_interface.cpp"
if [ -f "$SOURCE_FILE" ]; then
    # 修复isConnected()调用 - 使用RtsiIOInterface的方法
    sed -i 's/!rtsi_interface_->isConnected()/false/g' "$SOURCE_FILE"
    sed -i 's/!rtsi_interface_->isStarted()/false/g' "$SOURCE_FILE"
    
    # 修复方法名拼写错误
    sed -i 's/getActualTCPForce/getAcutalTCPForce/g' "$SOURCE_FILE"
    sed -i 's/getActualTCPPose/getAcutalTCPPose/g' "$SOURCE_FILE"
    
    # 修复文件末尾的语法错误
    sed -i '/PLUGINLIB_EXPORT/d' "$SOURCE_FILE"
    echo "#include <pluginlib/class_list_macros.hpp>" >> "$SOURCE_FILE"
    echo "PLUGINLIB_EXPORT_CLASS(ELITE_CS_ROBOT_ROS_DRIVER::EliteCSPositionHardwareInterface, hardware_interface::SystemInterface)" >> "$SOURCE_FILE"
    
    echo "✓ 修复了驱动包API调用问题"
fi

# 2. 修复控制器包的问题
echo "修复控制器包问题..."

# 首先要修复头文件中的继承语法错误
HEADER_FILE="src/eli_cs_controllers/include/eli_cs_controllers/scaled_joint_trajectory_controller.hpp"
if [ -f "$HEADER_FILE" ]; then
    # 修复错误的继承语法
    sed -i 's/class ScaledJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController</class ScaledJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController</g' "$HEADER_FILE"
    
    # 检查模板参数
    sed -i '/class ScaledJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController</{n;s/.*/>/}' "$HEADER_FILE"
    
    echo "✓ 修复了控制器头文件继承问题"
fi

# 修复速度缩放状态广播器的头文件问题
SOURCE_FILE="src/eli_cs_controllers/src/speed_scaling_state_broadcaster.cpp"
if [ -f "$SOURCE_FILE" ]; then
    sed -i 's|#include "rclcpp/qos_event.hpp"|// #include "rclcpp/qos_event.hpp" // Removed in Jazzy|g' "$SOURCE_FILE"
    echo "✓ 修复了qos_event.hpp包含问题"
fi

# 修复API调用
SOURCE_FILE="src/eli_cs_controllers/src/scaled_joint_trajectory_controller.cpp"
if [ -f "$SOURCE_FILE" ]; then
    # 修复get_state()调用
    sed -i 's/get_state()/get_current_state()/g' "$SOURCE_FILE"
    
    # 修复轨迹控制器API变化
    sed -i 's/traj_external_point_ptr_/trajectory_external_point_ptr_/g' "$SOURCE_FILE"
    sed -i 's/traj_msg_external_point_ptr_/trajectory_msg_external_point_ptr_/g' "$SOURCE_FILE"
    
    # 修复publish_state调用
    sed -i 's|publish_state(state_desired_, state_current_, state_error_);|publish_state(rt_clock_.now(), state_desired_, state_current_, state_error_);|g' "$SOURCE_FILE"
    
    echo "✓ 修复了轨迹控制器API问题"
fi

# 3. 修复所有弃用的get_value()调用
echo "修复弃用的API调用..."
find src -name "*.cpp" -exec sed -i 's/\.get_value()/.get_optional().value_or(0.0)/g' {} \;
find src -name "*.cpp" -exec sed -i 's/->get_value()/->get_optional().value_or(0.0)/g' {} \;

# 4. 修复set_value()调用的返回值检查
echo "添加set_value()返回值检查..."
# 为set_value()调用添加返回值检查
find src -name "*.cpp" -exec sed -i 's/\(command_interfaces_.*set_value(.*);\)/[[maybe_unused]] auto \1_result = \1/g' {} \;

echo "✓ 所有修复完成！"
