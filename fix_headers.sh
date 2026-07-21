#!/bin/bash

# 修复 scaled_joint_trajectory_controller.hpp 中的继承问题
HEADER_FILE="src/eli_cs_controllers/include/eli_cs_controllers/scaled_joint_trajectory_controller.hpp"
if [ -f "$HEADER_FILE" ]; then
    # 检查基类名称
    sed -i 's/class ScaledJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController/class ScaledJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController</g' "$HEADER_FILE"
    
    # 添加模板参数
    sed -i '/class ScaledJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController</a\
  <hardware_interface::HWInterfaceType>' "$HEADER_FILE"
    
    echo "Fixed header inheritance in $HEADER_FILE"
fi

# 修复硬件接口插件注册
SOURCE_FILE="src/eli_cs_robot_driver/src/hardware_interface.cpp"
if [ -f "$SOURCE_FILE" ]; then
    # 检查插件注册宏
    grep -n "PLUGINLIB_EXPORT" "$SOURCE_FILE"
    
    # 如果使用旧宏，更新为新的
    sed -i 's/PLUGINLIB_EXPORT_CLASS/PLUGINLIB_EXPORT/g' "$SOURCE_FILE" 2>/dev/null
    echo "Fixed plugin export macro in $SOURCE_FILE"
fi
