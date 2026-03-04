#!/bin/bash
SOURCE_FILE="src/eli_cs_controllers/src/scaled_joint_trajectory_controller.cpp"
if [ -f "$SOURCE_FILE" ]; then
    sed -i 's/get_current_state()/joint_trajectory_controller::JointTrajectoryController::get_current_state()/g' "$SOURCE_FILE"
    echo "✓ 修复了 get_current_state 调用"
fi

HEADER_FILE="src/eli_cs_controllers/include/eli_cs_controllers/scaled_joint_trajectory_controller.hpp"
if [ -f "$HEADER_FILE" ]; then
    if ! grep -q "#include <angles/angles.h>" "$HEADER_FILE"; then
        sed -i '1i #include <angles/angles.h>' "$HEADER_FILE"
        echo "✓ 添加了angles头文件包含"
    fi
else
    echo "✗ 头文件不存在: $HEADER_FILE"
fi
echo "✓ 最终修复完成"
