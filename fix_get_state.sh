#!/bin/bash
SOURCE_FILE="src/eli_cs_controllers/src/scaled_joint_trajectory_controller.cpp"
if [ -f "$SOURCE_FILE" ]; then
    sed -i 's/joint_trajectory_controller::JointTrajectoryController::get_current_state()/get_state()/g' "$SOURCE_FILE"
    echo "✓ 修复了 get_state 调用"
else
    echo "✗ 源文件不存在: $SOURCE_FILE"
fi
