#!/bin/bash
CALIBRATION_CPP="src/eli_cs_robot_calibration/src/calibration_correction.cpp"

if [ -f "$CALIBRATION_CPP" ]; then
    # 修复头文件路径（去掉多余的 'd'）
    sed -i 's/ament_index_cpp\/get_package_share_directory\.hpp/ament_index_cpp\/get_package_share_directory.hpp/g' "$CALIBRATION_CPP"
    
    # 显示修复后的行
    echo "=== 修复后的头文件包含 ==="
    grep -n "ament_index_cpp" "$CALIBRATION_CPP"
    echo "=========================="
    
    echo "✓ 修复了头文件拼写错误"
else
    echo "✗ 源文件不存在: $CALIBRATION_CPP"
fi
