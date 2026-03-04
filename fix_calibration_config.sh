#!/bin/bash
PACKAGE_XML="src/eli_cs_robot_calibration/package.xml"
CMAKELISTS="src/eli_cs_robot_calibration/CMakeLists.txt"

# 修复 package.xml
if [ -f "$PACKAGE_XML" ]; then
    # 确保有正确的依赖
    if ! grep -q "ament_index_cpp" "$PACKAGE_XML"; then
        sed -i '/<buildtool_depend>ament_cmake<\/buildtool_depend>/a\  <depend>ament_index_cpp</depend>' "$PACKAGE_XML"
        echo "✓ 添加了 ament_index_cpp 依赖到 package.xml"
    fi
fi

# 修复 CMakeLists.txt
if [ -f "$CMAKELISTS" ]; then
    # 添加 find_package
    if ! grep -q "find_package(ament_index_cpp REQUIRED)" "$CMAKELISTS"; then
        sed -i '/find_package(ament_cmake REQUIRED)/a find_package(ament_index_cpp REQUIRED)' "$CMAKELISTS"
    fi
    
    # 添加链接库
    if grep -q "target_link_libraries(calibration_correction" "$CMAKELISTS"; then
        if ! grep -q "ament_index_cpp" "$CMAKELISTS" | grep -A2 "target_link_libraries"; then
            sed -i 's/target_link_libraries(calibration_correction\(.*\))/target_link_libraries(calibration_correction\1 ament_index_cpp::ament_index_cpp)/' "$CMAKELISTS"
        fi
    fi
    echo "✓ 修复了 CMakeLists.txt"
    
    # 显示相关部分
    echo "=== CMakeLists.txt 相关内容 ==="
    grep -A5 -B5 "calibration_correction" "$CMAKELISTS" | head -20
    echo "..."
    echo "=============================="
fi
