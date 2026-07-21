#!/bin/bash
PACKAGE_XML="src/eli_cs_robot_calibration/package.xml"

if [ -f "$PACKAGE_XML" ]; then
    # 检查是否已经有ament_index_cpp依赖
    if ! grep -q "ament_index_cpp" "$PACKAGE_XML"; then
        # 在<buildtool_depend>ament_cmake</buildtool_depend>后面添加依赖
        sed -i '/<buildtool_depend>ament_cmake<\/buildtool_depend>/a\  <depend>ament_index_cpp</depend>' "$PACKAGE_XML"
        echo "✓ 在 package.xml 中添加了 ament_index_cpp 依赖"
    else
        echo "✓ ament_index_cpp 依赖已存在"
    fi
else
    echo "✗ package.xml 文件不存在: $PACKAGE_XML"
fi
