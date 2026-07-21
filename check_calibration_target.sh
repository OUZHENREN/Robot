#!/bin/bash
CMAKELISTS="src/eli_cs_robot_calibration/CMakeLists.txt"

if [ -f "$CMAKELISTS" ]; then
    echo "=== 完整的 CMakeLists.txt 内容 ==="
    cat "$CMAKELISTS"
    echo "=================================="
else
    echo "✗ CMakeLists.txt 文件不存在"
fi
