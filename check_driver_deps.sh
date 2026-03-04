#!/bin/bash
PACKAGE_XML="src/eli_cs_robot_driver/package.xml"

if [ -f "$PACKAGE_XML" ]; then
    echo "=== eli_cs_robot_driver package.xml 内容 ==="
    cat "$PACKAGE_XML"
    echo "==========================================="
else
    echo "✗ package.xml 文件不存在: $PACKAGE_XML"
fi
