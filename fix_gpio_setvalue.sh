#!/bin/bash
SOURCE_FILE="src/eli_cs_controllers/src/gpio_controller.cpp"
if [ -f "$SOURCE_FILE" ]; then
    sed -i 's/command_interfaces_\(.*\)\.set_value(\(.*\));/(void)command_interfaces_\1.set_value(\2);/g' "$SOURCE_FILE"
    echo "✓ 修复了GPIO控制器中的set_value调用"
else
    echo "✗ 文件不存在: $SOURCE_FILE"
fi
