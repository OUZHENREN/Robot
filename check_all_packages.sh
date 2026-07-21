#!/bin/bash
echo "=== 工作空间中的所有包 ==="
colcon list

echo ""
echo "=== 尝试编译所有剩余的包 ==="
colcon build --event-handlers console_direct+
