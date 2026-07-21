#!/bin/bash
echo "🔄 ELI CS625机器人终极恢复脚本"

# 方法1: 尝试使用原始launch文件
echo "方法1: 尝试原始启动方式"
source install/setup.bash

timeout 10 ros2 launch eli_cs_robot_description view_cs.launch.py cs_type:=cs625 2>&1 | head -20
if [ ${PIPESTATUS[0]} -eq 0 ]; then
    echo "✅ 原始启动成功!"
    exit 0
else
    echo "❌ 原始启动失败，使用方法2"
fi
# 方法2: 简化URDF启动
echo ""
echo "方法2: 简化URDF启动"
./launch_cs625_direct.sh
