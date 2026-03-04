#!/bin/bash
# 切换到工作空间目录
cd ~/elite_ros_ws
# 检查并启动交互式向导
if [ -f "cs625_wizard.sh" ]; then
    ./cs625_wizard.sh
else
    echo "错误: 未找到cs625_wizard.sh"
    echo "请确保在elite_ros_ws目录中运行"
fi
