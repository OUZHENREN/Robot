#!/bin/bash
# =======================================================
# ==      艾利特机器人 & ROS 2 系统统一启动脚本        ==
# =======================================================

echo "--- 正在加载ROS 2工作空间环境..."
source /opt/ros/jazzy/setup.bash
source ~/elite_ros_ws/install/setup.bash

echo "--- 环境加载成功！"
echo "--- 即将启动 tcp_bridge 包中的 cs625_launcher.py..."
sleep 1

# 【【核心】】使用 ros2 launch 调用正确的 Python 启动文件
ros2 launch tcp_bridge cs625_launcher.py

echo ""
echo "--- ROS 2 Launch 进程已结束。---"
echo "--- 按下 [Enter] 键关闭此终端。 ---"
read

