#!/bin/bash

gnome-terminal -- bash -lc '
set -e
cd /home/yff/elite_ros_ws

# 1. 先 source ROS2
source /opt/ros/jazzy/setup.bash

# 2. 再 source 工作区
source /home/yff/elite_ros_ws/install/setup.bash

# 3. 显式把 rviz 插件两个包加入 AMENT_PREFIX_PATH（如果不在的话）
export AMENT_PREFIX_PATH=/home/yff/elite_ros_ws/install/elite_io_rviz_plugin:/home/yff/elite_ros_ws/install/elite_dashboard_rviz_plugin:$AMENT_PREFIX_PATH

echo "===== 桌面启动环境（强制补全版）====="
echo "AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH"
echo "CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH"

# 4. 启动 launch
ros2 launch cs625_full_system cs625_full_system.launch.py \
  robot_ip:=192.168.1.200 \
  cs_type:=cs625 \
  headless_mode:=true \
  use_fake_hardware:=false \
  launch_rviz:=true \
  safety_limits:=false \
  fake_sensor_commands:=false \
  use_tool_communication:=false \
  local_ip:=192.168.1.102

echo
echo "========== 启动结束或被中断，按 Enter 关闭窗口 =========="
read
'
