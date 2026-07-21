#!/bin/bash
echo "=== 检查启动文件参数 ==="
source install/setup.bash

echo "1. 查看机器人描述启动文件参数:"
ros2 launch eli_cs_robot_description view_cs.launch.py --show-args

echo ""
echo "2. 查看仿真启动文件参数:"
ros2 launch eli_cs_robot_simulation_gz cs_sim_control.launch.py --show-args

echo ""
echo "3. 查看MoveIt启动文件参数:"
ros2 launch eli_cs_robot_moveit_config cs_moveit.launch.py --show-args

echo ""
echo "4. 查看控制启动文件参数:"
ros2 launch eli_cs_robot_driver elite_control.launch.py --show-args
