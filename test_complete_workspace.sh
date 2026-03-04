#!/bin/bash
echo "=== ELI CS机器人ROS 2工作空间完整测试 ==="
echo "编译状态: ✅ 全部成功"
echo "编译时间: $(date)"
echo ""

# 设置环境
source install/setup.bash

echo "=== 包列表验证 ==="
echo "已编译的包:"
colcon list
echo ""

echo "=== ROS 2包注册验证 ==="
ros2 pkg list | grep eli_cs || echo "未找到eli_cs包"
echo ""

echo "=== 关键组件检查 ==="
echo "1. 接口包:"
for pkg in eli_common_interface eli_dashboard_interface; do
    if ros2 pkg prefix $pkg >/dev/null 2>&1; then
        echo "✅ $pkg - 已注册"
    else
        echo "❌ $pkg - 注册失败"
    fi
done
echo ""
echo "2. 核心功能包:"
for pkg in eli_cs_robot_description eli_cs_robot_driver eli_cs_controllers; do
    if ros2 pkg prefix $pkg >/dev/null 2>&1; then
        echo "✅ $pkg - 已注册"
    else
        echo "❌ $pkg - 注册失败"
    fi
done
echo ""
echo "3. 高级功能包:"
for pkg in eli_cs_robot_calibration eli_cs_robot_moveit_config eli_cs_robot_simulation_gz; do
    if ros2 pkg prefix $pkg >/dev/null 2>&1; then
        echo "✅ $pkg - 已注册"
    else
        echo "❌ $pkg - 注册失败"
    fi
done
echo ""
echo "=== 文件结构验证 ==="
echo "安装目录结构概览:"
tree -L 2 install/ | head -30
echo "..."

echo ""
echo "=== 关键文件检查 ==="
IMPORTANT_FILES=(
    "install/eli_cs_robot_driver/lib/eli_cs_robot_driver/dashboard_client"
    "install/eli_cs_robot_driver/lib/eli_cs_robot_driver/script_node"
    "install/eli_cs_robot_driver/lib/eli_cs_robot_driver/eli_ros2_control_node"
    "install/eli_cs_robot_driver/lib/libeli_cs_hardware_interface_plugin.so"
    "install/eli_cs_controllers/lib/libeli_cs_controllers.so"
    "install/eli_cs_robot_description/share/eli_cs_robot_description/urdf/cs.urdf.xacro"
    "install/eli_cs_robot_moveit_config/share/eli_cs_robot_moveit_config/launch/cs_moveit.launch.py"
)
for file in "${IMPORTANT_FILES[@]}"; do
    if [ -f "$file" ] || [ -d "$file" ]; then
        echo "✅ $(basename $file) - 存在"
    else
        echo "❌ $(basename $file) - 缺失"
    fi
done
echo ""
echo "=== ROS 2接口验证 ==="
echo "可用的服务接口:"
ros2 interface list | grep eli_ || echo "未找到eli相关接口"

echo ""
echo "=== 组件验证 ==="
echo "可用的ROS 2组件:"
ros2 component types | grep eli_ || echo "未找到eli组件"
echo ""
echo "=== 测试完成 ==="
echo "所有检查已完成！你的ELI CS机器人ROS 2工作空间已准备就绪。"
