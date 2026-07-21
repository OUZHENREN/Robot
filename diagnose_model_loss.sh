#!/bin/bash
echo "🔍 分析模型丢失原因并尝试恢复"

echo "=== 问题分析 ==="
echo "1. 原始的xacro文件可能存在语法错误"
echo "2. ROS包可能未正确安装或配置"
echo "3. 环境变量可能有问题"
echo "4. 文件权限或路径可能不正确"

echo ""
echo "=== 检查当前状态 ==="
# 检查包状态
source install/setup.bash
echo "已注册的ELI包:"
ros2 pkg list | grep eli_ | nl

# 检查原始源文件
echo ""
echo "源文件检查:"
if [ -d "src" ]; then
    find src -name "*cs*" -type f | grep -E "(xacro|urdf|launch)" | head -10
fi
# 检查安装的文件
echo ""
echo "安装文件检查:"
if [ -d "install" ]; then
    find install -name "*cs*" -type f | grep -E "(xacro|urdf|launch)" | head -10
fi
# 恢复策略
echo ""
echo "🔄 恢复策略:"
echo "1. 重新编译工作空间"
echo "2. 使用简化的URDF直接启动"
echo "3. 检查并修复原始xacro文件"

# 重新编译
echo ""
echo "重新编译工作空间..."
colcon build
# 尝试恢复原始xacro文件
echo ""
echo "尝试恢复原始xacro文件..."
SRC_XACRO=$(find src -name "cs.urdf.xacro" | head -1)
if [ -n "$SRC_XACRO" ]; then
    echo "找到源xacro文件: $SRC_XACRO"
    # 创建备份
    cp "$SRC_XACRO" "${SRC_XACRO}.backup.$(date +%s)"
    
    # 尝试修复常见的xacro问题
    echo "修复xacro文件..."
    sed -i 's/initial_positions="${xacro.load_yaml(initial_positions_file)}"/initial_positions="{}"/g' "$SRC_XACRO"
    sed -i 's/xacro:eli_cs_robot/xacro:cs_robot/g' "$SRC_XACRO"
    
    echo "重新编译包..."
    colcon build --packages-select eli_cs_robot_description
else
    echo "未找到源xacro文件"
fi
echo ""
echo "🎯 建议下一步:"
echo "1. 运行: ./launch_cs625_direct.sh (使用简化URDF)"
echo "2. 或者尝试原始启动命令: ros2 launch eli_cs_robot_description view_cs.launch.py cs_type:=cs625"
