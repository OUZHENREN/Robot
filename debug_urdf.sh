#!/bin/bash
echo "🔧 URDF/Xacro文件调试"
echo "======================"

source install/setup.bash

URDF_FILE="/home/yff/elite_ros_ws/install/eli_cs_robot_description/share/eli_cs_robot_description/urdf/cs.urdf.xacro"

echo "1. 检查Xacro文件内容..."
echo "文件大小: $(wc -l < "$URDF_FILE") 行"
# 检查可能的问题模式
echo ""
echo "2. 查找未定义变量模式..."
grep -n "\${" "$URDF_FILE" | head -20

echo ""
echo "3. 检查Xacro参数定义..."
grep -n "xacro:arg" "$URDF_FILE" | head -10
echo ""
echo "4. 尝试直接查看文件开头和结尾..."
echo "=== 文件前20行 ==="
head -20 "$URDF_FILE"
echo ""
echo "=== 文件后20行 ==="
tail -20 "$URDF_FILE"
echo ""
echo "5. 尝试使用不同方法处理Xacro..."
# 方法1: 直接使用xacro命令（显示详细错误）
echo "方法1: 直接xacro处理"
ros2 run xacro xacro "$URDF_FILE" cs_type:=cs63 2>&1 | head -30
echo ""
echo "方法2: 使用inorder处理（老版本兼容）"
xacro --inorder "$URDF_FILE" cs_type:=cs63 2>&1 | head -30
