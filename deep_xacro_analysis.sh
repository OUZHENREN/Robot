#!/bin/bash
echo "🔍 深度分析Xacro语法问题"
echo "========================"

SRC_FILE="src/eli_cs_robot_description/urdf/cs.urdf.xacro"

echo "检查文件语法结构..."
echo "=== 查看包含的宏文件 ==="
grep -n "xacro:include" "$SRC_FILE"
# 查看宏文件内容
MACRO_FILE="src/eli_cs_robot_description/urdf/cs_macro.xacro"
echo ""
echo "=== 宏文件内容分析 ==="
if [ -f "$MACRO_FILE" ]; then
    echo "宏文件行数: $(wc -l < "$MACRO_FILE")"
    echo "查找initial_positions相关代码:"
    grep -n -A3 -B3 "initial_positions" "$MACRO_FILE" | head -30
else
    echo "❌ 未找到宏文件: $MACRO_FILE"
fi
echo ""
echo "=== 检查Xacro参数传递 ==="
# 查看参数定义和使用
echo "参数定义:"
grep -n "<xacro:arg" "$SRC_FILE"

echo ""
echo "参数使用模式:"
grep -n '\$(arg' "$SRC_FILE" | head -10
echo ""
echo "=== 问题区域的详细上下文 ==="
# 查看第75行周围的完整上下文
sed -n '40,85p' "$SRC_FILE" | cat -n

echo ""
echo "=== 尝试不同的Xacro处理方法 ==="
# 避免使用ros2 run，直接使用xacro命令
if command -v xacro >/dev/null 2>&1; then
    echo "使用系统xacro命令:"
    xacro --version 2>/dev/null || echo "未知版本"
    
    echo "测试1: 直接处理"
    xacro "$SRC_FILE" cs_type:=cs63 2>&1 | head -20
    
    echo ""
    echo "测试2: 显式传递所有参数"
    xacro "$SRC_FILE" \
name:=cs \
        cs_type:=cs63 \
        tf_prefix:="" \
        joint_limit_params:="config/cs63/joint_limits.yaml" \
        kinematics_params:="config/cs63/default_kinematics.yaml" \
        physical_params:="config/cs63/physical_parameters.yaml" \
        visual_params:="config/cs63/visual_parameters.yaml" \
        transmission_hw_interface:="" \
        safety_limits:=false \
        safety_pos_margin:=0.15 \
        initial_positions_file:="config/initial_positions.yaml" \
        2>&1 | head -20
else
echo "❌ xacro命令未找到"
fi
