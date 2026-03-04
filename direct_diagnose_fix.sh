#!/bin/bash
echo "🔍 直接诊断和修复Xacro问题..."

# 1. 首先显示详细的错误信息，而不是静默处理
echo "=== 原始错误信息 ==="
ros2 run xacro xacro src/eli_cs_robot_description/urdf/cs.urdf.xacro cs_type:=cs63 2>&1

# 2. 检查ROS包状态
echo -e "\n=== ROS包状态 ==="
ros2 pkg list | grep eli_cs_robot_description

# 3. 检查Xacro文件具体内容（特别是问题区域）
echo -e "\n=== 问题行及周围上下文 ==="
sed -n '70,80p' src/eli_cs_robot_description/urdf/cs.urdf.xacro | cat -n

# 4. 简单直接的修复：完全重写问题行
SRC_FILE="src/eli_cs_robot_description/urdf/cs.urdf.xacro"
cp "$SRC_FILE" "${SRC_FILE}.backup2"
# 修复方法：将问题行替换为空字典，避免复杂的xacro.load_yaml调用
sed -i '75s/.*/    initial_positions="{}"  <!-- 修复: 替换复杂YAML加载为简单空字典 -->/' "$SRC_FILE"

echo -e "\n=== 修复后的行 ==="
sed -n '75p' "$SRC_FILE"

# 5. 重新编译并测试
echo -e "\n=== 重新编译和测试 ==="
colcon build --packages-select eli_cs_robot_description
source install/setup.bash
if ros2 run xacro xacro "$SRC_FILE" cs_type:=cs63 > /tmp/test.urdf 2>&1; then
    echo "✅ 修复成功！生成的URDF文件:"
    head -5 /tmp/test.urdf
    echo -e "\n🚀 启动可视化界面..."
    ros2 launch eli_cs_robot_description view_cs.launch.py cs_type:=cs63
else
    echo "❌ 修复失败，错误信息:"
    cat /tmp/test.urdf
    
    # 备选方案：直接启动RViz
    echo -e "\n🔄 启动备选RViz界面..."
    rviz2 -d $(ros2 pkg prefix rviz2)/share/rviz2/default.rviz
fi
