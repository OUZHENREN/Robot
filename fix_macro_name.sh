#!/bin/bash
echo "🔧 修复宏名称不一致问题"

# 查找文件
XACRO_FILE="install/eli_cs_robot_description/share/eli_cs_robot_description/urdf/cs.urdf.xacro"
MACRO_FILE="install/eli_cs_robot_description/share/eli_cs_robot_description/urdf/cs_macro.xacro"

echo "Xacro文件: $XACRO_FILE"
echo "宏文件: $MACRO_FILE"
# 确认宏名称
echo "宏文件中定义的宏名:"
grep -n 'xacro:macro name=' "$MACRO_FILE" | head -5

echo "Xacro文件中调用的宏名:"
grep -n 'xacro:eli_cs_robot' "$XACRO_FILE" | head -5

# 修复：将 eli_cs_robot 改为 cs_robot
echo "修复宏名称不匹配..."
cp "$XACRO_FILE" "${XACRO_FILE}.backup"
# 替换宏调用名称
sed -i 's/xacro:eli_cs_robot/xacro:cs_robot/g' "$XACRO_FILE"

echo "修复后的宏调用:"
grep -n 'xacro:cs_robot' "$XACRO_FILE" | head -5
# 测试修复
echo "测试修复结果..."
source install/setup.bash

if ros2 run xacro xacro "$XACRO_FILE" cs_type:=cs625 > /dev/null 2>&1; then
    echo "✅ 宏名称修复成功!"
    echo "🚀 启动CS625机器人可视化..."
    ros2 launch eli_cs_robot_description view_cs.launch.py cs_type:=cs625
else
    echo "❌ 修复失败，显示错误信息:"
    ros2 run xacro xacro "$XACRO_FILE" cs_type:=cs625 2>&1 | head -10
    
    # 尝试其他修复
    echo "🔄 尝试其他修复方法..."
    ./comprehensive_macro_fix.sh
fi
