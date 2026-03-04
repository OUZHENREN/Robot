#!/bin/bash
echo "🔧 快速修复Xacro问题..."

SRC_FILE="src/eli_cs_robot_description/urdf/cs.urdf.xacro"
BACKUP="${SRC_FILE}.backup.$(date +%s)"

# 备份原文件
cp "$SRC_FILE" "$BACKUP"
echo "✅ 备份创建: $BACKUP"
# 直接修复问题行 - 使用属性引用而非变量
sed -i '75s/initial_positions="${xacro.load_yaml(initial_positions_file)}"/initial_positions="${xacro.load_yaml(initial_positions_file)}"\/><!-- 临时注释：initial_positions="{}" -->/' "$SRC_FILE"

# 同时修复包路径查找问题 - 替换 $(find ...) 为相对路径
sed -i 's/\$(find eli_cs_robot_description)/\.\./g' "$SRC_FILE"
echo "✅ 修复完成，重新编译工作空间..."
colcon build --packages-select eli_cs_robot_description
source install/setup.bash
echo "🔄 测试修复结果..."
if ros2 run xacro xacro "$SRC_FILE" cs_type:=cs63 > /dev/null 2>&1; then
    echo "✅ Xacro语法修复成功！"
    echo "🚀 正在启动可视化界面..."
    timeout 10 ros2 launch eli_cs_robot_description view_cs.launch.py cs_type:=cs63
else
echo "❌ 修复失败，恢复备份..."
    mv "$BACKUP" "$SRC_FILE"
    echo "使用备选方案启动简单机器人..."
    rviz2 -d $(ros2 pkg prefix rviz2)/share/rviz2/default.rviz
fi
