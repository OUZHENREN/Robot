#!/bin/bash
echo "🎯 直接修复CS625机器人模型"

# 恢复原始Xacro文件（从备份或重新创建）
SRC_FILE="src/eli_cs_robot_description/urdf/cs.urdf.xacro"

# 如果备份存在，恢复它
if [ -f "${SRC_FILE}.backup.1768463902" ]; then
    echo "🔄 从备份恢复Xacro文件..."
    cp "${SRC_FILE}.backup.1768463902" "$SRC_FILE"
else
echo "📝 重新创建Xacro文件..."
    # 创建简化的Xacro文件，避免复杂的YAML加载
    cat > "$SRC_FILE" << 'XACRO'
<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="$(arg name)">
   <xacro:arg name="name" default="cs"/>
   <xacro:include filename="$(find eli_cs_robot_description)/urdf/cs_macro.xacro"/>
   
   <xacro:arg name="cs_type" default="cs63"/>
<xacro:arg name="tf_prefix" default=""/>
   
   <!-- 简化参数，避免复杂的YAML加载 -->
   <xacro:eli_cs_robot 
     name="$(arg name)"
     tf_prefix="$(arg tf_prefix)"
     cs_type="$(arg cs_type)"
     initial_positions="{}"
   />
</robot>
XACRO
fi
# 修复宏文件中的问题行
MACRO_FILE="src/eli_cs_robot_description/urdf/cs_macro.xacro"
if [ -f "$MACRO_FILE" ]; then
    echo "🔧 修复宏文件..."
    # 注释掉问题行，使用简单的空字典
    sed -i 's/initial_positions="${xacro.load_yaml(initial_positions_file)}"/<!-- initial_positions="${xacro.load_yaml(initial_positions_file)}" -->\n    initial_positions="{}"/' "$MACRO_FILE"
fi
# 重新编译
echo "🛠️ 重新编译工作空间..."
colcon build --packages-select eli_cs_robot_description
source install/setup.bash
# 测试
echo "🧪 测试CS625模型生成..."
if ros2 run xacro xacro "$SRC_FILE" cs_type:=cs625 > /dev/null 2>&1; then
    echo "✅ CS625模型生成成功！"
    echo "🚀 启动CS625机器人可视化..."
    ros2 launch eli_cs_robot_description view_cs.launch.py cs_type:=cs625
else
echo "❌ 模型生成失败，使用备选方案..."
    echo "🎯 启动RViz并手动加载模型..."
    rviz2 -d $(ros2 pkg prefix rviz2)/share/rviz2/default.rviz
fi
