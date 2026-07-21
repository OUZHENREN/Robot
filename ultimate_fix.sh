#!/bin/bash
echo "🎯 终极修复：恢复CS625机器人模型显示"

# 1. 从git恢复原始Xacro文件（如果有）
if [ -d ".git" ]; then
    echo "🔄 从git恢复原始Xacro文件..."
    git checkout HEAD -- src/eli_cs_robot_description/urdf/cs.urdf.xacro 2>/dev/null || echo "❌ Git恢复失败，手动修复..."
fi
# 2. 如果git恢复失败，手动创建正确的Xacro文件
SRC_FILE="src/eli_cs_robot_description/urdf/cs.urdf.xacro"
if [ ! -f "$SRC_FILE" ] || grep -q "initial_positions="{}" "$SRC_FILE"; then
    echo "🧩 手动重建Xacro文件..."
    
    # 创建正确的Xacro文件内容
    cat > "$SRC_FILE" << 'XACRO'
<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="$(arg name)">
   <xacro:arg name="name" default="cs"/>
   <xacro:include filename="$(find eli_cs_robot_description)/urdf/cs_macro.xacro"/>
 <xacro:arg name="cs_type" default="cs63"/>
   <xacro:arg name="tf_prefix" default="" />
   <xacro:arg name="joint_limit_params" default="$(find eli_cs_robot_description)/config/$(arg cs_type)/joint_limits.yaml"/>
   <xacro:arg name="kinematics_params" default="$(find eli_cs_robot_description)/config/$(arg cs_type)/default_kinematics.yaml"/>
   <xacro:arg name="physical_params" default="$(find eli_cs_robot_description)/config/$(arg cs_type)/physical_parameters.yaml"/>
   <xacro:arg name="visual_params" default="$(find eli_cs_robot_description)/config/$(arg cs_type)/visual_parameters.yaml"/>
   <xacro:arg name="transmission_hw_interface" default=""/>
   <xacro:arg name="safety_limits" default="false"/>
   <xacro:arg name="safety_pos_margin" default="0.15"/>
   <xacro:arg name="initial_positions_file" default="$(find eli_cs_robot_description)/config/initial_positions.yaml"/>

   <!-- eli CS robot -->
<xacro:eli_cs_robot 
     name="$(arg name)"
     tf_prefix="$(arg tf_prefix)"
     cs_type="$(arg cs_type)"
     joint_limit_params="$(arg joint_limit_params)"
     kinematics_params="$(arg kinematics_params)"
     physical_params="$(arg physical_params)"
     visual_params="$(arg visual_params)"
     transmission_hw_interface="$(arg transmission_hw_interface)"
     safety_limits="$(arg safety_limits)"
     safety_pos_margin="$(arg safety_pos_margin)"
     initial_positions_file="$(arg initial_positions_file)"
   />
</robot>
XACRO
fi
# 3. 检查并修复宏文件中的问题行
MACRO_FILE="src/eli_cs_robot_description/urdf/cs_macro.xacro"
if [ -f "$MACRO_FILE" ]; then
    echo "🔍 检查宏文件..."
    # 查找并修复问题行
    if grep -q 'initial_positions="${xacro.load_yaml(initial_positions_file)}"' "$MACRO_FILE"; then
        echo "🔄 修复宏文件中的YAML加载问题..."
        sed -i 's/initial_positions="${xacro.load_yaml(initial_positions_file)}"/initial_positions="${xacro.load_yaml(initial_positions_file) if initial_positions_file else {}}"/' "$MACRO_FILE"
    fi
fi
# 4. 重新编译
echo "🛠️ 重新编译工作空间..."
colcon build --packages-select eli_cs_robot_description
source install/setup.bash

# 5. 直接测试CS625启动
echo "🚀 直接启动CS625机器人可视化..."
ros2 launch eli_cs_robot_description view_cs.launch.py cs_type:=cs625
