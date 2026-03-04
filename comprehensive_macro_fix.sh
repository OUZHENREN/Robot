#!/bin/bash
echo "🔧 全面宏修复"

XACRO_FILE="install/eli_cs_robot_description/share/eli_cs_robot_description/urdf/cs.urdf.xacro"
MACRO_FILE="install/eli_cs_robot_description/share/eli_cs_robot_description/urdf/cs_macro.xacro"
# 备份
cp "$XACRO_FILE" "${XACRO_FILE}.backup2"

# 完全重写Xacro文件，确保与宏文件匹配
cat > "$XACRO_FILE" << 'XACRO'
<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="$(arg name)">
   <xacro:arg name="name" default="cs"/>
   <xacro:include filename="cs_macro.xacro"/>

   <xacro:arg name="cs_type" default="cs63"/>
   <xacro:arg name="tf_prefix" default="" />
   <xacro:arg name="joint_limit_params" default="$(find eli_cs_robot_description)/config/$(arg cs_type)/joint_limits.yaml"/>
   <xacro:arg name="kinematics_params" default="$(find eli_cs_robot_description)/config/$(arg cs_type)/default_kinematics.yaml"/>
   <xacro:arg name="physical_params" default="$(find eli_cs_robot_description)/config/$(arg cs_type)/physical_parameters.yaml"/>
   <xacro:arg name="visual_params" default="$(find eli_cs_robot_description)/config/$(arg cs_type)/visual_parameters.yaml"/>
   <xacro:arg name="transmission_hw_interface" default=""/>
   <xacro:arg name="safety_limits" default="false"/>
   <xacro:arg name="safety_pos_margin" default="0.15"/>

<!-- 使用正确的宏名 -->
   <xacro:cs_robot 
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
   />
</robot>
XACRO

echo "✅ 重写Xacro文件完成"
source install/setup.bash
if ros2 run xacro xacro "$XACRO_FILE" cs_type:=cs625 > /dev/null 2>&1; then
    echo "✅ 修复成功!"
    ros2 launch eli_cs_robot_description view_cs.launch.py cs_type:=cs625
else
    echo "❌ 修复失败"
    ./start_without_xacro.sh
fi
