#!/bin/bash
echo "🔧 绕过Xacro问题启动CS625机器人"

source install/setup.bash
# 创建简化URDF
URDF_FILE="/tmp/cs625_simple.urdf"
cat > "$URDF_FILE" << 'XML'
<?xml version="1.0"?>
<robot name="eli_cs625">
  <link name="base_link">
    <visual><geometry><cylinder length="0.3" radius="0.3"/></geometry><material name="blue"/></visual>
  </link>
  <link name="link1"><visual><geometry><cylinder length="0.6" radius="0.15"/></geometry><material name="red"/></visual></link>
  <link name="link2"><visual><geometry><cylinder length="0.5" radius="0.12"/></geometry><material name="green"/></visual></link>
  <link name="link3"><visual><geometry><cylinder length="0.4" radius="0.1"/></geometry><material name="yellow"/></visual></link>
  <link name="link4"><visual><geometry><cylinder length="0.3" radius="0.08"/></geometry><material name="purple"/></visual></link>
  <link name="link5"><visual><geometry><cylinder length="0.2" radius="0.06"/></geometry><material name="orange"/></visual></link>
  <link name="link6"><visual><geometry><cylinder length="0.15" radius="0.04"/></geometry><material name="white"/></visual></link>
<joint name="joint1" type="revolute"><parent link="base_link"/><child link="link1"/><origin xyz="0 0 0.3"/><axis xyz="0 0 1"/></joint>
  <joint name="joint2" type="revolute"><parent link="link1"/><child link="link2"/><origin xyz="0 0 0.6"/><axis xyz="0 1 0"/></joint>
  <joint name="joint3" type="revolute"><parent link="link2"/><child link="link3"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/></joint>
  <joint name="joint4" type="revolute"><parent link="link3"/><child link="link4"/><origin xyz="0 0 0.4"/><axis xyz="1 0 0"/></joint>
  <joint name="joint5" type="revolute"><parent link="link4"/><child link="link5"/><origin xyz="0 0 0.3"/><axis xyz="0 1 0"/></joint>
  <joint name="joint6" type="revolute"><parent link="link5"/><child link="link6"/><origin xyz="0 0 0.2"/><axis xyz="1 0 0"/></joint>
</robot>
XML
echo "✅ 创建简化URDF: $URDF_FILE"
echo "🚀 启动RViz..."
rviz2 -d $(ros2 pkg prefix rviz2)/share/rviz2/default.rviz &
echo "在RViz中添加RobotModel，设置Description File为: $URDF_FILE"
wait
