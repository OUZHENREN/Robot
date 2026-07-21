#!/bin/bash
echo "🚀 直接启动CS625机器人系统（跳过验证）"

# 创建完整的URDF文件
URDF_FILE="/tmp/cs625_direct.urdf"
cat > "$URDF_FILE" << 'XML'
<?xml version="1.0"?>
<robot name="eli_cs625">
  <!-- ELI CS625 6轴工业机器人 -->
  <link name="base_link">
    <visual><geometry><box size="0.6 0.6 0.1"/></geometry><material name="gray"><color rgba="0.7 0.7 0.7 1"/></material></visual>
  </link>
  <link name="link1"><visual><geometry><cylinder length="0.5" radius="0.1"/></geometry><material name="red"/></visual></link>
  <link name="link2"><visual><geometry><cylinder length="0.4" radius="0.08"/></geometry><material name="blue"/></visual></link>
  <link name="link3"><visual><geometry><cylinder length="0.3" radius="0.06"/></geometry><material name="green"/></visual></link>
  <link name="link4"><visual><geometry><cylinder length="0.2" radius="0.05"/></geometry><material name="yellow"/></visual></link>
  <link name="link5"><visual><geometry><cylinder length="0.15" radius="0.04"/></geometry><material name="purple"/></visual></link>
  <link name="link6"><visual><geometry><cylinder length="0.1" radius="0.03"/></geometry><material name="orange"/></visual></link>
<!-- 6个关节，简化参数 -->
  <joint name="joint1" type="revolute"><parent link="base_link"/><child link="link1"/><origin xyz="0 0 0.1"/><axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14"/></joint>
  <joint name="joint2" type="revolute"><parent link="link1"/><child link="link2"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/><limit lower="-1.57" upper="1.57"/></joint>
  <joint name="joint3" type="revolute"><parent link="link2"/><child link="link3"/><origin xyz="0 0 0.4"/><axis xyz="0 1 0"/><limit lower="-1.57" upper="1.57"/></joint>
  <joint name="joint4" type="revolute"><parent link="link3"/><child link="link4"/><origin xyz="0 0 0.3"/><axis xyz="1 0 0"/><limit lower="-3.14" upper="3.14"/></joint>
  <joint name="joint5" type="revolute"><parent link="link4"/><child link="link5"/><origin xyz="0 0 0.2"/><axis xyz="0 1 0"/><limit lower="-1.57" upper="1.57"/></joint>
  <joint name="joint6" type="revolute"><parent link="link5"/><child link="link6"/><origin xyz="0 0 0.15"/><axis xyz="1 0 0"/><limit lower="-3.14" upper="3.14"/></joint>
</robot>
XML
echo "✅ URDF文件创建完成: $URDF_FILE"

# 设置环境
source install/setup.bash

echo "🔄 启动系统..."
# 方法1: 使用参数文件启动
echo "方法1: 使用robot_state_publisher参数"
ros2 run robot_state_publisher robot_state_publisher $URDF_FILE &
RSP_PID=$!

# 方法2: 使用launch文件启动（如果可用）
echo "启动RViz..."
rviz2 -d $(ros2 pkg prefix rviz2)/share/rviz2/default.rviz &
RVIZ_PID=$!

sleep 2
echo "启动关节控制器..."
ros2 run joint_state_publisher_gui joint_state_publisher_gui &
JSP_PID=$!

echo ""
echo "✅ 系统已启动!"
echo "===================="
echo "进程PID:"
echo "RViz: $RVIZ_PID"
echo "Robot State Publisher: $RSP_PID"
echo "Joint State Publisher: $JSP_PID"
echo ""
echo "📝 配置指南:"
echo "1. 在RViz中设置Fixed Frame为: base_link"
echo "2. 添加RobotModel显示"
echo "3. 使用Joint State Publisher窗口控制关节"
echo ""
echo "按Ctrl+C退出"

wait $RVIZ_PID
kill $RSP_PID $JSP_PID 2>/dev/null
