#!/bin/bash
echo "🎯 终极CS625机器人修复方案"

# 设置环境
source install/setup.bash

# 查找Xacro和宏文件
XACRO_FILE="install/eli_cs_robot_description/share/eli_cs_robot_description/urdf/cs.urdf.xacro"
MACRO_FILE="install/eli_cs_robot_description/share/eli_cs_robot_description/urdf/cs_macro.xacro"

echo "分析宏文件参数..."
# 查看宏定义中的参数
if [ -f "$MACRO_FILE" ]; then
    echo "宏文件中的参数定义:"
    grep -A2 'xacro:macro name="cs_robot"' "$MACRO_FILE" | head -10
fi
# 创建一个正确的简化版Xacro文件
echo "创建正确的Xacro文件..."
cat > "/tmp/cs625_correct.xacro" << 'XACRO'
<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="cs625">
  <xacro:include filename="$(find eli_cs_robot_description)/urdf/cs_macro.xacro"/>
  
  <!-- 简化的参数，只传递必要的 -->
  <xacro:cs_robot 
    name="cs625"
    tf_prefix=""
    joint_limit_params=""
    kinematics_params=""
    physical_params=""
    visual_params=""
    transmission_hw_interface=""
    safety_limits="false"
    safety_pos_margin="0.15"
  />
</robot>
XACRO
# 测试这个简化版本
echo "测试简化版Xacro..."
if ros2 run xacro xacro "/tmp/cs625_correct.xacro" > /tmp/cs625_test.urdf 2>&1; then
    echo "✅ 简化版成功!"
    echo "使用简化版Xacro启动..."
    # 替换原文件
    cp "/tmp/cs625_correct.xacro" "$XACRO_FILE"
    ros2 launch eli_cs_robot_description view_cs.launch.py cs_type:=cs625
else
 echo "❌ 简化版失败，错误信息:"
    cat /tmp/cs625_test.urdf
    
    # 创建最小的URDF文件
    echo "创建最小的CS625 URDF..."
    cat > "/tmp/cs625_minimal.urdf" << 'XML'
<?xml version="1.0"?>
<robot name="eli_cs625">
  <!-- CS625 6轴工业机器人简化模型 -->
  <link name="base_link">
    <visual><geometry><cylinder length="0.3" radius="0.3"/></geometry><material name="steel"/></visual>
</link>
  <link name="link1"><visual><geometry><cylinder length="0.6" radius="0.15"/></geometry><material name="red"/></visual></link>
  <link name="link2"><visual><geometry><cylinder length="0.5" radius="0.12"/></geometry><material name="green"/></visual></link>
  <link name="link3"><visual><geometry><cylinder length="0.4" radius="0.1"/></geometry><material name="blue"/></visual></link>
  <link name="link4"><visual><geometry><cylinder length="0.3" radius="0.08"/></geometry><material name="yellow"/></visual></link>
  <link name="link5"><visual><geometry><cylinder length="0.2" radius="0.06"/></geometry><material name="purple"/></visual></link>
  <link name="link6"><visual><geometry><cylinder length="0.15" radius="0.04"/></geometry><material name="orange"/></visual></link>
<joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0.3"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14"/>
  </joint>
<joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0 0 0.6"/><axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0"/>
  </joint>
<joint name="joint3" type="revolute">
    <parent link="link2"/><child link="link3"/>
    <origin xyz="0 0 0.5"/><axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="2.5"/>
  </joint>
<joint name="joint4" type="revolute">
    <parent link="link3"/><child link="link4"/>
    <origin xyz="0 0 0.4"/><axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14"/>
  </joint>
<joint name="joint5" type="revolute">
    <parent link="link4"/><child link="link5"/>
    <origin xyz="0 0 0.3"/><axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0"/>
  </joint>
<joint name="joint6" type="revolute">
    <parent link="link5"/><child link="link6"/>
    <origin xyz="0 0 0.2"/><axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14"/>
  </joint>
</robot>
XML
echo "✅ 最小URDF创建完成: /tmp/cs625_minimal.urdf"
    echo "🚀 启动RViz可视化..."
    
    # 启动机器人状态发布器
    ros2 run robot_state_publisher robot_state_publisher /tmp/cs625_minimal.urdf &
    RSP_PID=$!
    
    # 启动RViz
    rviz2 -d $(ros2 pkg prefix rviz2)/share/rviz2/default.rviz &
    RVIZ_PID=$!
    
    echo "✅ 系统已启动"
    echo "RViz PID: $RVIZ_PID"
    echo "Robot State Publisher PID: $RSP_PID"
    echo ""
    echo "📝 在RViz中配置:"
    echo "1. 设置 Fixed Frame: base_link"
    echo "2. 添加 JointStatePublisher 显示来手动控制关节"
    echo ""
    echo "按 Ctrl+C 退出"
# 等待RViz进程
    wait $RVIZ_PID
    kill $RSP_PID 2>/dev/null
fi
