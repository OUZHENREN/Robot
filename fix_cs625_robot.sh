#!/bin/bash
echo "🔧 修复CS625机器人XML语法错误..."

SRC_FILE="src/eli_cs_robot_description/urdf/cs.urdf.xacro"

# 恢复备份并正确修复
cp "${SRC_FILE}.backup.1768463902" "$SRC_FILE"

# 正确修复第75行：移除复杂的YAML加载，使用简单语法
sed -i '75s/initial_positions="${xacro.load_yaml(initial_positions_file)}"/initial_positions="{}"/' "$SRC_FILE"

echo "=== 修复后的行 ==="
sed -n '75p' "$SRC_FILE"
# 重新编译
echo "🔄 重新编译..."
colcon build --packages-select eli_cs_robot_description
source install/setup.bash
# 测试CS625型号
echo "🧪 测试CS625型号..."
if ros2 run xacro xacro "$SRC_FILE" cs_type:=cs625 > /tmp/cs625.urdf 2>&1; then
    echo "✅ CS625模型生成成功！"
    echo "🚀 启动CS625机器人可视化..."
    ros2 launch eli_cs_robot_description view_cs.launch.py cs_type:=cs625
else
    echo "❌ 模型生成失败，错误信息:"
    cat /tmp/cs625.urdf
    
    # 备选方案：手动创建简单CS625 URDF并启动
    echo "🔄 创建简单CS625模型..."
    cat > /tmp/cs625_simple.urdf << 'XML'
<?xml version="1.0"?>
<robot name="eli_cs625_robot">
  <link name="base_link">
    <visual>
      <geometry><cylinder length="0.1" radius="0.2"/></geometry>
      <material name="blue"><color rgba="0 0.5 0.8 1"/></material>
    </visual>
  </link>
 <!-- 6个关节的工业机器人 -->
  <link name="link1">
    <visual>
      <geometry><cylinder length="0.4" radius="0.08"/></geometry>
      <material name="red"><color rgba="0.8 0.1 0.1 1"/></material>
    </visual>
  </link>
  
  <link name="link2">
    <visual>
      <geometry><cylinder length="0.35" radius="0.06"/></geometry>
      <material name="green"><color rgba="0.1 0.8 0.1 1"/></material>
    </visual>
  </link>
<link name="link3">
    <visual>
      <geometry><cylinder length="0.3" radius="0.05"/></geometry>
      <material name="yellow"><color rgba="0.8 0.8 0.1 1"/></material>
    </visual>
  </link>
  
  <link name="link4">
    <visual>
      <geometry><cylinder length="0.25" radius="0.04"/></geometry>
      <material name="purple"><color rgba="0.6 0.1 0.8 1"/></material>
    </visual>
  </link>
<link name="link5">
    <visual>
      <geometry><cylinder length="0.2" radius="0.03"/></geometry>
      <material name="orange"><color rgba="0.9 0.5 0.1 1"/></material>
    </visual>
  </link>
<link name="link6">
    <visual>
      <geometry><cylinder length="0.15" radius="0.02"/></geometry>
      <material name="white"><color rgba="0.9 0.9 0.9 1"/></material>
    </visual>
  </link>
  
  <!-- 6个旋转关节 -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.2"/>
    <axis xyz="0 0 1"/>
  </joint>
<joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.4"/>
    <axis xyz="0 1 0"/>
  </joint>
  
  <joint name="joint3" type="revolute">
    <parent link="link2"/>
    <child link="link3"/>
    <origin xyz="0 0 0.35"/>
    <axis xyz="0 1 0"/>
  </joint>
<joint name="joint4" type="revolute">
    <parent link="link3"/>
    <child link="link4"/>
    <origin xyz="0 0 0.3"/>
    <axis xyz="1 0 0"/>
  </joint>
  
  <joint name="joint5" type="revolute">
    <parent link="link4"/>
    <child link="link5"/>
    <origin xyz="0 0 0.25"/>
    <axis xyz="0 1 0"/>
  </joint>
  
  <joint name="joint6" type="revolute">
    <parent link="link5"/>
    <child link="link6"/>
    <origin xyz="0 0 0.2"/>
    <axis xyz="1 0 0"/>
  </joint>
</robot>
XML
echo "✅ 简单CS625模型创建完成"
    echo "🎯 启动RViz加载CS625模型..."
    rviz2 -d $(ros2 pkg prefix rviz2)/share/rviz2/default.rviz &
    echo "在RViz中添加RobotModel，设置Fixed Frame为base_link"
    echo "设置Description File为: /tmp/cs625_simple.urdf"
    wait
fi
