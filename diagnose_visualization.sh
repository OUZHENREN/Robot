#!/bin/bash
echo "🔍 可视化启动诊断"
echo "当前目录: $(pwd)"

# 检查是否在工作空间目录
if [ ! -d "install" ] || [ ! -f "install/setup.bash" ]; then
    echo "❌ 错误：请在ROS 2工作空间根目录运行此脚本"
    echo "当前目录应该包含 install/ 和 src/ 文件夹"
    exit 1
fi
# 设置环境
source install/setup.bash

echo "1. 检查包是否存在..."
if ros2 pkg list | grep -q eli_cs_robot_description; then
    echo "✅ 机器人描述包存在"
    PKG_PATH=$(ros2 pkg prefix eli_cs_robot_description)
    echo "   包路径: $PKG_PATH"
else
    echo "❌ 机器人描述包不存在"
    echo "   可用的eli相关包:"
    ros2 pkg list | grep eli_ | nl
    exit 1
fi
echo "2. 检查启动文件..."
LAUNCH_FILE="$PKG_PATH/share/eli_cs_robot_description/launch/view_cs.launch.py"
if [ -f "$LAUNCH_FILE" ]; then
    echo "✅ 启动文件存在: view_cs.launch.py"
    echo "   路径: $LAUNCH_FILE"
else
    echo "❌ 启动文件不存在"
    echo "   尝试查找其他启动文件..."
    find "$PKG_PATH" -name "*.launch.py" 2>/dev/null | nl
    exit 1
fi
echo "3. 检查URDF文件..."
URDF_FILE="$PKG_PATH/share/eli_cs_robot_description/urdf/cs.urdf.xacro"
if [ -f "$URDF_FILE" ]; then
    echo "✅ URDF文件存在"
else
    echo "❌ URDF文件不存在"
    echo "   尝试查找URDF文件..."
    find "$PKG_PATH" -name "*.urdf*" -o -name "*.xacro" 2>/dev/null | nl
    exit 1
fi
echo "4. 测试URDF生成..."
TEMP_URDF="/tmp/test_urdf.urdf"
echo "   使用命令: ros2 run xacro xacro '$URDF_FILE' cs_type:=cs63"
if ros2 run xacro xacro "$URDF_FILE" cs_type:=cs63 > "$TEMP_URDF" 2>/dev/null; then
    if [ -s "$TEMP_URDF" ]; then
        LINE_COUNT=$(wc -l < "$TEMP_URDF")
        echo "✅ URDF生成成功"
        echo "   文件包含 $LINE_COUNT 行"
        echo "   前5行内容:"
        head -5 "$TEMP_URDF"
    else
        echo "❌ URDF生成了空文件"
    fi
    rm -f "$TEMP_URDF"
else
    echo "❌ Xacro处理失败"
    echo "   错误信息:"
    ros2 run xacro xacro "$URDF_FILE" cs_type:=cs63 2>&1 | head -10
fi
echo "5. 检查RViz..."
if command -v rviz2 >/dev/null 2>&1; then
    echo "✅ RViz2已安装"
else
    echo "⚠️  RViz2未安装"
    echo "   安装命令: sudo apt install ros-${ROS_DISTRO}-rviz2"
fi
echo "6. 检查ROS 2环境..."
echo "   ROS_DISTRO: ${ROS_DISTRO:-未设置}"
echo "   当前工作空间: $(pwd)"
echo "   可执行文件:"
find install -name "view_cs*" -type f 2>/dev/null | nl

echo "🔧 诊断完成"
