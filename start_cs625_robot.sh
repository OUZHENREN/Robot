#!/bin/bash
echo "🚀 启动CS625工业机器人可视化"

# 设置环境
source install/setup.bash
echo "📦 确认包状态..."
if ros2 pkg list | grep -q eli_cs_robot_description; then
    echo "✅ eli_cs_robot_description 包已注册"
    pkg_path=$(ros2 pkg prefix eli_cs_robot_description)
    echo "   包路径: $pkg_path"
else
    echo "❌ 包未注册，退出"
    exit 1
fi
# 检查启动文件
LAUNCH_FILE="$pkg_path/share/eli_cs_robot_description/launch/view_cs.launch.py"
if [ -f "$LAUNCH_FILE" ]; then
    echo "✅ 启动文件存在: $LAUNCH_FILE"
else
    echo "❌ 启动文件不存在"
    exit 1
fi
# 检查URDF文件
URDF_FILE="$pkg_path/share/eli_cs_robot_description/urdf/cs.urdf.xacro"
if [ -f "$URDF_FILE" ]; then
    echo "✅ URDF文件存在: $URDF_FILE"
else
    echo "❌ URDF文件不存在"
    exit 1
fi
echo ""
echo "🎯 启动参数:"
echo "   模型: CS625 (6轴, 25kg负载工业机器人)"
echo "   启动文件: view_cs.launch.py"
echo "   URDF文件: cs.urdf.xacro"
# 尝试启动
echo ""
echo "🚀 启动CS625机器人可视化界面..."
echo "   如果出现错误，请查看控制台输出"
echo "   按 Ctrl+C 退出程序"
echo ""
# 直接启动，捕获可能的错误
ros2 launch eli_cs_robot_description view_cs.launch.py cs_type:=cs625
