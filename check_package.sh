#!/bin/bash
echo "🔍 检查ELI CS机器人包状态..."

# 检查包是否正常安装
ros2 pkg list | grep eli_cs_robot_description

# 检查包内容
echo -e "\n📁 包内容结构:"
pkg_path=$(ros2 pkg prefix eli_cs_robot_description 2>/dev/null || echo "未找到")
echo "包路径: $pkg_path"
if [ -d "$pkg_path" ]; then
    find "$pkg_path" -name "*.launch.py" -o -name "*.xacro" -o -name "*.urdf" | head -10
fi
# 检查是否有预编译的URDF
echo -e "\n🔎 寻找预编译的URDF文件:"
find install -name "*.urdf" ! -name "*.xacro" 2>/dev/null | head -5

# 检查启动文件参数
echo -e "\n⚙️ 启动文件参数:"
ros2 launch eli_cs_robot_description view_cs.launch.py --show-args 2>/dev/null || echo "无法显示启动参数"
# 最终建议
echo -e "\n💡 建议:"
echo "1. 如果包有问题，尝试重新安装或更新"
echo "2. 联系包维护者报告Xacro语法问题"
echo "3. 使用RViz手动加载URDF文件作为临时解决方案"
