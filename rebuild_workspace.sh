#!/bin/bash
echo "🛠️ 重新构建工作空间..."

# 清除之前的构建（可选）
echo "1. 清理构建缓存..."
rm -rf build install log 2>/dev/null || echo "无需清理"

# 重新构建
echo "2. 重新构建工作空间..."
colcon build

# 设置环境
echo "3. 设置环境..."
source install/setup.bash
# 验证构建结果
echo "4. 验证构建结果..."
if ros2 pkg list | grep -q eli_cs_robot_description; then
    echo "✅ eli_cs_robot_description 包已成功注册"
    pkg_path=$(ros2 pkg prefix eli_cs_robot_description)
    echo "   包路径: $pkg_path"
    
    echo -e "\n📁 包内容:"
    find "$pkg_path" -type f -name "*.launch.py" -o -name "*.xacro" -o -name "*.urdf" | head -10
else
    echo "❌ eli_cs_robot_description 包仍未注册"
    echo -e "\n📋 所有已注册的包:"
    ros2 pkg list | head -20
fi
