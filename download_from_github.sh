#!/bin/bash
echo "🌐 从GitHub下载原始ELI CS机器人文件"

# 创建临时目录
TEMP_DIR="/tmp/eli_cs_github"
mkdir -p "$TEMP_DIR"
cd "$TEMP_DIR"
# 尝试不同的仓库URL
REPO_URLS=(
    "https://github.com/ELI-Robotics/eli_cs_robot_description"
    "https://github.com/eli-robotics/eli_cs_robot_description" 
    "https://github.com/industrial-robotics/eli_cs_robot"
)
for repo in "${REPO_URLS[@]}"; do
    echo "尝试仓库: $repo"
    # 尝试下载特定文件
    if wget -q "$repo/raw/main/urdf/cs.urdf.xacro" || \
       wget -q "$repo/raw/master/urdf/cs.urdf.xacro"; then
        echo "✅ 成功下载cs.urdf.xacro"
        break
    else
 echo "❌ 下载失败: $repo"
    fi
done
# 如果直接下载失败，尝试git clone（精简版）
echo "尝试git clone..."
if git clone --depth=1 https://github.com/ELI-Robotics/eli_cs_robot_description.git 2>/dev/null || \
   git clone --depth=1 https://github.com/eli-robotics/eli_cs_robot_description.git 2>/dev/null; then
    echo "✅ Git clone 成功"
    # 查找关键文件
    find . -name "cs.urdf.xacro" -o -name "cs_macro.xacro" | head -5
else
echo "❌ Git clone 失败"
    
    # 备用方案：从其他源下载
    echo "尝试其他源..."
    wget -q "https://raw.githubusercontent.com/ros-industrial/robot_messages/master/urdf/robot.urdf.xacro" || \
    wget -q 
"https://raw.githubusercontent.com/ros/urdf_tutorial/master/urdf/01-myfirst.urdf"
fi
# 检查下载的文件
if [ -f "cs.urdf.xacro" ]; then
    echo "✅ 找到Xacro文件，检查内容..."
    head -20 cs.urdf.xacro
    echo "复制到工作空间..."
    cp cs.urdf.xacro ~/elite_ros_ws/src/eli_cs_robot_description/urdf/
else
    echo "❌ 未找到合适的文件"
fi
cd ~/elite_ros_ws
