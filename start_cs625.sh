#!/bin/bash
# CS625机器人启动脚本 - 自动切换到正确目录

WORKSPACE_DIR="$HOME/elite_ros_ws"

# 检查工作空间是否存在
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "错误: 工作空间目录不存在: $WORKSPACE_DIR"
    exit 1
fi
# 创建一个在任何位置都能运行的包装脚本
cat > start_cs625.sh << 'EOF'
#!/bin/bash
# CS625机器人启动脚本 - 自动切换到正确目录

WORKSPACE_DIR="$HOME/elite_ros_ws"

# 检查工作空间是否存在
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "错误: 工作空间目录不存在: $WORKSPACE_DIR"
    exit 1
fi

# 切换到工作空间目录
cd "$WORKSPACE_DIR"
# 检查脚本是否存在
if [ ! -f "cs625_wizard_fixed.sh" ]; then
    echo "错误: 未找到启动向导脚本"
    echo "请在 $WORKSPACE_DIR 目录中创建脚本"
    exit 1
fi
# 运行向导
./cs625_wizard_fixed.sh
