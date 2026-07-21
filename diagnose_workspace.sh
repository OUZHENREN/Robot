#!/bin/bash
echo "🔍 诊断工作空间状态..."

# 检查当前目录
echo "当前工作目录: $(pwd)"

# 检查关键目录是否存在
echo -e "\n📁 目录结构检查:"
for dir in src build install log; do
    if [ -d "$dir" ]; then
        echo "✅ $dir/ 存在"
        if [ "$dir" = "src" ]; then
            echo "   ELI相关包: $(find src -name "*eli*" -type d | grep -v "/\." | wc -l) 个"
        fi
    else
        echo "❌ $dir/ 不存在"
    fi
done
# 检查setup.bash
echo -e "\n⚙️ 环境设置检查:"
if [ -f "install/setup.bash" ]; then
    echo "✅ install/setup.bash 存在"
    echo "   文件大小: $(wc -l < install/setup.bash) 行"
    
    # 检查source后的包列表
    source install/setup.bash
    echo -e "\n📦 已注册的包:"
    ros2 pkg list | grep eli_ || echo "   未找到eli相关包"
else
    echo "❌ install/setup.bash 不存在"
    echo "   工作空间可能未编译成功"
fi
# 检查是否有编译日志错误
echo -e "\n📋 编译日志检查:"
if [ -d "log" ]; then
    latest_log=$(find log -name "*.log" -type f -printf "%T@ %p\n" | sort -n | tail -1 | cut -d' ' -f2-)
    if [ -n "$latest_log" ]; then
        echo "最新日志: $latest_log"
        echo "最后几行:"
        tail -5 "$latest_log" 2>/dev/null || echo "无法读取日志"
    fi
fi
# 检查src目录中的ELI包
echo -e "\n🔎 源文件检查:"
if [ -d "src" ]; then
    echo "ELI相关源文件:"
    find src -path "*/eli_cs_robot_description/*" -name "*.xacro" -o -name "*.launch.py" -o -name "package.xml" | head -10
else
    echo "❌ src/ 目录不存在"
fi
