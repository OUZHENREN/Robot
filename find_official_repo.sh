#!/bin/bash
echo "🔍 查找ELI CS机器人官方仓库信息"

# 检查当前包的元数据
echo "=== 当前包的package.xml信息 ==="
find src -name "package.xml" -exec grep -l "eli_cs" {} \; | head -3 | while read file; do
    echo "文件: $file"
    grep -E "(url|description|license)" "$file" | head -5
    echo ""
done
# 检查是否有README或文档文件
echo "=== 文档文件检查 ==="
find src -name "README*" -o -name "README.md" -o -name "package.xml" | head -5 | while read file; do
    echo "文件: $file"
    grep -i "github\|repository\|url" "$file" | head -2
    echo ""
done
# 常见的ELI机器人仓库URL
echo "=== 可能的官方仓库URL ==="
echo "1. https://github.com/ELI-Robotics/eli_cs_robot_description"
echo "2. https://github.com/eli-robotics/eli_cs_robot"
echo "3. https://github.com/eli-robotics/eli_robots"
echo "4. https://github.com/industrial-robotics/eli_cs"

echo ""
echo "🎯 建议: 尝试访问上述仓库或联系包维护者获取原始文件"
