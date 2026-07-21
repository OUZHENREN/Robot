#!/bin/bash
echo "🔧 修复Xacro语法错误"
echo "==================="

# 查找源文件（不是install目录中的）
SRC_FILE=$(find src -name "cs.urdf.xacro" 2>/dev/null | head -1)
INSTALL_FILE="/home/yff/elite_ros_ws/install/eli_cs_robot_description/share/eli_cs_robot_description/urdf/cs.urdf.xacro"

if [ -z "$SRC_FILE" ]; then
    echo "❌ 未找到源文件，检查install目录中的文件"
    SRC_FILE="$INSTALL_FILE"
fi
echo "处理文件: $SRC_FILE"

# 备份文件
BACKUP_FILE="${SRC_FILE}.backup.$(date +%s)"
cp "$SRC_FILE" "$BACKUP_FILE"
echo "✅ 已创建备份: $BACKUP_FILE"
# 检查问题行
echo ""
echo "问题行内容:"
sed -n '70,80p' "$SRC_FILE"
# 查找initial_positions_file的定义
echo ""
echo "查找initial_positions_file的定义:"
grep -n "initial_positions_file" "$SRC_FILE"
# 查找问题所在的行范围
echo ""
echo "分析问题区域 (70-80行):"
sed -n '70,80p' "$SRC_FILE" | cat -n
# 尝试自动修复（如果模式匹配）
if grep -q 'initial_positions="${xacro.load_yaml(initial_positions_file)}"' "$SRC_FILE"; then
    echo ""
    echo "尝试自动修复..."
# 创建修复版本
    TEMP_FILE="${SRC_FILE}.fixed"
    sed 's/initial_positions="${xacro.load_yaml(initial_positions_file)}"/initial_positions="${xacro.load_yaml(initial_positions_file)}" if initial_positions_file is defined?/' "$SRC_FILE" > "$TEMP_FILE"
# 检查修复后的语法
    if xacro "$TEMP_FILE" cs_type:=cs63 > /dev/null 2>&1; then
        echo "✅ 修复成功！"
        mv "$TEMP_FILE" "$SRC_FILE"
    else
echo "❌ 自动修复失败，需要手动修复"
        rm "$TEMP_FILE"
    fi
fi
# 尝试注释掉问题行
echo ""
echo "方案2: 注释掉问题行进行测试"
COMMENTED_FILE="${SRC_FILE}.commented"
sed '75s/^/<!-- /; 75s/$/ -->/' "$SRC_FILE" > "$COMMENTED_FILE"

if xacro "$COMMENTED_FILE" cs_type:=cs63 > /dev/null 2>&1; then
    echo "✅ 注释问题行后语法正确"
    echo "临时测试文件: $COMMENTED_FILE"
else
echo "❌ 注释后仍有其他问题"
    rm "$COMMENTED_FILE"
fi
