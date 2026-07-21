#!/bin/bash
echo "正在恢复 eli_cs_robot_description 包..."
SRC_DIR="src"
PKG_DIR="$SRC_DIR/eli_cs_robot_description"

# 如果目录存在，先删除，确保是全新恢复
if [ -d "$PKG_DIR" ]; then
    echo "发现旧目录，正在删除: $PKG_DIR"
    rm -rf "$PKG_DIR"
fi

# 使用 svn 从 GitHub 子目录下载
echo "正在从GitHub下载官方文件..."
svn export https://github.com/Elite-Robots/Elite_Robots_CS_ROS2_Driver/trunk/eli_cs_robot_description "$PKG_DIR"

if [ $? -eq 0 ]; then
    echo "✅ 成功恢复包到: $PKG_DIR"
    echo "文件列表预览:"
    ls -l "$PKG_DIR"
else
    echo "❌ 下载失败！请检查网络连接或svn命令是否可用。"
fi
