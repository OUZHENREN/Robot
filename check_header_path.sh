#!/bin/bash
# 检查系统中是否存在正确的头文件路径
find /opt/ros -name "*get_package_share_directory*" -type f 2>/dev/null
