#!/bin/bash
echo "修复所有set_value调用的警告..."

# 修复freedrive_controller.cpp中的set_value调用
SOURCE_FILE="src/eli_cs_controllers/src/freedrive_controller.cpp"
if [ -f "$SOURCE_FILE" ]; then
    sed -i 's/end_interface_->get().set_value(1.0);/(void)end_interface_->get().set_value(1.0);/g' "$SOURCE_FILE"
    sed -i 's/start_interface_->get().set_value(1.0);/(void)start_interface_->get().set_value(1.0);/g' "$SOURCE_FILE"
    sed -i 's/end_interface_->get().set_value(1.0);/(void)end_interface_->get().set_value(1.0);/g' "$SOURCE_FILE"  # 再次修复，确保所有都被修复
    echo "✓ 修复了freedrive_controller中的set_value调用"
fi
# 修复gpio_controller.cpp中的set_value调用
SOURCE_FILE="src/eli_cs_controllers/src/gpio_controller.cpp"
if [ -f "$SOURCE_FILE" ]; then
    sed -i 's/command_interfaces_\[(int)CommandOffset::TARGET_SPEED_FRACTION\].set_value(/(void)command_interfaces_[(int)CommandOffset::TARGET_SPEED_FRACTION].set_value(/g' "$SOURCE_FILE"
    echo "✓ 修复了gpio_controller中的set_value调用"
fi
# 修复所有其他set_value调用
find src/eli_cs_controllers -name "*.cpp" -exec sed -i 's/\([a-zA-Z_][a-zA-Z0-9_]*\)->get().set_value(\([^)]*\));/(void)\1->get().set_value(\2);/g' {} \;
find src/eli_cs_controllers -name "*.cpp" -exec sed -i 's/\([a-zA-Z_][a-zA-Z0-9_]*\)\.get().set_value(\([^)]*\));/(void)\1.get().set_value(\2);/g' {} \;
echo "✓ 所有set_value警告修复完成"
