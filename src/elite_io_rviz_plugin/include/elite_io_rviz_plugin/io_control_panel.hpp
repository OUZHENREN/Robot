#ifndef ELITE_IO_RVIZ_PLUGIN_IO_CONTROL_PANEL_HPP
#define ELITE_IO_RVIZ_PLUGIN_IO_CONTROL_PANEL_HPP

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <QPushButton>

#include "eli_common_interface/srv/set_io.hpp"

namespace elite_io_rviz_plugin
{

class IOControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit IOControlPanel(QWidget * parent = nullptr);

  void load(const rviz_common::Config &config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onT0OnClicked();
  void onT0OffClicked();
  void onT1OnClicked();
  void onT1OffClicked();

  void onInnerClicked();   // 内缩：T0 ON + T1 ON
  void onOuterClicked();   // 外撑：T0 OFF + T1 OFF

private:
  void callSetIO(int8_t pin, bool on);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<eli_common_interface::srv::SetIO>::SharedPtr client_;

  QPushButton * inner_btn_;   // 左侧大按钮
  QPushButton * outer_btn_;   // 右侧大按钮

  QPushButton * t0_on_btn_;
  QPushButton * t0_off_btn_;
  QPushButton * t1_on_btn_;
  QPushButton * t1_off_btn_;
};

}  // namespace elite_io_rviz_plugin

#endif  // ELITE_IO_RVIZ_PLUGIN_IO_CONTROL_PANEL_HPP
