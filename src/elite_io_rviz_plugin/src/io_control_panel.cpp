#include "elite_io_rviz_plugin/io_control_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace elite_io_rviz_plugin
{

using eli_common_interface::srv::SetIO;

IOControlPanel::IOControlPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto context = getDisplayContext();
  if (context) {
    auto ros_node_abstraction = context->getRosNodeAbstraction().lock();
    if (ros_node_abstraction) {
      node_ = ros_node_abstraction->get_raw_node();
    }
  }

  if (!node_) {
    node_ = rclcpp::Node::make_shared("elite_io_rviz_plugin_node");
  }

  client_ = node_->create_client<SetIO>("/io_and_status_controller/set_io");

  // ---------------- GUI 布局 ----------------
  auto * main_layout = new QHBoxLayout;

  // 先创建所有按钮，后面统一设置大小
  inner_btn_ = new QPushButton(QString::fromUtf8("内缩"));
  outer_btn_ = new QPushButton(QString::fromUtf8("外撑"));

  t0_on_btn_  = new QPushButton("T0 ON");
  t0_off_btn_ = new QPushButton("T0 OFF");
  t1_on_btn_  = new QPushButton("T1 ON");
  t1_off_btn_ = new QPushButton("T1 OFF");

  // 中间按钮的高度作为参考
  int base_h = t0_on_btn_->sizeHint().height();
  // 让内缩/外撑看起来大约占两行
  inner_btn_->setMinimumHeight(base_h * 2 + 4);
  outer_btn_->setMinimumHeight(base_h * 2 + 4);

  // 左侧：一个“内缩”按钮，垂直居中，看起来跨两行
  auto * left_vlayout = new QVBoxLayout;
  left_vlayout->addStretch(1);
  left_vlayout->addWidget(inner_btn_);
  left_vlayout->addStretch(1);

  // 中间：两行 T0/T1 ON/OFF
  auto * center_vlayout = new QVBoxLayout;

  auto * t0_hlayout = new QHBoxLayout;
  t0_hlayout->addWidget(t0_on_btn_);
  t0_hlayout->addWidget(t0_off_btn_);

  auto * t1_hlayout = new QHBoxLayout;
  t1_hlayout->addWidget(t1_on_btn_);
  t1_hlayout->addWidget(t1_off_btn_);

  center_vlayout->addLayout(t0_hlayout);
  center_vlayout->addLayout(t1_hlayout);

  // 右侧：一个“外撑”按钮，垂直居中，看起来跨两行
  auto * right_vlayout = new QVBoxLayout;
  right_vlayout->addStretch(1);
  right_vlayout->addWidget(outer_btn_);
  right_vlayout->addStretch(1);

  // 合并三块区域
  main_layout->addLayout(left_vlayout);
  main_layout->addLayout(center_vlayout);
  main_layout->addLayout(right_vlayout);

  setLayout(main_layout);

  // ---------- 信号-槽连接 ----------
  // 单 IO 按钮
  connect(t0_on_btn_,  &QPushButton::clicked, this, &IOControlPanel::onT0OnClicked);
  connect(t0_off_btn_, &QPushButton::clicked, this, &IOControlPanel::onT0OffClicked);
  connect(t1_on_btn_,  &QPushButton::clicked, this, &IOControlPanel::onT1OnClicked);
  connect(t1_off_btn_, &QPushButton::clicked, this, &IOControlPanel::onT1OffClicked);

  // 宏按键
  connect(inner_btn_, &QPushButton::clicked, this, &IOControlPanel::onInnerClicked);
  connect(outer_btn_, &QPushButton::clicked, this, &IOControlPanel::onOuterClicked);
}

void IOControlPanel::callSetIO(int8_t pin, bool on)
{
  if (!client_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("elite_io_rviz_plugin"),
      "SetIO client not initialized.");
    return;
  }

  if (!client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(
      rclcpp::get_logger("elite_io_rviz_plugin"),
      "Service /io_and_status_controller/set_io not available.");
    return;
  }

  auto request = std::make_shared<SetIO::Request>();
  request->fun = 3;           // FUN_SET_TOOL_DIG_OUT
  request->pin = pin;         // 0 or 1
  request->analog_type = 0;   // 0 for digital
  request->state = on ? 1.0 : 0.0;

  client_->async_send_request(
    request,
    [pin, on](rclcpp::Client<SetIO>::SharedFuture future) {
      try {
        auto response = future.get();
        if (!response->success) {
          RCLCPP_WARN(
            rclcpp::get_logger("elite_io_rviz_plugin"),
            "SetIO failed for pin %d, state %s",
            pin, on ? "ON" : "OFF");
        } else {
          RCLCPP_INFO(
            rclcpp::get_logger("elite_io_rviz_plugin"),
            "SetIO success for pin %d, state %s",
            pin, on ? "ON" : "OFF");
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          rclcpp::get_logger("elite_io_rviz_plugin"),
          "Exception in SetIO response callback: %s", e.what());
      }
    });
}

// 四个单 IO 按钮槽函数
void IOControlPanel::onT0OnClicked()
{
  callSetIO(0, true);
}

void IOControlPanel::onT0OffClicked()
{
  callSetIO(0, false);
}

void IOControlPanel::onT1OnClicked()
{
  callSetIO(1, true);
}

void IOControlPanel::onT1OffClicked()
{
  callSetIO(1, false);
}

// 宏按键：内缩（T0 ON + T1 ON）
void IOControlPanel::onInnerClicked()
{
  callSetIO(0, true);
  callSetIO(1, true);
}

// 宏按键：外撑（T0 OFF + T1 OFF）
void IOControlPanel::onOuterClicked()
{
  callSetIO(0, false);
  callSetIO(1, false);
}

// RViz 配置保存/加载
void IOControlPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void IOControlPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

}  // namespace elite_io_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(elite_io_rviz_plugin::IOControlPanel, rviz_common::Panel)
