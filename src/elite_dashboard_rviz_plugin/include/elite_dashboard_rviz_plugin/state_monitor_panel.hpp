#pragma once

#include <memory>
#include <string>

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QPlainTextEdit>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>

#include "cs625_state_monitor/srv/state_logging_control.hpp"
#include "cs625_state_monitor/msg/cs625_state.hpp"
#include "cs625_compliant_placement/msg/compliant_placement_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace elite_dashboard_rviz_plugin
{

class StateMonitorPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit StateMonitorPanel(QWidget * parent = nullptr);
  ~StateMonitorPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onStartClicked();
  void onPauseClicked();
  void onStopClicked();

private:
  void initRosNode();
  void initUI();
  void appendLog(const QString & text);

  void updateStatusLabel();
  void updateTcpPoseDisplay();
  void updateTcpWrenchDisplay();
  void updateTauDisplay();

  void rawStateCallback(
    const cs625_state_monitor::msg::CS625State::SharedPtr msg);

  void tcpForceCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  void compliantPlacementStateCallback(
    const cs625_compliant_placement::msg::CompliantPlacementState::SharedPtr msg);

  void sendControlCommand(const std::string & command);

private:
  // ROS
  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<cs625_state_monitor::msg::CS625State>::SharedPtr
    raw_state_sub_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    tcp_force_sub_;

  rclcpp::Subscription<cs625_compliant_placement::msg::CompliantPlacementState>::SharedPtr
    compliant_placement_state_sub_;

  rclcpp::Client<cs625_state_monitor::srv::StateLoggingControl>::SharedPtr control_client_;

  // UI 控件
  QPushButton * start_button_{nullptr};
  QPushButton * pause_button_{nullptr};
  QPushButton * stop_button_{nullptr};

  QLabel * status_label_{nullptr};
  QLabel * session_folder_label_{nullptr};

  QLabel * tcp_x_label_{nullptr};
  QLabel * tcp_y_label_{nullptr};
  QLabel * tcp_z_label_{nullptr};
  QLabel * rot_x_label_{nullptr};
  QLabel * rot_y_label_{nullptr};
  QLabel * rot_z_label_{nullptr};

  QLabel * tcp_force_x_label_{nullptr};
  QLabel * tcp_force_y_label_{nullptr};
  QLabel * tcp_force_z_label_{nullptr};
  QLabel * tcp_torque_x_label_{nullptr};
  QLabel * tcp_torque_y_label_{nullptr};
  QLabel * tcp_torque_z_label_{nullptr};

  QLabel * tau_metric_label_{nullptr};
  QLabel * filtered_tau_metric_label_{nullptr};

  QPlainTextEdit * log_text_{nullptr};

  // 本地状态记录（供 UI 显示用）
  QString record_state_{"STOP"};
  QString current_session_folder_{"N/A"};

  // TCP pose cache
  double tcp_x_mm_{0.0};
  double tcp_y_mm_{0.0};
  double tcp_z_mm_{0.0};
  double rot_x_deg_{0.0};
  double rot_y_deg_{0.0};
  double rot_z_deg_{0.0};

  // TCP wrench cache
  double tcp_force_x_{0.0};
  double tcp_force_y_{0.0};
  double tcp_force_z_{0.0};
  double tcp_torque_x_{0.0};
  double tcp_torque_y_{0.0};
  double tcp_torque_z_{0.0};

  // compliant placement metric cache
  double tau_metric_{0.0};
  double filtered_tau_metric_{0.0};
};

}  // namespace elite_dashboard_rviz_plugin
