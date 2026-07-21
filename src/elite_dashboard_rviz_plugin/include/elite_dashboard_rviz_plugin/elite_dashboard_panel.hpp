#ifndef ELITE_DASHBOARD_RVIZ_PLUGIN_ELITE_DASHBOARD_PANEL_HPP
#define ELITE_DASHBOARD_RVIZ_PLUGIN_ELITE_DASHBOARD_PANEL_HPP

#include <memory>
#include <thread>
#include <atomic>

#include <QPushButton>
#include <QPlainTextEdit>
#include <QLabel>
#include <QSlider>
#include <QSpinBox>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <std_srvs/srv/trigger.hpp>

#include "eli_common_interface/srv/get_robot_mode.hpp"
#include "eli_common_interface/srv/get_safety_mode.hpp"
#include "eli_common_interface/srv/get_task_status.hpp"
#include "eli_common_interface/srv/set_io.hpp"

// 新增：速度滑块服务
#include "eli_common_interface/srv/set_speed_slider_fraction.hpp"

#include "eli_dashboard_interface/srv/load.hpp"
#include "eli_dashboard_interface/srv/log.hpp"

class QTimer;

namespace elite_dashboard_rviz_plugin
{

class EliteDashboardPanel : public rviz_common::Panel
{
  Q_OBJECT

private:
  QTimer * mode_refresh_timer_ {nullptr};

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

public:
  explicit EliteDashboardPanel(QWidget * parent = nullptr);
  ~EliteDashboardPanel() override;

  // ========== 新增：设置机器人速度百分比（0–100） ==========
  // 内部会通过 /io_and_status_controller/set_speed_slider 调用
  bool setSpeedScaling(int scaling_percent);

private Q_SLOTS:
  // 电源 & 抱闸 & 一键启动
  void onPowerOnClicked();
  void onPowerOffClicked();
  void onBrakeReleaseClicked();
  void onOneClickStartClicked();

  // 状态刷新
  void onRefreshStatusClicked();

  // 日志
  void onClearLogClicked();
  void onSendLogClicked();

  // 工具 IO
  void onT0OnClicked();
  void onT0OffClicked();
  void onT1OnClicked();
  void onT1OffClicked();
  void onInnerClicked();
  void onOuterClicked();
  
  void onRobotModeUpdated(QString mode_str, QString power_state_str);
  
  void onRtServerReconnectClicked();
  
signals:
  // 从 ROS 回调线程通知 GUI 线程
  void robotModeUpdated(QString mode_str, QString power_state_str);
  void logMessage(QString message);

private:
  // ========== ROS Node & 执行器 ==========
  rclcpp::Node::SharedPtr node_;

  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
  std::atomic_bool running_{false};

  // ========== ROS 服务 Clients ==========

  // Trigger 型服务
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr power_on_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr power_off_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr brake_release_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr get_task_path_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr resend_external_script_client_;

  // 自定义状态查询
  rclcpp::Client<eli_common_interface::srv::GetRobotMode>::SharedPtr get_robot_mode_client_;
  rclcpp::Client<eli_common_interface::srv::GetSafetyMode>::SharedPtr get_safety_mode_client_;
  rclcpp::Client<eli_common_interface::srv::GetTaskStatus>::SharedPtr get_task_status_client_;

  // 配置与任务加载 / 日志
  rclcpp::Client<eli_dashboard_interface::srv::Load>::SharedPtr load_configure_client_;
  rclcpp::Client<eli_dashboard_interface::srv::Load>::SharedPtr load_task_client_;
  rclcpp::Client<eli_dashboard_interface::srv::Log>::SharedPtr log_client_;

  // IO
  rclcpp::Client<eli_common_interface::srv::SetIO>::SharedPtr set_io_client_;

  // 新增：速度滑块服务 client
  // /io_and_status_controller/set_speed_slider : eli_common_interface/srv/SetSpeedSliderFraction
  rclcpp::Client<eli_common_interface::srv::SetSpeedSliderFraction>::SharedPtr speed_slider_client_;

  // ========== UI 控件 ==========

  // 电源 & 抱闸 & 一键启动
  QPushButton * power_on_btn_ {nullptr};
  QPushButton * power_off_btn_ {nullptr};
  QPushButton * brake_release_btn_ {nullptr};
  QPushButton * one_click_start_btn_ {nullptr};

  // 状态刷新
  QPushButton * refresh_status_btn_ {nullptr};

  // 状态显示
  QLabel * power_state_label_ {nullptr};
  QLabel * brake_state_label_ {nullptr};
  QLabel * robot_mode_label_ {nullptr};
  QLabel * safety_mode_label_ {nullptr};
  QLabel * task_status_label_ {nullptr};
  QLabel * task_path_label_ {nullptr};

  // 新增：RT SERVER 连接状态与重连按钮
  QLabel * rt_server_status_label_ {nullptr};
  QPushButton * rt_server_reconnect_btn_ {nullptr};

  // 日志区
  QPlainTextEdit * log_text_edit_ {nullptr};
  QPushButton * clear_log_btn_ {nullptr};
  QPushButton * send_log_btn_ {nullptr};

  // 工具 IO
  QPushButton * inner_btn_ {nullptr};
  QPushButton * outer_btn_ {nullptr};
  QPushButton * t0_on_btn_ {nullptr};
  QPushButton * t0_off_btn_ {nullptr};
  QPushButton * t1_on_btn_ {nullptr};
  QPushButton * t1_off_btn_ {nullptr};

  // 新增：速度控制 UI（可选）
  QSlider  * speed_slider_   {nullptr};  // 0–100
  QSpinBox * speed_spinbox_  {nullptr};  // 0–100

  // 一键启动流程正在执行标志
  bool one_click_running_ = false;

  // ========== 内部工具函数 ==========
  void initRosNode();
  void initServiceClients();
  void initUI();
  
  void initDefaultSpeed();

  void appendLog(const QString & text);
  void showErrorMessage(const QString & title, const QString & message);

  // Trigger 服务通用封装（简单同步调用）
  bool callTriggerService(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
    const std::string & service_name,
    std::string & out_message);

  // 刷新各类状态
  void refreshRobotMode();
  void refreshSafetyMode();
  void refreshTaskStatus();
  void refreshTaskPath();

  // 工具 IO
  void callSetIO(int8_t pin, bool on);
  
  // 新增：RT SERVER 重连逻辑封装
  void tryReconnectRtServer();

  // 将枚举转换为人类可读字符串
  QString robotModeToString(int8_t mode);
  QString safetyModeToString(int8_t mode);
  QString taskStatusToString(int8_t status);
};

}  // namespace elite_dashboard_rviz_plugin

#endif  // ELITE_DASHBOARD_RVIZ_PLUGIN_ELITE_DASHBOARD_PANEL_HPP
