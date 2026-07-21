#pragma once

#include <memory>
#include <thread>
#include <atomic>

#include <QWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QTimer>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace elite_dashboard_rviz_plugin
{

class TcpClientPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit TcpClientPanel(QWidget * parent = nullptr);
  ~TcpClientPanel() override;

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onT0Clicked();
  void onT1Clicked();
  void onT2Clicked();
  void onT3Clicked();

  void onManualSamplingClicked();
  void onEnvironmentRecognitionClicked();

  void onCustomSendClicked();

private:
  void initRosNode();
  void initUI();

  /// 从 TF 查询当前 TCP 位姿（base_link -> my_end_effector_link），单位 m + rad
  bool getCurrentTcpPose(double & x_m, double & y_m, double & z_m,
                         double & rx_rad, double & ry_rad, double & rz_rad);

  /// 发送一条 TCP 文本消息（短连接）
  void sendTcpMessage(const std::string & message);

  /// 实际执行：构造字符串并发送
  void buildAndSend(const std::string & header);

  /// 在面板日志区域追加文本
  void appendLog(const QString & text);

private:
  // ROS node & TF
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic_bool running_{false};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // TCP 参数
  std::string remote_ip_{"192.168.1.100"};
  int remote_port_{7000};
  std::string base_frame_{"base_link"};
  std::string tcp_frame_{"flange"};

  // UI 控件
  QPushButton * t0_button_{nullptr};
  QPushButton * t1_button_{nullptr};
  QPushButton * t2_button_{nullptr};
  QPushButton * t3_button_{nullptr};

  QPushButton * manual_sampling_button_{nullptr};
  QPushButton * environment_recognition_button_{nullptr};

  QLineEdit * custom_header_edit_{nullptr};
  QPushButton * custom_send_button_{nullptr};

  QPlainTextEdit * log_text_{nullptr};
};

}  // namespace elite_dashboard_rviz_plugin
