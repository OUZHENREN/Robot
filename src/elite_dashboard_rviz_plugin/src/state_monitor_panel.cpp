#include "elite_dashboard_rviz_plugin/state_monitor_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QGridLayout>
#include <QThread>
#include <QMetaObject>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <chrono>

using namespace std::chrono_literals;

namespace elite_dashboard_rviz_plugin
{

StateMonitorPanel::StateMonitorPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  initUI();
}

StateMonitorPanel::~StateMonitorPanel()
{
}

void StateMonitorPanel::onInitialize()
{
  initRosNode();
  appendLog(QString::fromUtf8("StateMonitorPanel 已初始化"));
}

void StateMonitorPanel::initRosNode()
{
  auto context = getDisplayContext();
  if (context) {
    auto ros_node_abstraction = context->getRosNodeAbstraction().lock();
    if (ros_node_abstraction) {
      node_ = ros_node_abstraction->get_raw_node();
    }
  }

  if (!node_) {
    node_ = rclcpp::Node::make_shared("state_monitor_panel_node");
  }

  raw_state_sub_ =
    node_->create_subscription<cs625_state_monitor::msg::CS625State>(
      "/cs625/raw_state",
      10,
      std::bind(&StateMonitorPanel::rawStateCallback, this, std::placeholders::_1));

  tcp_force_sub_ =
    node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/cs625/tcp_force",
      10,
      std::bind(&StateMonitorPanel::tcpForceCallback, this, std::placeholders::_1));

  compliant_placement_state_sub_ =
    node_->create_subscription<cs625_compliant_placement::msg::CompliantPlacementState>(
      "/cs625/compliant_placement/state",
      10,
      std::bind(&StateMonitorPanel::compliantPlacementStateCallback, this, std::placeholders::_1));

  control_client_ = node_->create_client<cs625_state_monitor::srv::StateLoggingControl>(
    "/cs625/state_logging_control");

  appendLog(QString::fromUtf8(
    "已订阅 /cs625/raw_state、/cs625/tcp_force、/cs625/compliant_placement/state，"
    "服务 /cs625/state_logging_control 就绪（等待可用）"));
}

void StateMonitorPanel::initUI()
{
  auto * main_layout = new QVBoxLayout;

  // === 1. 控制按钮区域 ===
  auto * control_group = new QGroupBox(QString::fromUtf8("数据记录控制"));
  auto * control_layout = new QHBoxLayout;

  start_button_ = new QPushButton(QString::fromUtf8("开始记录"));
  pause_button_ = new QPushButton(QString::fromUtf8("暂停"));
  stop_button_  = new QPushButton(QString::fromUtf8("停止"));

  control_layout->addWidget(start_button_);
  control_layout->addWidget(pause_button_);
  control_layout->addWidget(stop_button_);
  control_group->setLayout(control_layout);
  main_layout->addWidget(control_group);

  // 状态显示
  auto * status_group = new QGroupBox(QString::fromUtf8("记录状态"));
  auto * status_layout = new QGridLayout;

  status_label_ = new QLabel("STOP");
  session_folder_label_ = new QLabel("N/A");

  status_layout->addWidget(new QLabel(QString::fromUtf8("当前状态：")), 0, 0);
  status_layout->addWidget(status_label_, 0, 1);

  status_layout->addWidget(new QLabel(QString::fromUtf8("当前 Session 目录：")), 1, 0);
  status_layout->addWidget(session_folder_label_, 1, 1);

  status_group->setLayout(status_layout);
  main_layout->addWidget(status_group);

  // === 2. 实时数据区域 ===
  auto * realtime_group = new QGroupBox(QString::fromUtf8("实时 TCP 数据（与 TaskExecutionPanel 同源）"));
  auto * realtime_layout = new QGridLayout;

  tcp_x_label_ = new QLabel("0.0");
  tcp_y_label_ = new QLabel("0.0");
  tcp_z_label_ = new QLabel("0.0");
  rot_x_label_ = new QLabel("0.0");
  rot_y_label_ = new QLabel("0.0");
  rot_z_label_ = new QLabel("0.0");

  tcp_force_x_label_ = new QLabel("0.0");
  tcp_force_y_label_ = new QLabel("0.0");
  tcp_force_z_label_ = new QLabel("0.0");
  tcp_torque_x_label_ = new QLabel("0.0");
  tcp_torque_y_label_ = new QLabel("0.0");
  tcp_torque_z_label_ = new QLabel("0.0");

  tau_metric_label_ = new QLabel("0.0");
  filtered_tau_metric_label_ = new QLabel("0.0");

  realtime_layout->addWidget(new QLabel("tcp_x_mm:"), 0, 0);
  realtime_layout->addWidget(tcp_x_label_,           0, 1);
  realtime_layout->addWidget(new QLabel("rot_x_deg:"), 0, 2);
  realtime_layout->addWidget(rot_x_label_,            0, 3);
  realtime_layout->addWidget(new QLabel("tcp_force_x:"), 0, 4);
  realtime_layout->addWidget(tcp_force_x_label_,         0, 5);
  realtime_layout->addWidget(new QLabel("tcp_torque_x:"), 0, 6);
  realtime_layout->addWidget(tcp_torque_x_label_,         0, 7);

  realtime_layout->addWidget(new QLabel("tcp_y_mm:"), 1, 0);
  realtime_layout->addWidget(tcp_y_label_,           1, 1);
  realtime_layout->addWidget(new QLabel("rot_y_deg:"), 1, 2);
  realtime_layout->addWidget(rot_y_label_,            1, 3);
  realtime_layout->addWidget(new QLabel("tcp_force_y:"), 1, 4);
  realtime_layout->addWidget(tcp_force_y_label_,         1, 5);
  realtime_layout->addWidget(new QLabel("tcp_torque_y:"), 1, 6);
  realtime_layout->addWidget(tcp_torque_y_label_,         1, 7);

  realtime_layout->addWidget(new QLabel("tcp_z_mm:"), 2, 0);
  realtime_layout->addWidget(tcp_z_label_,           2, 1);
  realtime_layout->addWidget(new QLabel("rot_z_deg:"), 2, 2);
  realtime_layout->addWidget(rot_z_label_,            2, 3);
  realtime_layout->addWidget(new QLabel("tcp_force_z:"), 2, 4);
  realtime_layout->addWidget(tcp_force_z_label_,         2, 5);
  realtime_layout->addWidget(new QLabel("tcp_torque_z:"), 2, 6);
  realtime_layout->addWidget(tcp_torque_z_label_,         2, 7);

  realtime_layout->addWidget(new QLabel("tau_metric:"), 3, 0);
  realtime_layout->addWidget(tau_metric_label_,       3, 1);
  realtime_layout->addWidget(new QLabel("filtered_tau_metric:"), 3, 2);
  realtime_layout->addWidget(filtered_tau_metric_label_, 3, 3);

  realtime_group->setLayout(realtime_layout);
  main_layout->addWidget(realtime_group);

  // === 3. 日志区域 ===
  auto * log_group = new QGroupBox(QString::fromUtf8("控制与状态日志"));
  auto * log_layout = new QVBoxLayout;

  log_text_ = new QPlainTextEdit;
  log_text_->setReadOnly(true);
  log_layout->addWidget(log_text_);
  log_group->setLayout(log_layout);
  main_layout->addWidget(log_group);

  setLayout(main_layout);

  connect(start_button_, &QPushButton::clicked, this, &StateMonitorPanel::onStartClicked);
  connect(pause_button_, &QPushButton::clicked, this, &StateMonitorPanel::onPauseClicked);
  connect(stop_button_,  &QPushButton::clicked, this, &StateMonitorPanel::onStopClicked);

  updateStatusLabel();
  updateTcpPoseDisplay();
  updateTcpWrenchDisplay();
  updateTauDisplay();
}

void StateMonitorPanel::appendLog(const QString & text)
{
  if (!log_text_) {
    return;
  }

  auto fn = [this, text]() {
    if (log_text_) {
      log_text_->appendPlainText(text);
    }
  };

  if (QThread::currentThread() == this->thread()) {
    fn();
  } else {
    QMetaObject::invokeMethod(this, fn, Qt::QueuedConnection);
  }
}

void StateMonitorPanel::updateStatusLabel()
{
  if (!status_label_) {
    return;
  }

  status_label_->setText(record_state_);

  if (record_state_ == "RECORDING") {
    status_label_->setStyleSheet("QLabel { color: green; font-weight: bold; }");
  } else if (record_state_ == "PAUSED") {
    status_label_->setStyleSheet("QLabel { color: orange; font-weight: bold; }");
  } else {
    status_label_->setStyleSheet("QLabel { color: red; font-weight: bold; }");
  }

  if (session_folder_label_) {
    session_folder_label_->setText(current_session_folder_);
  }
}

void StateMonitorPanel::updateTcpPoseDisplay()
{
  if (!tcp_x_label_) {
    return;
  }

  tcp_x_label_->setText(QString::number(tcp_x_mm_, 'f', 3));
  tcp_y_label_->setText(QString::number(tcp_y_mm_, 'f', 3));
  tcp_z_label_->setText(QString::number(tcp_z_mm_, 'f', 3));
  rot_x_label_->setText(QString::number(rot_x_deg_, 'f', 3));
  rot_y_label_->setText(QString::number(rot_y_deg_, 'f', 3));
  rot_z_label_->setText(QString::number(rot_z_deg_, 'f', 3));
}

void StateMonitorPanel::updateTcpWrenchDisplay()
{
  if (!tcp_force_x_label_) {
    return;
  }

  tcp_force_x_label_->setText(QString::number(tcp_force_x_, 'f', 6));
  tcp_force_y_label_->setText(QString::number(tcp_force_y_, 'f', 6));
  tcp_force_z_label_->setText(QString::number(tcp_force_z_, 'f', 6));
  tcp_torque_x_label_->setText(QString::number(tcp_torque_x_, 'f', 6));
  tcp_torque_y_label_->setText(QString::number(tcp_torque_y_, 'f', 6));
  tcp_torque_z_label_->setText(QString::number(tcp_torque_z_, 'f', 6));
}

void StateMonitorPanel::updateTauDisplay()
{
  if (!tau_metric_label_) {
    return;
  }

  tau_metric_label_->setText(QString::number(tau_metric_, 'f', 4));
  filtered_tau_metric_label_->setText(QString::number(filtered_tau_metric_, 'f', 4));
}

void StateMonitorPanel::rawStateCallback(
  const cs625_state_monitor::msg::CS625State::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  if (!msg->has_actual_tool_data) {
    return;
  }

  tcp_x_mm_ = msg->actual_tcp_x_mm;
  tcp_y_mm_ = msg->actual_tcp_y_mm;
  tcp_z_mm_ = msg->actual_tcp_z_mm;
  rot_x_deg_ = msg->actual_rot_x_deg;
  rot_y_deg_ = msg->actual_rot_y_deg;
  rot_z_deg_ = msg->actual_rot_z_deg;

  QMetaObject::invokeMethod(
    this,
    [this]() {
      updateTcpPoseDisplay();
    },
    Qt::QueuedConnection);
}

void StateMonitorPanel::tcpForceCallback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  if (msg->data.size() < 6) {
    return;
  }

  tcp_force_x_ = msg->data[0];
  tcp_force_y_ = msg->data[1];
  tcp_force_z_ = msg->data[2];
  tcp_torque_x_ = msg->data[3];
  tcp_torque_y_ = msg->data[4];
  tcp_torque_z_ = msg->data[5];

  QMetaObject::invokeMethod(
    this,
    [this]() {
      updateTcpWrenchDisplay();
    },
    Qt::QueuedConnection);
}

void StateMonitorPanel::compliantPlacementStateCallback(
  const cs625_compliant_placement::msg::CompliantPlacementState::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  tau_metric_ = msg->tau_metric;
  filtered_tau_metric_ = msg->filtered_tau_metric;

  QMetaObject::invokeMethod(
    this,
    [this]() {
      updateTauDisplay();
    },
    Qt::QueuedConnection);
}

void StateMonitorPanel::sendControlCommand(const std::string & command)
{
  if (!node_ || !control_client_) {
    appendLog(QString::fromUtf8("错误：ROS 节点或控制服务 client 尚未初始化"));
    return;
  }

  if (!control_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("错误：服务 /cs625/state_logging_control 不可用"));
    return;
  }

  auto request = std::make_shared<cs625_state_monitor::srv::StateLoggingControl::Request>();
  request->command = command;

  appendLog(QString::fromUtf8("发送控制命令：%1").arg(QString::fromStdString(command)));

  control_client_->async_send_request(
    request,
    [this, command](
      rclcpp::Client<cs625_state_monitor::srv::StateLoggingControl>::SharedFuture future)
    {
      try {
        auto resp = future.get();
        QString msg = QString::fromStdString(resp->message);
        if (!resp->success) {
          appendLog(
            QString::fromUtf8("控制命令失败：command=%1, message=%2")
            .arg(QString::fromStdString(command))
            .arg(msg));
        } else {
          appendLog(
            QString::fromUtf8("控制命令成功：command=%1, message=%2")
            .arg(QString::fromStdString(command))
            .arg(msg));

          if (command == "start") {
            record_state_ = "RECORDING";

            QString sessionFolder = QString::fromUtf8("由 state_csv_logger_node 决定");
            if (msg.startsWith(QStringLiteral("started:"))) {
              sessionFolder = msg.mid(QStringLiteral("started:").size()).trimmed();
            } else {
              QString key = QStringLiteral("at ");
              int idx = msg.lastIndexOf(key);
              if (idx != -1) {
                sessionFolder = msg.mid(idx + key.size()).trimmed();
              }
            }

            current_session_folder_ = sessionFolder;
          } else if (command == "pause") {
            record_state_ = "PAUSED";
          } else if (command == "stop") {
            record_state_ = "STOP";
            current_session_folder_ = "N/A";
          }

          updateStatusLabel();
        }
      } catch (const std::exception & e) {
        appendLog(QString::fromUtf8("控制命令回调异常：%1").arg(e.what()));
      }
    });
}

void StateMonitorPanel::onStartClicked()
{
  sendControlCommand("start");
}

void StateMonitorPanel::onPauseClicked()
{
  sendControlCommand("pause");
}

void StateMonitorPanel::onStopClicked()
{
  sendControlCommand("stop");
}

void StateMonitorPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void StateMonitorPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

}  // namespace elite_dashboard_rviz_plugin

PLUGINLIB_EXPORT_CLASS(
  elite_dashboard_rviz_plugin::StateMonitorPanel,
  rviz_common::Panel)
