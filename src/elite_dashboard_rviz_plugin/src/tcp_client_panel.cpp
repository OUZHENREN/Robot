#include "elite_dashboard_rviz_plugin/tcp_client_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/exceptions.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>

#include <cmath>
#include <chrono>
#include <sstream>

#include <QThread>
#include <QMetaObject>

using namespace std::chrono_literals;

namespace elite_dashboard_rviz_plugin
{

TcpClientPanel::TcpClientPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  initRosNode();
  initUI();
}

TcpClientPanel::~TcpClientPanel()
{
  running_ = false;
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

void TcpClientPanel::initRosNode()
{
  auto context = getDisplayContext();
  if (context) {
    auto ros_node_abstraction = context->getRosNodeAbstraction().lock();
    if (ros_node_abstraction) {
      node_ = ros_node_abstraction->get_raw_node();
    }
  }

  if (!node_) {
    node_ = rclcpp::Node::make_shared("tcp_client_panel_node");

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);

    running_ = true;
    spin_thread_ = std::thread([this]() {
      rclcpp::Rate rate(100.0);
      while (rclcpp::ok() && running_) {
        executor_->spin_some();
        rate.sleep();
      }
    });
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

void TcpClientPanel::initUI()
{
  auto * main_layout = new QVBoxLayout;

  // 原有 2x2 网格按钮
  auto * grid = new QGridLayout;
  t0_button_ = new QPushButton(QString::fromUtf8("插槽特征粗略采集"));
  t1_button_ = new QPushButton(QString::fromUtf8("插槽特征精确采集"));
  t2_button_ = new QPushButton(QString::fromUtf8("弹仓特征粗略采集"));
  t3_button_ = new QPushButton(QString::fromUtf8("弹仓特征精确采集"));

  grid->addWidget(t0_button_, 0, 0);
  grid->addWidget(t1_button_, 0, 1);
  grid->addWidget(t2_button_, 1, 0);
  grid->addWidget(t3_button_, 1, 1);

  // 新增固定功能按钮
  auto * fixed_action_layout = new QHBoxLayout;
  manual_sampling_button_ = new QPushButton(QString::fromUtf8("手动采样"));
  environment_recognition_button_ = new QPushButton(QString::fromUtf8("环境总体识别"));

  fixed_action_layout->addWidget(manual_sampling_button_);
  fixed_action_layout->addWidget(environment_recognition_button_);

  // 底部自定义字头 + 发送
  auto * bottom_layout = new QHBoxLayout;
  custom_header_edit_ = new QLineEdit;
  custom_header_edit_->setPlaceholderText(QString::fromUtf8("输入自定义字头"));
  custom_send_button_ = new QPushButton(QString::fromUtf8("发送"));

  bottom_layout->addWidget(custom_header_edit_);
  bottom_layout->addWidget(custom_send_button_);

  // 日志区域（简单）
  log_text_ = new QPlainTextEdit;
  log_text_->setReadOnly(true);

  main_layout->addLayout(grid);
  main_layout->addLayout(fixed_action_layout);
  main_layout->addLayout(bottom_layout);
  main_layout->addWidget(new QLabel(QString::fromUtf8("发送日志：")));
  main_layout->addWidget(log_text_);

  setLayout(main_layout);

  // 信号槽
  connect(t0_button_, &QPushButton::clicked,
          this, &TcpClientPanel::onT0Clicked);
  connect(t1_button_, &QPushButton::clicked,
          this, &TcpClientPanel::onT1Clicked);
  connect(t2_button_, &QPushButton::clicked,
          this, &TcpClientPanel::onT2Clicked);
  connect(t3_button_, &QPushButton::clicked,
          this, &TcpClientPanel::onT3Clicked);

  connect(manual_sampling_button_, &QPushButton::clicked,
          this, &TcpClientPanel::onManualSamplingClicked);
  connect(environment_recognition_button_, &QPushButton::clicked,
          this, &TcpClientPanel::onEnvironmentRecognitionClicked);

  connect(custom_send_button_, &QPushButton::clicked,
          this, &TcpClientPanel::onCustomSendClicked);
}

void TcpClientPanel::appendLog(const QString & text)
{
  if (!log_text_) return;
  if (QThread::currentThread() == this->thread()) {
    log_text_->appendPlainText(text);
  } else {
    QString copy = text;
    QMetaObject::invokeMethod(
      this,
      [this, copy]() {
        if (log_text_) {
          log_text_->appendPlainText(copy);
        }
      },
      Qt::QueuedConnection);
  }
}

bool TcpClientPanel::getCurrentTcpPose(double & x_m, double & y_m, double & z_m,
                                       double & rx_rad, double & ry_rad, double & rz_rad)
{
  if (!node_) {
    appendLog(QString::fromUtf8("节点未初始化，无法获取 TCP 位姿"));
    return false;
  }

  try {
    geometry_msgs::msg::TransformStamped tf =
      tf_buffer_->lookupTransform(
        base_frame_, tcp_frame_, tf2::TimePointZero,
        tf2::durationFromSec(1.0));

    x_m = tf.transform.translation.x;
    y_m = tf.transform.translation.y;
    z_m = tf.transform.translation.z;

    tf2::Quaternion q(
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z,
      tf.transform.rotation.w);

    tf2::Matrix3x3 m(q);
    m.getRPY(rx_rad, ry_rad, rz_rad);  // rad

    return true;
  } catch (const tf2::TransformException & ex) {
    appendLog(QString::fromUtf8("获取 TCP 位姿失败: ") + ex.what());
    return false;
  }
}

void TcpClientPanel::buildAndSend(const std::string & header)
{
  double x_m, y_m, z_m, rx_rad, ry_rad, rz_rad;
  if (!getCurrentTcpPose(x_m, y_m, z_m, rx_rad, ry_rad, rz_rad)) {
    return;
  }

  // 转 mm / deg
  double x_mm = x_m * 1000.0;
  double y_mm = y_m * 1000.0;
  double z_mm = z_m * 1000.0;
  double rx_deg = rx_rad * 180.0 / M_PI;
  double ry_deg = ry_rad * 180.0 / M_PI;
  double rz_deg = rz_rad * 180.0 / M_PI;

  std::ostringstream oss;
  oss.setf(std::ios::fixed, std::ios::floatfield);
  oss.precision(3);
  oss << header << ","
      << x_mm << ","
      << y_mm << ","
      << z_mm << ","
      << rx_deg << ","
      << ry_deg << ","
      << rz_deg << "\n";

  std::string msg = oss.str();
  sendTcpMessage(msg);

  appendLog(QString::fromUtf8("已发送: ") + QString::fromStdString(msg));
}

void TcpClientPanel::sendTcpMessage(const std::string & message)
{
  // 简单短连接实现（Linux 下）
  int sock = ::socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    appendLog(QString::fromUtf8("创建 socket 失败"));
    return;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(remote_port_));
  addr.sin_addr.s_addr = inet_addr(remote_ip_.c_str());

  if (::connect(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    appendLog(QString::fromUtf8("连接失败到 ") +
              QString::fromStdString(remote_ip_) + ":" +
              QString::number(remote_port_));
    ::close(sock);
    return;
  }

  ssize_t n = ::send(sock, message.c_str(), message.size(), 0);
  if (n < 0) {
    appendLog(QString::fromUtf8("发送失败"));
  }

  ::shutdown(sock, SHUT_RDWR);
  ::close(sock);
}

// 原有槽函数
void TcpClientPanel::onT0Clicked()
{
  buildAndSend("T0");
}

void TcpClientPanel::onT1Clicked()
{
  buildAndSend("T1");
}

void TcpClientPanel::onT2Clicked()
{
  buildAndSend("T2");
}

void TcpClientPanel::onT3Clicked()
{
  buildAndSend("T3");
}

// 新增固定功能按钮槽函数
void TcpClientPanel::onManualSamplingClicked()
{
  buildAndSend("E1");
}

void TcpClientPanel::onEnvironmentRecognitionClicked()
{
  buildAndSend("E0");
}

void TcpClientPanel::onCustomSendClicked()
{
  QString header_q = custom_header_edit_->text().trimmed();
  if (header_q.isEmpty()) {
    appendLog(QString::fromUtf8("自定义字头为空，未发送"));
    return;
  }
  if (header_q.contains(",") || header_q.contains(" ")) {
    appendLog(QString::fromUtf8("自定义字头包含非法字符（逗号/空格），未发送"));
    return;
  }
  buildAndSend(header_q.toStdString());
}

// RViz 配置保存/加载（目前无自定义项）
void TcpClientPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void TcpClientPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

}  // namespace elite_dashboard_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  elite_dashboard_rviz_plugin::TcpClientPanel,
  rviz_common::Panel)
