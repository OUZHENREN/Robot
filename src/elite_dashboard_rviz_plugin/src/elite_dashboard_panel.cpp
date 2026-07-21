#include "elite_dashboard_rviz_plugin/elite_dashboard_panel.hpp"

#include <chrono>
#include <sstream>

#include <QThread>
#include <QPointer>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QMessageBox>
#include <QTimer>
#include <QMetaObject>
#include <QGridLayout>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <eli_dashboard_interface/srv/load.hpp>

using namespace std::chrono_literals;

namespace elite_dashboard_rviz_plugin
{

using std_srvs::srv::Trigger;
using eli_common_interface::srv::GetRobotMode;
using eli_common_interface::srv::GetSafetyMode;
using eli_common_interface::srv::GetTaskStatus;
using eli_common_interface::srv::SetIO;
using eli_dashboard_interface::srv::Load;
using eli_dashboard_interface::srv::Log;
using eli_common_interface::srv::SetSpeedSliderFraction;  // 速度服务

EliteDashboardPanel::EliteDashboardPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  initRosNode();
  initServiceClients();
  initUI();
  
  

  // 周期刷新 robot_mode
  mode_refresh_timer_ = new QTimer(this);
  mode_refresh_timer_->setInterval(1000);  // 每 1 秒刷新一次
  connect(mode_refresh_timer_, &QTimer::timeout,
          this, &EliteDashboardPanel::refreshRobotMode);
  mode_refresh_timer_->start();
}

EliteDashboardPanel::~EliteDashboardPanel()
{
  running_ = false;
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

void EliteDashboardPanel::initRosNode()
{
  // 1. 优先使用 RViz 提供的 node
  auto context = getDisplayContext();
  if (context) {
    auto ros_node_abstraction = context->getRosNodeAbstraction().lock();
    if (ros_node_abstraction) {
      node_ = ros_node_abstraction->get_raw_node();
    }
  }

  // 2. 如果 RViz node 不可用，则自己创建一个 node，并为它启动 executor + spin 线程
  if (!node_) {
    node_ = rclcpp::Node::make_shared("elite_dashboard_rviz_plugin_node");

    // 创建执行器并添加节点
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);

    // 启动 spin 线程
    running_ = true;
    spin_thread_ = std::thread([this]() {
      rclcpp::Rate rate(100);
      while (rclcpp::ok() && running_) {
        executor_->spin_some();
        rate.sleep();
      }
    });
  }
}

void EliteDashboardPanel::initServiceClients()
{
  // Trigger 型服务
  power_on_client_ =
    node_->create_client<Trigger>("/dashboard_client/power_on");
  power_off_client_ =
    node_->create_client<Trigger>("/dashboard_client/power_off");
  brake_release_client_ =
    node_->create_client<Trigger>("/dashboard_client/brake_release");
  get_task_path_client_ =
    node_->create_client<Trigger>("/dashboard_client/get_task_path");

  // 重发 ExternalControl 脚本
  resend_external_script_client_ =
    node_->create_client<Trigger>("/io_and_status_controller/resend_external_script");

  // 自定义状态服务
  get_robot_mode_client_ =
    node_->create_client<GetRobotMode>("/dashboard_client/robot_mode");
  get_safety_mode_client_ =
    node_->create_client<GetSafetyMode>("/dashboard_client/get_safety_mode");
  get_task_status_client_ =
    node_->create_client<GetTaskStatus>("/dashboard_client/get_task_status");

  // 配置与任务、日志
  load_configure_client_ =
    node_->create_client<Load>("/dashboard_client/load_configure");
  load_task_client_ =
    node_->create_client<Load>("/dashboard_client/load_task");

  log_client_ =
    node_->create_client<Log>("/dashboard_client/log");

  // IO
  set_io_client_ =
    node_->create_client<SetIO>("/io_and_status_controller/set_io");

  // 新增：速度滑块服务 client
  // /io_and_status_controller/set_speed_slider : eli_common_interface/srv/SetSpeedSliderFraction
  speed_slider_client_ =
    node_->create_client<SetSpeedSliderFraction>("/io_and_status_controller/set_speed_slider");
}

void EliteDashboardPanel::initUI()
{
  auto * main_layout = new QVBoxLayout;

  // ========== 1. 电源 & 抱闸 & 一键启动 ==========
  auto * power_group = new QGroupBox(QString::fromUtf8("电源与抱闸控制"));
  auto * power_layout = new QGridLayout;

  power_on_btn_        = new QPushButton(QString::fromUtf8("开启电源"));
  power_off_btn_       = new QPushButton(QString::fromUtf8("关闭电源"));
  brake_release_btn_   = new QPushButton(QString::fromUtf8("释放抱闸"));
  one_click_start_btn_ = new QPushButton(QString::fromUtf8("一键启动"));

  power_layout->addWidget(power_on_btn_,      0, 0);
  power_layout->addWidget(power_off_btn_,     0, 1);
  power_layout->addWidget(brake_release_btn_, 1, 0);
  power_layout->addWidget(one_click_start_btn_, 1, 1);

  power_group->setLayout(power_layout);
  main_layout->addWidget(power_group);

  // ========== 2. 工具 IO ==========
  auto * io_group = new QGroupBox(QString::fromUtf8("工具端 IO 控制"));
  auto * io_layout = new QVBoxLayout;

  inner_btn_ = new QPushButton(QString::fromUtf8("内缩"));
  outer_btn_ = new QPushButton(QString::fromUtf8("外撑"));

  t0_on_btn_  = new QPushButton("T0 ON");
  t0_off_btn_ = new QPushButton("T0 OFF");
  t1_on_btn_  = new QPushButton("T1 ON");
  t1_off_btn_ = new QPushButton("T1 OFF");

  // 快捷动作区
  auto * io_action_layout = new QHBoxLayout;
  io_action_layout->addWidget(inner_btn_);
  io_action_layout->addWidget(outer_btn_);

  // 通道控制区
  auto * io_channel_layout = new QGridLayout;
  io_channel_layout->addWidget(new QLabel(QString::fromUtf8("通道 T0：")), 0, 0);
  io_channel_layout->addWidget(t0_on_btn_, 0, 1);
  io_channel_layout->addWidget(t0_off_btn_, 0, 2);

  io_channel_layout->addWidget(new QLabel(QString::fromUtf8("通道 T1：")), 1, 0);
  io_channel_layout->addWidget(t1_on_btn_, 1, 1);
  io_channel_layout->addWidget(t1_off_btn_, 1, 2);

  io_layout->addLayout(io_action_layout);
  io_layout->addSpacing(6);
  io_layout->addLayout(io_channel_layout);

  io_group->setLayout(io_layout);
  main_layout->addWidget(io_group);
  
  // ========== 3. 速度控制 ==========
  auto * speed_group  = new QGroupBox(QString::fromUtf8("速度控制 (示教器速度比例)"));
  auto * speed_layout = new QHBoxLayout;

  speed_slider_ = new QSlider(Qt::Horizontal);
  speed_slider_->setRange(0, 100);
  speed_slider_->setValue(20);

  speed_spinbox_ = new QSpinBox;
  speed_spinbox_->setRange(0, 100);
  speed_spinbox_->setValue(20);
  speed_spinbox_->setSuffix("%");

  speed_layout->addWidget(new QLabel(QString::fromUtf8("速度比例：")));
  speed_layout->addWidget(speed_slider_);
  speed_layout->addWidget(speed_spinbox_);
  speed_group->setLayout(speed_layout);

  main_layout->addWidget(speed_group);

  // ========== 4. 状态控制：刷新 + 重连 RT SERVER ==========
  auto * status_ctrl_group = new QGroupBox(QString::fromUtf8("状态控制"));
  auto * status_ctrl_layout = new QVBoxLayout;

  refresh_status_btn_ = new QPushButton(QString::fromUtf8("刷新状态"));
  rt_server_reconnect_btn_ = new QPushButton(QString::fromUtf8("重连 RT SERVER"));

  auto * status_ctrl_btn_layout = new QHBoxLayout;
  status_ctrl_btn_layout->addWidget(refresh_status_btn_);
  status_ctrl_btn_layout->addWidget(rt_server_reconnect_btn_);
  status_ctrl_btn_layout->addStretch(1);

  auto * status_hint_label = new QLabel(
    QString::fromUtf8("说明：RT SERVER 重连会重新发送 ExternalControl 脚本。"));
  status_hint_label->setWordWrap(true);
  status_hint_label->setStyleSheet("color: gray;");

  status_ctrl_layout->addLayout(status_ctrl_btn_layout);
  status_ctrl_layout->addWidget(status_hint_label);

  status_ctrl_group->setLayout(status_ctrl_layout);
  main_layout->addWidget(status_ctrl_group);
  
  // ========== 5. 机器人状态显示 ==========
  auto * status_group  = new QGroupBox(QString::fromUtf8("机器人状态"));
  auto * status_layout = new QGridLayout;

  power_state_label_ = new QLabel(QString::fromUtf8("未知"));
  brake_state_label_ = new QLabel(QString::fromUtf8("未知"));
  robot_mode_label_  = new QLabel("N/A");
  safety_mode_label_ = new QLabel("N/A");
  task_status_label_ = new QLabel("N/A");
  task_path_label_   = new QLabel("N/A");

  // 可选：RT SERVER 状态显示（文字，不放按钮）
  rt_server_status_label_ = new QLabel(QString::fromUtf8("未知"));

  // 左列：电源 / 抱闸 / RobotMode / RT SERVER
  status_layout->addWidget(new QLabel(QString::fromUtf8("电源状态：")), 0, 0);
  status_layout->addWidget(power_state_label_,                        0, 1);

  status_layout->addWidget(new QLabel(QString::fromUtf8("抱闸状态：")), 1, 0);
  status_layout->addWidget(brake_state_label_,                        1, 1);

  status_layout->addWidget(new QLabel("RobotMode："),                 2, 0);
  status_layout->addWidget(robot_mode_label_,                         2, 1);

  status_layout->addWidget(new QLabel(QString::fromUtf8("RT SERVER：")), 3, 0);
  status_layout->addWidget(rt_server_status_label_,                      3, 1);

  // 右列：任务状态 / 任务文件 / SafetyMode
  status_layout->addWidget(new QLabel(QString::fromUtf8("任务状态：")), 0, 2);
  status_layout->addWidget(task_status_label_,                        0, 3);

  status_layout->addWidget(new QLabel(QString::fromUtf8("任务文件：")), 1, 2);
  status_layout->addWidget(task_path_label_,                          1, 3);

  status_layout->addWidget(new QLabel("SafetyMode："),                2, 2);
  status_layout->addWidget(safety_mode_label_,                        2, 3);

  status_group->setLayout(status_layout);
  main_layout->addWidget(status_group);

  // ========== 6. 日志窗口 ==========
  auto * log_group  = new QGroupBox(QString::fromUtf8("日志与信息"));
  auto * log_layout = new QVBoxLayout;

  log_text_edit_ = new QPlainTextEdit;
  log_text_edit_->setReadOnly(true);

  auto * log_btn_layout = new QHBoxLayout;
  clear_log_btn_        = new QPushButton(QString::fromUtf8("清空日志"));
  send_log_btn_         = new QPushButton(QString::fromUtf8("发送到机器人日志"));
  log_btn_layout->addWidget(clear_log_btn_);
  log_btn_layout->addWidget(send_log_btn_);

  log_layout->addWidget(log_text_edit_);
  log_layout->addLayout(log_btn_layout);
  log_group->setLayout(log_layout);

  main_layout->addWidget(log_group);

  setLayout(main_layout);

  // ========== 信号-槽 ==========
  // 电源与抱闸
  connect(power_on_btn_,        &QPushButton::clicked, this, &EliteDashboardPanel::onPowerOnClicked);
  connect(power_off_btn_,       &QPushButton::clicked, this, &EliteDashboardPanel::onPowerOffClicked);
  connect(brake_release_btn_,   &QPushButton::clicked, this, &EliteDashboardPanel::onBrakeReleaseClicked);
  connect(one_click_start_btn_, &QPushButton::clicked, this, &EliteDashboardPanel::onOneClickStartClicked);

  // 状态控制
  connect(refresh_status_btn_,      &QPushButton::clicked, this, &EliteDashboardPanel::onRefreshStatusClicked);
  connect(rt_server_reconnect_btn_, &QPushButton::clicked, this, &EliteDashboardPanel::onRtServerReconnectClicked);

  // 日志按钮
  connect(clear_log_btn_, &QPushButton::clicked, this, &EliteDashboardPanel::onClearLogClicked);
  connect(send_log_btn_,  &QPushButton::clicked, this, &EliteDashboardPanel::onSendLogClicked);

  // 工具 IO
  connect(t0_on_btn_,   &QPushButton::clicked, this, &EliteDashboardPanel::onT0OnClicked);
  connect(t0_off_btn_,  &QPushButton::clicked, this, &EliteDashboardPanel::onT0OffClicked);
  connect(t1_on_btn_,   &QPushButton::clicked, this, &EliteDashboardPanel::onT1OnClicked);
  connect(t1_off_btn_,  &QPushButton::clicked, this, &EliteDashboardPanel::onT1OffClicked);
  connect(inner_btn_,   &QPushButton::clicked, this, &EliteDashboardPanel::onInnerClicked);
  connect(outer_btn_,   &QPushButton::clicked, this, &EliteDashboardPanel::onOuterClicked);

  // 速度 slider & spinbox 联动
  connect(speed_slider_, &QSlider::valueChanged,
          this, [this](int value) {
            if (speed_spinbox_->value() != value) {
              speed_spinbox_->setValue(value);
            }
            setSpeedScaling(value);
          });

  connect(speed_spinbox_, QOverload<int>::of(&QSpinBox::valueChanged),
          this, [this](int value) {
            if (speed_slider_->value() != value) {
              speed_slider_->setValue(value);
            }
            setSpeedScaling(value);
          });

  // 跨线程更新 UI 和日志
  connect(this, &EliteDashboardPanel::robotModeUpdated,
          this, &EliteDashboardPanel::onRobotModeUpdated);

  connect(this, &EliteDashboardPanel::logMessage,
          this, &EliteDashboardPanel::appendLog);
}


// ========== 日志 & 错误 ==========
// 1) 日志追加：只允许在 GUI 线程真正操作控件
void EliteDashboardPanel::appendLog(const QString &text)
{
  if (QThread::currentThread() == this->thread()) {
    if (log_text_edit_) {
      log_text_edit_->appendPlainText(text);
    }
    return;
  }

  QString copy = text;
  QMetaObject::invokeMethod(
    this,
    [this, copy]() {
      if (log_text_edit_) {
        log_text_edit_->appendPlainText(copy);
      }
    },
    Qt::QueuedConnection);
}


// 2) 错误提示：内部也用 appendLog，通过 GUI 线程执行
void EliteDashboardPanel::showErrorMessage(const QString & title,
                                           const QString & message)
{
  // 不直接操作 message box，统一在日志窗口中显示
  appendLog(title + QString::fromUtf8("：") + message);
}

// 3) 设置速度比例：所有 UI 更新通过 appendLog / invokeMethod
// ========== 速度设置实现（简单安全版） ==========
bool EliteDashboardPanel::setSpeedScaling(int scaling_percent)
{
  // 基本检查
  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("EliteDashboardPanel"),
                 "setSpeedScaling: Node not initialized");
    showErrorMessage(QString::fromUtf8("设置速度失败"),
                     QString::fromUtf8("节点未初始化"));
    return false;
  }

  if (scaling_percent < 0 || scaling_percent > 100) {
    RCLCPP_WARN(node_->get_logger(),
                "setSpeedScaling: value %d out of range [0,100]", scaling_percent);
    showErrorMessage(QString::fromUtf8("设置速度失败"),
                     QString::fromUtf8("数值超出范围 0–100"));
    return false;
  }

  if (!speed_slider_client_) {
    RCLCPP_ERROR(node_->get_logger(),
                 "setSpeedScaling: speed_slider_client_ not created");
    showErrorMessage(QString::fromUtf8("设置速度失败"),
                     QString::fromUtf8("速度服务 client 未初始化"));
    return false;
  }

  if (!speed_slider_client_->wait_for_service(1s)) {
    RCLCPP_WARN(node_->get_logger(),
                "setSpeedScaling: service /io_and_status_controller/set_speed_slider not available");
    showErrorMessage(QString::fromUtf8("设置速度失败"),
                     QString::fromUtf8("服务 /io_and_status_controller/set_speed_slider 不可用"));
    return false;
  }

  auto request = std::make_shared<SetSpeedSliderFraction::Request>();
  request->speed_slider_fraction =
    static_cast<double>(scaling_percent) / 100.0;

  RCLCPP_DEBUG(node_->get_logger(),
              "Setting speed slider to %.2f (percent=%d)",
              request->speed_slider_fraction, scaling_percent);

  // 用 appendLog 记录，内部已保证在 GUI 线程更新
  appendLog(QString::fromUtf8("设置速度比例为 %1%").arg(scaling_percent));

  // 直接捕获 this，并在回调内检查 node_ 是否还有效
  speed_slider_client_->async_send_request(
    request,
    [this, scaling_percent](rclcpp::Client<SetSpeedSliderFraction>::SharedFuture future)
    {
      try {
        auto resp = future.get();
        if (!node_) {
          // node_ 已无效，不再操作
          return;
        }

        if (!resp->success) {
          RCLCPP_ERROR(node_->get_logger(),
                       "Speed slider service reported failure");
          showErrorMessage(
            QString::fromUtf8("设置速度失败"),
            QString::fromUtf8("服务返回失败（%1%）").arg(scaling_percent));
        } else {
          RCLCPP_DEBUG(node_->get_logger(),
                      "Speed slider set successfully to %d%%",
                      scaling_percent);
          appendLog(
            QString::fromUtf8("设置速度成功：%1%").arg(scaling_percent));
        }
      } catch (const std::exception & e) {
        if (!node_) {
          return;
        }
        RCLCPP_ERROR(node_->get_logger(),
                     "Speed slider service callback exception: %s", e.what());
        showErrorMessage(
          QString::fromUtf8("设置速度失败"),
          QString::fromUtf8("回调异常：") + e.what());
      }
    });

  return true;
}

void EliteDashboardPanel::initDefaultSpeed()
{
  const int default_speed_percent = 20;
  const double default_fraction =
    static_cast<double>(default_speed_percent) / 100.0;

  // 基本检查：node_
  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("EliteDashboardPanel"),
                 "initDefaultSpeed: Node not initialized");
    appendLog(QString::fromUtf8("初始化速度失败：节点未初始化"));
    return;
  }

  // 先更新 UI 控件的显示（不触发信号）
  if (speed_slider_) {
    speed_slider_->blockSignals(true);
    speed_slider_->setValue(default_speed_percent);
    speed_slider_->blockSignals(false);
  }

  if (speed_spinbox_) {
    speed_spinbox_->blockSignals(true);
    speed_spinbox_->setValue(default_speed_percent);
    speed_spinbox_->blockSignals(false);
  }

  // 检查 client
  if (!speed_slider_client_) {
    RCLCPP_ERROR(node_->get_logger(),
                 "initDefaultSpeed: speed_slider_client_ not created");
    appendLog(QString::fromUtf8("初始化速度失败：速度服务 client 未初始化"));
    return;
  }

  using namespace std::chrono_literals;
  if (!speed_slider_client_->wait_for_service(1s)) {
    RCLCPP_WARN(node_->get_logger(),
                "initDefaultSpeed: service /io_and_status_controller/set_speed_slider not available");
    appendLog(QString::fromUtf8("初始化速度失败：服务 /io_and_status_controller/set_speed_slider 不可用"));
    return;
  }

  auto request = std::make_shared<SetSpeedSliderFraction::Request>();
  request->speed_slider_fraction = default_fraction;

  RCLCPP_INFO(node_->get_logger(),
              "Initializing speed slider to %.2f (percent=%d)",
              request->speed_slider_fraction, default_speed_percent);

  appendLog(QString::fromUtf8("初始化速度比例为 %1%").arg(default_speed_percent));

  // 和 setSpeedScaling 保持一致：异步发送 + 回调里处理 success
  speed_slider_client_->async_send_request(
    request,
    [this, default_speed_percent](rclcpp::Client<SetSpeedSliderFraction>::SharedFuture future)
    {
      try {
        auto resp = future.get();
        if (!node_) {
          // node_ 已无效，不再操作
          return;
        }

        if (!resp->success) {
          RCLCPP_ERROR(node_->get_logger(),
                       "initDefaultSpeed: speed slider service reported failure");
          appendLog(
            QString::fromUtf8("初始化速度失败：服务返回失败（%1%）").arg(default_speed_percent));
        } else {
          RCLCPP_INFO(node_->get_logger(),
                      "initDefaultSpeed: speed slider initialized to %d%%",
                      default_speed_percent);
          appendLog(
            QString::fromUtf8("已将机器人速度初始化为 %1%").arg(default_speed_percent));
        }
      } catch (const std::exception & e) {
        if (!node_) {
          return;
        }
        RCLCPP_ERROR(node_->get_logger(),
                     "initDefaultSpeed: speed slider service callback exception: %s", e.what());
        appendLog(
          QString::fromUtf8("初始化速度失败：回调异常：") + e.what());
      }
    });
}

// ========== RT SERVER 重连相关 ==========

void EliteDashboardPanel::onRtServerReconnectClicked()
{
  appendLog(QString::fromUtf8("尝试重连 RT SERVER（重新发送 ExternalControl 脚本）..."));
  tryReconnectRtServer();
}

void EliteDashboardPanel::tryReconnectRtServer()
{
  if (!node_) {
    showErrorMessage(QString::fromUtf8("RT SERVER 重连失败"),
                     QString::fromUtf8("节点未初始化"));
    return;
  }

  if (!resend_external_script_client_) {
    showErrorMessage(QString::fromUtf8("RT SERVER 重连失败"),
                     QString::fromUtf8("resend_external_script client 未初始化"));
    return;
  }

  if (!resend_external_script_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("RT SERVER 重连失败：服务 /io_and_status_controller/resend_external_script 不可用"));
    showErrorMessage(QString::fromUtf8("RT SERVER 重连失败"),
                     QString::fromUtf8("服务 /io_and_status_controller/resend_external_script 不可用"));
    return;
  }

  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();

  // 避免重复点击
  if (rt_server_reconnect_btn_) {
    rt_server_reconnect_btn_->setEnabled(false);
  }

  resend_external_script_client_->async_send_request(
    req,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
    {
      try {
        auto resp = future.get();
        QString detail = QString::fromStdString(resp->message);

        if (resp->success) {
          appendLog(QString::fromUtf8("RT SERVER 重连指令发送成功：%1").arg(detail));

          // 这里我们只能确信“脚本已重新下发”，实际连接建立要看控制器端
          QMetaObject::invokeMethod(
            this,
            [this, detail]() {
              if (rt_server_status_label_) {
                rt_server_status_label_->setText(
                  QString::fromUtf8("已发送重连指令（请稍后检查连接）"));
              }
            },
            Qt::QueuedConnection);
        } else {
          appendLog(QString::fromUtf8("RT SERVER 重连失败：resend_external_script 返回失败（%1）").arg(detail));
          showErrorMessage(QString::fromUtf8("RT SERVER 重连失败"),
                           QString::fromUtf8("resend_external_script 返回失败：") + detail);
        }
      } catch (const std::exception & e) {
        QString err = QString::fromUtf8("调用 resend_external_script 异常：") +
                      QString::fromUtf8(e.what());
        appendLog(err);
        showErrorMessage(QString::fromUtf8("RT SERVER 重连失败"), err);
      }

      // 恢复按钮可用
      QMetaObject::invokeMethod(
        this,
        [this]() {
          if (rt_server_reconnect_btn_) {
            rt_server_reconnect_btn_->setEnabled(true);
          }
        },
        Qt::QueuedConnection);
    });
}

bool EliteDashboardPanel::callTriggerService(
  const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
  const std::string & service_name,
  std::string & out_message)
{
  out_message.clear();

  if (!node_) {
    out_message = "Node not initialized";
    RCLCPP_ERROR(
      rclcpp::get_logger("elite_dashboard_rviz_plugin"),
      "callTriggerService(%s): node_ is null", service_name.c_str());
    return false;
  }

  if (!client) {
    out_message = "Client not initialized";
    RCLCPP_ERROR(
      rclcpp::get_logger("elite_dashboard_rviz_plugin"),
      "callTriggerService(%s): client is null", service_name.c_str());
    return false;
  }

  // 等待服务可用
  if (!client->wait_for_service(std::chrono::seconds(3))) {
    out_message = "Service not available: " + service_name;
    RCLCPP_WARN(
      rclcpp::get_logger("elite_dashboard_rviz_plugin"),
      "callTriggerService(%s): service not available", service_name.c_str());
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  // 同步调用（阻塞等待最多 5 秒）
  // 同步调用（阻塞等待）
  // 根据服务不同设定不同的超时时间
  std::chrono::seconds timeout(5);

  // 抱闸动作实际需要 7–10 秒，这里适当放宽到 15 秒
  if (service_name == "/dashboard_client/brake_release") {
    timeout = std::chrono::seconds(15);
  }

  auto future = client->async_send_request(request);
  auto status = future.wait_for(timeout);

  if (status != std::future_status::ready) {
    out_message =
      "Service call timeout (response not received within " +
    std::to_string(timeout.count()) + "s): " + service_name;
    RCLCPP_WARN(
      rclcpp::get_logger("elite_dashboard_rviz_plugin"),
      "callTriggerService(%s): timeout waiting for response (action may have been executed)",
      service_name.c_str());
    return false;
  }

  try {
    auto response = future.get();
    out_message = response->message;

    if (!response->success) {
      RCLCPP_WARN(
        rclcpp::get_logger("elite_dashboard_rviz_plugin"),
        "callTriggerService(%s): service returned failure, message=%s",
        service_name.c_str(), response->message.c_str());
      return false;
    }

    RCLCPP_INFO(
      rclcpp::get_logger("elite_dashboard_rviz_plugin"),
      "callTriggerService(%s): success, message=%s",
      service_name.c_str(), response->message.c_str());
    return true;

  } catch (const std::exception & e) {
    out_message =
      std::string("Exception when calling service ") +
      service_name + ": " + e.what();

    RCLCPP_ERROR(
      rclcpp::get_logger("elite_dashboard_rviz_plugin"),
      "callTriggerService(%s): exception: %s",
      service_name.c_str(), e.what());
    return false;
  }
}

void EliteDashboardPanel::onRobotModeUpdated(QString mode_str, QString power_state_str)
{
  robot_mode_label_->setText(mode_str);
  power_state_label_->setText(power_state_str);
}

// ========== 状态刷新 ==========
void EliteDashboardPanel::refreshRobotMode()
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("elite_dashboard_rviz_plugin"),
    "refreshRobotMode() entered");

  if (!get_robot_mode_client_) {
    RCLCPP_WARN(
      rclcpp::get_logger("elite_dashboard_rviz_plugin"),
      "GetRobotMode client not initialized");
    appendLog("GetRobotMode client not initialized");
    return;
  }

  if (!node_) {
    RCLCPP_WARN(
      rclcpp::get_logger("elite_dashboard_rviz_plugin"),
      "Node pointer is null in refreshRobotMode()");
    appendLog("Node pointer is null in refreshRobotMode()");
    return;
  }

  auto req = std::make_shared<GetRobotMode::Request>();

  RCLCPP_DEBUG(
    rclcpp::get_logger("elite_dashboard_rviz_plugin"),
    "Sending async request for GetRobotMode (callback style)");

  EliteDashboardPanel * self = this;

  get_robot_mode_client_->async_send_request(
    req,
    [self](rclcpp::Client<GetRobotMode>::SharedFuture future)
    {
      try {
        auto resp = future.get();

        if (!resp->success) {
          QString msg = QString("GetRobotMode failed: %1")
                          .arg(QString::fromStdString(resp->message));
          self->appendLog(msg);
          RCLCPP_WARN(
            rclcpp::get_logger("elite_dashboard_rviz_plugin"),
            "%s", msg.toStdString().c_str());
          return;
        }

        int8_t mode = resp->mode.mode;
        QString mode_str = self->robotModeToString(mode);

        QString power_state_str;
        if (mode == eli_common_interface::msg::RobotMode::POWER_OFF) {
          power_state_str = QString::fromUtf8("已下电");
        } else if (
          mode == eli_common_interface::msg::RobotMode::POWER_ON  ||
          mode == eli_common_interface::msg::RobotMode::IDLE      ||
          mode == eli_common_interface::msg::RobotMode::BACKDRIVE ||
          mode == eli_common_interface::msg::RobotMode::RUNNING)
        {
          power_state_str = QString::fromUtf8("已上电");
        } else {
          power_state_str = QString::fromUtf8("未知");
        }

        RCLCPP_DEBUG(
          rclcpp::get_logger("elite_dashboard_rviz_plugin"),
          "GetRobotMode async callback success, mode=%d",
          static_cast<int>(mode));

        QMetaObject::invokeMethod(
          self,
          [self, mode_str, power_state_str]()
          {
            self->robot_mode_label_->setText(mode_str);
            self->power_state_label_->setText(power_state_str);
          },
          Qt::QueuedConnection);

      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          rclcpp::get_logger("elite_dashboard_rviz_plugin"),
          "GetRobotMode async callback exception: %s", e.what());
        self->appendLog(
          QString("GetRobotMode async callback exception: %1").arg(e.what()));
      }
    });
}

void EliteDashboardPanel::refreshSafetyMode()
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("elite_dashboard_rviz_plugin"),
    "refreshSafetyMode() entered");

  if (!get_safety_mode_client_) {
    appendLog("GetSafetyMode client not initialized");
    return;
  }

  if (!node_) {
    appendLog("Node pointer is null in refreshSafetyMode()");
    return;
  }

  if (!get_safety_mode_client_->wait_for_service(1s)) {
    appendLog("Service /dashboard_client/get_safety_mode not available");
    return;
  }

  auto req = std::make_shared<GetSafetyMode::Request>();
  EliteDashboardPanel * self = this;

  get_safety_mode_client_->async_send_request(
    req,
    [self](rclcpp::Client<GetSafetyMode>::SharedFuture future)
    {
      try {
        auto resp = future.get();
        if (!resp->success) {
          QString msg = QString("GetSafetyMode failed: %1")
                          .arg(QString::fromStdString(resp->message));
          self->appendLog(msg);
          RCLCPP_WARN(
            rclcpp::get_logger("elite_dashboard_rviz_plugin"),
            "%s", msg.toStdString().c_str());
          return;
        }

        int8_t mode = resp->mode.mode;
        QString mode_str = self->safetyModeToString(mode);

        RCLCPP_DEBUG(
          rclcpp::get_logger("elite_dashboard_rviz_plugin"),
          "GetSafetyMode async success, mode=%d", static_cast<int>(mode));

        QMetaObject::invokeMethod(
          self,
          [self, mode_str]() {
            self->safety_mode_label_->setText(mode_str);
          },
          Qt::QueuedConnection);

      } catch (const std::exception & e) {
        QString err = QString("GetSafetyMode async exception: %1").arg(e.what());
        self->appendLog(err);
        RCLCPP_ERROR(
          rclcpp::get_logger("elite_dashboard_rviz_plugin"),
          "%s", err.toStdString().c_str());
      }
    });
}

void EliteDashboardPanel::refreshTaskStatus()
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("elite_dashboard_rviz_plugin"),
    "refreshTaskStatus() entered");

  if (!get_task_status_client_) {
    appendLog("GetTaskStatus client not initialized");
    return;
  }
  if (!node_) {
    appendLog("Node pointer is null in refreshTaskStatus()");
    return;
  }

  if (!get_task_status_client_->wait_for_service(1s)) {
    appendLog("Service /dashboard_client/get_task_status not available");
    return;
  }

  auto req = std::make_shared<GetTaskStatus::Request>();
  EliteDashboardPanel * self = this;

  get_task_status_client_->async_send_request(
    req,
    [self](rclcpp::Client<GetTaskStatus>::SharedFuture future)
    {
      try {
        auto resp = future.get();
        if (!resp->success) {
          QString msg = QString("GetTaskStatus failed: %1")
                          .arg(QString::fromStdString(resp->message));
          self->appendLog(msg);
          RCLCPP_WARN(
            rclcpp::get_logger("elite_dashboard_rviz_plugin"),
            "%s", msg.toStdString().c_str());
          return;
        }

        int8_t status = resp->status.status;
        QString status_str = self->taskStatusToString(status);

        RCLCPP_DEBUG(
          rclcpp::get_logger("elite_dashboard_rviz_plugin"),
          "GetTaskStatus async success, status=%d", static_cast<int>(status));

        QMetaObject::invokeMethod(
          self,
          [self, status_str]() {
            self->task_status_label_->setText(status_str);
          },
          Qt::QueuedConnection);

      } catch (const std::exception & e) {
        QString err = QString("GetTaskStatus async exception: %1").arg(e.what());
        self->appendLog(err);
        RCLCPP_ERROR(
          rclcpp::get_logger("elite_dashboard_rviz_plugin"),
          "%s", err.toStdString().c_str());
      }
    });
}

void EliteDashboardPanel::refreshTaskPath()
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("elite_dashboard_rviz_plugin"),
    "refreshTaskPath() entered");

  if (!get_task_path_client_) {
    appendLog("GetTaskPath client not initialized");
    return;
  }
  if (!node_) {
    appendLog("Node pointer is null in refreshTaskPath()");
    return;
  }

  if (!get_task_path_client_->wait_for_service(1s)) {
    appendLog("Service /dashboard_client/get_task_path not available");
    return;
  }

  auto req = std::make_shared<Trigger::Request>();
  EliteDashboardPanel * self = this;

  get_task_path_client_->async_send_request(
    req,
    [self](rclcpp::Client<Trigger>::SharedFuture future)
    {
      try {
        auto resp = future.get();
        if (!resp->success) {
          QString msg = QString("GetTaskPath failed: %1")
                          .arg(QString::fromStdString(resp->message));
          self->appendLog(msg);
          RCLCPP_WARN(
            rclcpp::get_logger("elite_dashboard_rviz_plugin"),
            "%s", msg.toStdString().c_str());
          return;
        }

        QString path = QString::fromStdString(resp->message);

        RCLCPP_DEBUG(
          rclcpp::get_logger("elite_dashboard_rviz_plugin"),
          "GetTaskPath async success, path=%s", resp->message.c_str());

        QMetaObject::invokeMethod(
          self,
          [self, path]() {
            self->task_path_label_->setText(path);
          },
          Qt::QueuedConnection);

      } catch (const std::exception & e) {
        QString err = QString("GetTaskPath async exception: %1").arg(e.what());
        self->appendLog(err);
        RCLCPP_ERROR(
          rclcpp::get_logger("elite_dashboard_rviz_plugin"),
          "%s", err.toStdString().c_str());
      }
    });
}

void EliteDashboardPanel::onRefreshStatusClicked()
{
  RCLCPP_INFO(
    rclcpp::get_logger("elite_dashboard_rviz_plugin"),
    "onRefreshStatusClicked() called");

  appendLog(QString::fromUtf8("刷新状态..."));

  refreshRobotMode();
  refreshSafetyMode();
  refreshTaskStatus();
  refreshTaskPath();

  appendLog(QString::fromUtf8("刷新状态完成"));
}

// ========== 按钮槽：电源 & 抱闸 ==========
void EliteDashboardPanel::onPowerOnClicked()
{
  appendLog(QString::fromUtf8("调用 开启电源 ..."));

  if (!node_) {
    showErrorMessage(QString::fromUtf8("开启电源失败"),
                     QString::fromUtf8("节点未初始化"));
    return;
  }
  if (!load_configure_client_ || !load_task_client_ || !power_on_client_) {
    showErrorMessage(QString::fromUtf8("开启电源失败"),
                     QString::fromUtf8("服务 client 未初始化"));
    return;
  }

  if (!load_configure_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("配置文件加载失败: 服务 /dashboard_client/load_configure 不可用"));
    showErrorMessage(QString::fromUtf8("开启电源失败"),
                     QString::fromUtf8("服务 /dashboard_client/load_configure 不可用"));
    return;
  }

  const std::string config_file =
    "Removal_and_installation_of_side_shielding_sheets/"
    "Shielding_sheet_module_disassembly_and_assembly_configuration_file.configuration";

  auto cfg_req = std::make_shared<Load::Request>();
  cfg_req->filename = config_file;

  appendLog(QString::fromUtf8("开始加载配置文件: %1")
              .arg(QString::fromStdString(config_file)));

  EliteDashboardPanel * self = this;

  load_configure_client_->async_send_request(
    cfg_req,
    [self](rclcpp::Client<Load>::SharedFuture future_cfg)
    {
      self->appendLog(QString::fromUtf8("=== 进入 load_configure 回调 ==="));
      try {
        auto resp_cfg = future_cfg.get();
        QString answer = QString::fromStdString(resp_cfg->answer);
        
        self->appendLog(
          QString::fromUtf8("load_configure 返回: success=%1, answer=%2")
            .arg(resp_cfg->success ? "true" : "false")
            .arg(answer));

        if (!resp_cfg->success) {
          self->appendLog(QString::fromUtf8("配置文件加载失败: %1").arg(answer));
          self->showErrorMessage(QString::fromUtf8("配置文件加载失败"), answer);
          return;
        }

        self->appendLog(QString::fromUtf8("配置文件加载成功: %1").arg(answer));

        if (!self->load_task_client_) {
          self->showErrorMessage(QString::fromUtf8("任务文件加载失败"),
                                 QString::fromUtf8("load_task client 未初始化"));
          return;
        }

        if (!self->load_task_client_->wait_for_service(5s)) {
          self->appendLog(QString::fromUtf8("任务文件加载失败: 服务 /dashboard_client/load_task 不可用"));
          self->showErrorMessage(QString::fromUtf8("任务文件加载失败"),
                                 QString::fromUtf8("服务 /dashboard_client/load_task 不可用"));
          return;
        }

        const std::string task_file = "ros2_control.task";
        auto task_req = std::make_shared<Load::Request>();
        task_req->filename = task_file;

        self->appendLog(QString::fromUtf8("开始加载任务文件: %1")
                          .arg(QString::fromStdString(task_file)));

        self->load_task_client_->async_send_request(
          task_req,
          [self](rclcpp::Client<Load>::SharedFuture future_task)
          {
            try {
              auto resp_task = future_task.get();
              QString answer_task = QString::fromStdString(resp_task->answer);

              if (!resp_task->success) {
                self->appendLog(QString::fromUtf8("任务文件加载失败: %1").arg(answer_task));
                self->showErrorMessage(QString::fromUtf8("任务文件加载失败"), answer_task);
                return;
              }

              self->appendLog(QString::fromUtf8("任务文件加载成功: %1").arg(answer_task));

              if (!self->power_on_client_) {
                self->showErrorMessage(QString::fromUtf8("开启电源失败"),
                                       QString::fromUtf8("power_on client 未初始化"));
                return;
              }

              if (!self->power_on_client_->wait_for_service(5s)) {
                self->appendLog(QString::fromUtf8("开启电源失败: 服务 /dashboard_client/power_on 不可用"));
                self->showErrorMessage(QString::fromUtf8("开启电源失败"),
                                       QString::fromUtf8("服务 /dashboard_client/power_on 不可用"));
                return;
              }

              auto power_req = std::make_shared<std_srvs::srv::Trigger::Request>();

              self->appendLog(QString::fromUtf8("发送开启电源指令 ..."));

              self->power_on_client_->async_send_request(
                power_req,
                [self](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future_power)
                {
                  try {
                    auto resp_power = future_power.get();

                    QString detail = QString::fromStdString(resp_power->message);
                    if (!resp_power->success) {
                      self->appendLog(QString::fromUtf8("开启电源失败: %1").arg(detail));
                      self->showErrorMessage(QString::fromUtf8("开启电源失败"), detail);
                    } else {
                      self->appendLog(QString::fromUtf8("开启电源成功: %1").arg(detail));
                      
                      // ========== 新增：在电源开启成功后调用默认速度初始化 ==========
                      self->appendLog(QString::fromUtf8("开始初始化默认速度..."));
                      self->initDefaultSpeed();
                    }
                  } catch (const std::exception & e) {
                    QString err = QString::fromUtf8("开启电源异步回调异常: ") +
                                  QString::fromUtf8(e.what());
                    self->appendLog(err);
                    self->showErrorMessage(QString::fromUtf8("开启电源失败"), err);
                  }
                });

            } catch (const std::exception & e) {
              QString err = QString::fromUtf8("任务文件加载异步回调异常: ") +
                            QString::fromUtf8(e.what());
              self->appendLog(err);
              self->showErrorMessage(QString::fromUtf8("任务文件加载失败"), err);
            }
          });

      } catch (const std::exception & e) {
        QString err = QString::fromUtf8("配置文件加载异步回调异常: ") +
                      QString::fromUtf8(e.what());
        self->appendLog(err);
        self->showErrorMessage(QString::fromUtf8("配置文件加载失败"), err);
      }
    });
}

void EliteDashboardPanel::onPowerOffClicked()
{
  std::string msg;
  appendLog(QString::fromUtf8("调用 关闭电源 ..."));
  bool ok = callTriggerService(power_off_client_, "/dashboard_client/power_off", msg);

  if (!ok) {
    QString qmsg = QString::fromStdString(msg);
    appendLog(QString::fromUtf8("关闭电源指令发送失败: %1").arg(qmsg));
    showErrorMessage(QString::fromUtf8("关闭电源指令发送失败"), qmsg);
  } else {
    appendLog(QString::fromUtf8("关闭电源指令已发送: %1").arg(QString::fromStdString(msg)));
  }
}

void EliteDashboardPanel::onBrakeReleaseClicked()
{
  std::string msg;
  appendLog(QString::fromUtf8("调用 释放抱闸 ..."));
  bool ok = callTriggerService(brake_release_client_, "/dashboard_client/brake_release", msg);

  QString qmsg = QString::fromStdString(msg);

  if (!ok) {
    // 区分“响应超时”和“明确失败”
    if (qmsg.contains("timeout", Qt::CaseInsensitive)) {
      appendLog(QString::fromUtf8("释放抱闸服务响应超时（机械臂动作可能已执行）：%1").arg(qmsg));
      showErrorMessage(
        QString::fromUtf8("释放抱闸服务响应超时"),
        QString::fromUtf8("服务未在超时时间内返回响应，但机械臂可能已经完成释放抱闸，请通过状态或现场确认。\n详情：") + qmsg);
    } else {
      appendLog(QString::fromUtf8("释放抱闸指令失败: %1").arg(qmsg));
      showErrorMessage(QString::fromUtf8("释放抱闸指令失败"), qmsg);
    }
  } else {
    appendLog(QString::fromUtf8("释放抱闸指令成功: %1").arg(qmsg));
  }
}

// ========== 一键启动（headless 模式下重发 ExternalControl 脚本） ==========
void EliteDashboardPanel::onOneClickStartClicked()
{
  if (one_click_running_) {
    appendLog(QString::fromUtf8("一键启动已在执行中，忽略重复点击"));
    return;
  }

  if (!resend_external_script_client_) {
    appendLog(QString::fromUtf8("一键启动失败：resend_external_script client 未初始化"));
    showErrorMessage(QString::fromUtf8("一键启动失败"),
                     QString::fromUtf8("resend_external_script client 未初始化"));
    return;
  }

  if (!node_) {
    appendLog(QString::fromUtf8("一键启动失败：节点未初始化"));
    showErrorMessage(QString::fromUtf8("一键启动失败"),
                     QString::fromUtf8("节点未初始化"));
    return;
  }

  if (!resend_external_script_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("一键启动失败：服务 /io_and_status_controller/resend_external_script 不可用"));
    showErrorMessage(QString::fromUtf8("一键启动失败"),
                     QString::fromUtf8("服务 /io_and_status_controller/resend_external_script 不可用"));
    return;
  }

  one_click_running_ = true;
  one_click_start_btn_->setEnabled(false);

  appendLog(QString::fromUtf8("一键启动开始：重新发送 ExternalControl 脚本..."));

  auto req = std::make_shared<Trigger::Request>();

  resend_external_script_client_->async_send_request(
    req,
    [this](rclcpp::Client<Trigger>::SharedFuture future) {
      try {
        auto resp = future.get();

        QString detail = QString::fromStdString(resp->message);

        if (resp->success) {
          appendLog(QString::fromUtf8("一键启动成功：ExternalControl 已重新下发（%1）").arg(detail));
        } else {
          appendLog(QString::fromUtf8("一键启动失败：resend_external_script 返回失败（%1）").arg(detail));
          showErrorMessage(QString::fromUtf8("一键启动失败"),
                           QString::fromUtf8("resend_external_script 返回失败：") + detail);
        }
      } catch (const std::exception & e) {
        QString err = QString::fromUtf8("调用 resend_external_script 异常：") +
                      QString::fromUtf8(e.what());
        appendLog(err);
        showErrorMessage(QString::fromUtf8("一键启动失败"), err);
      }

      one_click_running_ = false;
      if (one_click_start_btn_) {
        QMetaObject::invokeMethod(
          this,
          [this]() {
            one_click_start_btn_->setEnabled(true);
          },
          Qt::QueuedConnection);
      }
    });
}

// ========== 日志区按钮 ==========
void EliteDashboardPanel::onClearLogClicked()
{
  log_text_edit_->clear();
}

void EliteDashboardPanel::onSendLogClicked()
{
  if (!log_client_) {
    appendLog("Log client not initialized");
    return;
  }
  if (!log_client_->wait_for_service(3s)) {
    appendLog("Service /dashboard_client/log not available");
    return;
  }

  auto req = std::make_shared<Log::Request>();
  req->message = "Message from RViz EliteDashboardPanel";

  appendLog("Sending log to /dashboard_client/log (async)...");

  auto callback = [this](rclcpp::Client<Log>::SharedFuture future) {
    try {
      auto resp = future.get();
      if (!resp->success) {
        appendLog(QString("Log service failed: %1")
                      .arg(QString::fromStdString(resp->message)));
      } else {
        appendLog(QString("Log service success: %1")
                      .arg(QString::fromStdString(resp->message)));
      }
    } catch (const std::exception &e) {
      appendLog(QString("Log service exception in callback: %1").arg(e.what()));
    }
  };

  log_client_->async_send_request(req, callback);
}

// ========== 工具 IO ==========
void EliteDashboardPanel::callSetIO(int8_t pin, bool on)
{
  if (!set_io_client_) {
    appendLog("SetIO client not initialized");
    return;
  }
  if (!set_io_client_->wait_for_service(1s)) {
    appendLog("Service /io_and_status_controller/set_io not available");
    return;
  }

  auto req = std::make_shared<SetIO::Request>();
  req->fun = SetIO::Request::FUN_SET_TOOL_DIG_OUT;
  req->pin = pin;
  req->analog_type = SetIO::Request::ANALOG_CURRENT;
  req->state = on ? SetIO::Request::STATE_ON : SetIO::Request::STATE_OFF;

  appendLog(QString("SetIO pin %1 -> %2").arg(pin).arg(on ? "ON" : "OFF"));

  set_io_client_->async_send_request(
    req,
    [pin, on](rclcpp::Client<SetIO>::SharedFuture future)
    {
      try {
        auto resp = future.get();
        if (!resp->success) {
          RCLCPP_WARN(
            rclcpp::get_logger("elite_dashboard_rviz_plugin"),
            "SetIO failed for pin %d, state %s",
            pin, on ? "ON" : "OFF");
        } else {
          RCLCPP_INFO(
            rclcpp::get_logger("elite_dashboard_rviz_plugin"),
            "SetIO success for pin %d, state %s",
            pin, on ? "ON" : "OFF");
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          rclcpp::get_logger("elite_dashboard_rviz_plugin"),
          "Exception in SetIO response callback: %s", e.what());
      }
    });
}

void EliteDashboardPanel::onT0OnClicked()
{
  callSetIO(0, true);
}

void EliteDashboardPanel::onT0OffClicked()
{
  callSetIO(0, false);
}

void EliteDashboardPanel::onT1OnClicked()
{
  callSetIO(1, true);
}

void EliteDashboardPanel::onT1OffClicked()
{
  callSetIO(1, false);
}

void EliteDashboardPanel::onInnerClicked()
{
  callSetIO(0, true);
  callSetIO(1, true);
}

void EliteDashboardPanel::onOuterClicked()
{
  callSetIO(0, false);
  callSetIO(1, false);
}

// ========== 枚举转字符串 ==========
QString EliteDashboardPanel::robotModeToString(int8_t mode)
{
  using eli_common_interface::msg::RobotMode;

  switch (mode) {
    case RobotMode::UNKNOWN: return QString::fromUtf8("UNKNOWN");
    case RobotMode::NO_CONTROLLER: return QString::fromUtf8("NO_CONTROLLER");
    case RobotMode::DISCONNECTED: return QString::fromUtf8("DISCONNECTED");
    case RobotMode::CONFIRM_SAFETY: return QString::fromUtf8("CONFIRM_SAFETY");
    case RobotMode::BOOTING: return QString::fromUtf8("BOOTING");
    case RobotMode::POWER_OFF: return QString::fromUtf8("POWER_OFF");
    case RobotMode::POWER_ON: return QString::fromUtf8("POWER_ON");
    case RobotMode::IDLE: return QString::fromUtf8("IDLE");
    case RobotMode::BACKDRIVE: return QString::fromUtf8("BACKDRIVE");
    case RobotMode::RUNNING: return QString::fromUtf8("RUNNING");
    case RobotMode::UPDATING_FIRMWARE: return QString::fromUtf8("UPDATING_FIRMWARE");
    case RobotMode::WAITING_CALIBRATION: return QString::fromUtf8("WAITING_CALIBRATION");
    default: return QString("UNKNOWN(%1)").arg(mode);
  }
}

QString EliteDashboardPanel::safetyModeToString(int8_t mode)
{
  using eli_common_interface::msg::SafetyMode;

  switch (mode) {
    case SafetyMode::UNKNOWN: return QString::fromUtf8("UNKNOWN");
    case SafetyMode::NORMAL: return QString::fromUtf8("NORMAL");
    case SafetyMode::REDUCED: return QString::fromUtf8("REDUCED");
    case SafetyMode::PROTECTIVE_STOP: return QString::fromUtf8("PROTECTIVE_STOP");
    case SafetyMode::RECOVERY: return QString::fromUtf8("RECOVERY");
    case SafetyMode::SAFEGUARD_STOP: return QString::fromUtf8("SAFEGUARD_STOP");
    case SafetyMode::SYSTEM_EMERGENCY_STOP: return QString::fromUtf8("SYSTEM_ESTOP");
    case SafetyMode::ROBOT_EMERGENCY_STOP: return QString::fromUtf8("ROBOT_ESTOP");
    case SafetyMode::VIOLATION: return QString::fromUtf8("VIOLATION");
    case SafetyMode::FAULT: return QString::fromUtf8("FAULT");
    case SafetyMode::VALIDATE_JOINT_ID: return QString::fromUtf8("VALIDATE_JOINT_ID");
    case SafetyMode::UNDEFINED_SAFETY_MODE: return QString::fromUtf8("UNDEFINED");
    case SafetyMode::AUTOMATIC_MODE_SAFEGUARD_STOP:
      return QString::fromUtf8("AUTO_MODE_SAFEGUARD_STOP");
    case SafetyMode::SYSTEM_THREE_POSITION_ENABLING_STOP:
      return QString::fromUtf8("SYSTEM_3POS_EN_STOP");
    case SafetyMode::TP_THREE_POSITION_ENABLING_STOP:
      return QString::fromUtf8("TP_3POS_EN_STOP");
    default: return QString("UNKNOWN(%1)").arg(mode);
  }
}

QString EliteDashboardPanel::taskStatusToString(int8_t status)
{
  using eli_common_interface::msg::TaskStatus;

  switch (status) {
    case TaskStatus::UNKNOWN: return QString::fromUtf8("UNKNOWN");
    case TaskStatus::STOPPED: return QString::fromUtf8("STOPPED");
    case TaskStatus::PAUSED: return QString::fromUtf8("PAUSED");
    case TaskStatus::PLAYING: return QString::fromUtf8("PLAYING");
    default: return QString("UNKNOWN(%1)").arg(status);
  }
}

// ========== RViz 配置保存/加载（暂不保存任何自定义配置） ==========
void EliteDashboardPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void EliteDashboardPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

}  // namespace elite_dashboard_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  elite_dashboard_rviz_plugin::EliteDashboardPanel,
  rviz_common::Panel)
