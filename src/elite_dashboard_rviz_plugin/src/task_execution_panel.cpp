#include "elite_dashboard_rviz_plugin/task_execution_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QMetaObject>
#include <QThread>
#include <QScrollBar>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <chrono>
#include <iostream>
#include <sstream>

using namespace std::chrono_literals;

namespace elite_dashboard_rviz_plugin
{

TaskExecutionPanel::TaskExecutionPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  std::cout << "[TaskExecutionPanel] ctor begin" << std::endl;
  initUI();
  std::cout << "[TaskExecutionPanel] ctor end" << std::endl;
}

TaskExecutionPanel::~TaskExecutionPanel()
{
  // cs625_compliant_placement 作为 report-only 节点，这里仅停止由 panel 启动的本地进程
  stopProcess(compliant_placement_process_, QString::fromUtf8("Compliant Placement Node"));
  std::cout << "[TaskExecutionPanel] dtor" << std::endl;
}

void TaskExecutionPanel::onInitialize()
{
  std::cout << "[TaskExecutionPanel] onInitialize begin" << std::endl;
  initRosNode();
  appendLog(QString::fromUtf8("TaskExecutionPanel 已初始化（装入 / 取出双流程版）"));
  appendLog(QString::fromUtf8(
    "请先在“流程启动确认”中选择“启动装入流程”或“启动取出流程”。"));
  updateFlowModeDisplay();
  updatePendingPlanDisplay();
  updateMultiConfigHandoffDisplay();
  updateHomeReturnDisplay();
  updateCompliantPlacementDisplay();
  updateProcessControlDisplay();
  updateButtonStates();
  std::cout << "[TaskExecutionPanel] onInitialize end" << std::endl;
}

void TaskExecutionPanel::initRosNode()
{
  std::cout << "[TaskExecutionPanel] initRosNode: enter" << std::endl;

  auto context = getDisplayContext();
  if (!context) {
    std::cout << "[TaskExecutionPanel] initRosNode: context is null" << std::endl;
  } else {
    std::cout << "[TaskExecutionPanel] initRosNode: got context" << std::endl;
    auto ros_node_abstraction = context->getRosNodeAbstraction().lock();
    if (!ros_node_abstraction) {
      std::cout << "[TaskExecutionPanel] initRosNode: ros_node_abstraction is null" << std::endl;
    } else {
      std::cout << "[TaskExecutionPanel] initRosNode: got ros_node_abstraction" << std::endl;
      node_ = ros_node_abstraction->get_raw_node();
    }
  }

  if (!node_) {
    std::cout << "[TaskExecutionPanel] initRosNode: node_ is null, creating own node" << std::endl;
    node_ = rclcpp::Node::make_shared("task_execution_panel_node");
  } else {
    std::cout << "[TaskExecutionPanel] initRosNode: using RViz node" << std::endl;
  }

  task_state_sub_ = node_->create_subscription<cs625_task_manager::msg::TaskState>(
    "/task_state",
    10,
    std::bind(&TaskExecutionPanel::taskStateCallback, this, std::placeholders::_1));
    


  trigger_step_client_ =
    node_->create_client<cs625_task_manager::srv::TriggerStep>("/trigger_step");
  confirm_grasp_client_ =
    node_->create_client<cs625_task_manager::srv::ConfirmAction>("/confirm_grasp");
  confirm_release_client_ =
    node_->create_client<cs625_task_manager::srv::ConfirmAction>("/confirm_release");
  confirm_step_completion_client_ =
    node_->create_client<cs625_task_manager::srv::ConfirmAction>("/confirm_step_completion");

  plan_client_ =
    node_->create_client<cs625_trajectory_tools::srv::PlanToFrame>(
      "/cs625/plan_to_frame");
  execute_client_ =
    node_->create_client<cs625_trajectory_tools::srv::ExecuteLastPlan>(
      "/cs625/execute_last_plan");
  cartesian_plan_client_ =
    node_->create_client<cs625_trajectory_tools::srv::CartesianPlanToFrame>(
      "/cs625/cartesian_plan_to_frame");

  cartesian_force_compensated_client_ =
    node_->create_client<cs625_trajectory_tools::srv::CartesianForceCompensatedToFrame>(
      "/cs625/cartesian_force_compensated_to_frame");

  prepare_unload_flow_client_ =
    node_->create_client<std_srvs::srv::Trigger>("/prepare_unload_flow");

  compliant_placement_state_sub_ =
    node_->create_subscription<cs625_compliant_placement::msg::CompliantPlacementState>(
      "/cs625/compliant_placement/state",
      10,
      std::bind(&TaskExecutionPanel::compliantPlacementStateCallback, this, std::placeholders::_1));
      
  tcp_force_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/cs625/tcp_force",
      10,
      std::bind(&TaskExecutionPanel::tcpForceCallback, this, std::placeholders::_1));
      



  std::cout << "[TaskExecutionPanel] initRosNode: subscribed /task_state, "
            << "/cs625/compliant_placement/state and /cs625/tcp_force, created task_manager, "
            << "trajectory, force_compensated_cartesian and prepare_unload_flow clients "
            << "(compliant_placement is report-only, no start/stop service clients)"
            << std::endl;
}

void TaskExecutionPanel::initUI()
{
  auto * main_layout = new QVBoxLayout;

  // ----------------------------
  // 流程启动确认
  // ----------------------------
  auto * flow_group = new QGroupBox(QString::fromUtf8("流程启动确认"));
  auto * flow_layout = new QGridLayout;

  start_load_flow_button_ = new QPushButton(QString::fromUtf8("启动装入流程"));
  start_unload_flow_button_ = new QPushButton(QString::fromUtf8("启动取出流程"));
  current_flow_mode_value_ = new QLabel(QString::fromUtf8("未选择"));

  flow_layout->addWidget(start_load_flow_button_, 0, 0);
  flow_layout->addWidget(start_unload_flow_button_, 0, 1);
  flow_layout->addWidget(new QLabel(QString::fromUtf8("当前流程模式：")), 1, 0);
  flow_layout->addWidget(current_flow_mode_value_, 1, 1);

  flow_group->setLayout(flow_layout);
  main_layout->addWidget(flow_group);

  // ----------------------------
  // 装入流程
  // ----------------------------
  load_step_group_ = new QGroupBox(QString::fromUtf8("当前装入执行阶段流程"));
  auto * load_step_layout = new QGridLayout;

  e0_button_ = new QPushButton(QString::fromUtf8("E0 复位 IO"));
  e1_button_ = new QPushButton(QString::fromUtf8("E1 抓取预备"));
  e2_button_ = new QPushButton(QString::fromUtf8("E2 进入抓取位"));
  e3_button_ = new QPushButton(QString::fromUtf8("E3 抓取"));
  e4_button_ = new QPushButton(QString::fromUtf8("E4 中转"));
  e5_button_ = new QPushButton(QString::fromUtf8("E5 到预插入位"));
  e6_button_ = new QPushButton(QString::fromUtf8("E6 预插入旋转"));
  e7_button_ = new QPushButton(QString::fromUtf8("E7 最终插入"));
  e8_button_ = new QPushButton(QString::fromUtf8("E8 释放"));
  e9_button_ = new QPushButton(QString::fromUtf8("E9 撤出夹爪"));

  load_step_layout->addWidget(e0_button_, 0, 0);
  load_step_layout->addWidget(e1_button_, 0, 1);
  load_step_layout->addWidget(e2_button_, 0, 2);
  load_step_layout->addWidget(e3_button_, 0, 3);
  load_step_layout->addWidget(e4_button_, 0, 4);

  load_step_layout->addWidget(e5_button_, 1, 0);
  load_step_layout->addWidget(e6_button_, 1, 1);
  load_step_layout->addWidget(e7_button_, 1, 2);
  load_step_layout->addWidget(e8_button_, 1, 3);
  load_step_layout->addWidget(e9_button_, 1, 4);

  load_step_group_->setLayout(load_step_layout);
  main_layout->addWidget(load_step_group_);

  // ----------------------------
  // 取出流程
  // ----------------------------
  unload_step_group_ = new QGroupBox(QString::fromUtf8("当前取出执行阶段流程"));
  auto * unload_step_layout = new QGridLayout;

  d0_button_ = new QPushButton(QString::fromUtf8("D0 复位 IO"));
  d1_button_ = new QPushButton(QString::fromUtf8("D1 抓取预备"));
  d2_button_ = new QPushButton(QString::fromUtf8("D2 进入抓取位"));
  d3_button_ = new QPushButton(QString::fromUtf8("D3 抓取"));
  d4_button_ = new QPushButton(QString::fromUtf8("D4 旋转解锁"));
  d5_button_ = new QPushButton(QString::fromUtf8("D5 取出"));
  d6_button_ = new QPushButton(QString::fromUtf8("D6 中转"));
  d7_button_ = new QPushButton(QString::fromUtf8("D7 到达放置点"));
  d8_button_ = new QPushButton(QString::fromUtf8("D8 释放"));
  d9_button_ = new QPushButton(QString::fromUtf8("D9 撤出夹爪"));

  unload_step_layout->addWidget(d0_button_, 0, 0);
  unload_step_layout->addWidget(d1_button_, 0, 1);
  unload_step_layout->addWidget(d2_button_, 0, 2);
  unload_step_layout->addWidget(d3_button_, 0, 3);
  unload_step_layout->addWidget(d4_button_, 0, 4);

  unload_step_layout->addWidget(d5_button_, 1, 0);
  unload_step_layout->addWidget(d6_button_, 1, 1);
  unload_step_layout->addWidget(d7_button_, 1, 2);
  unload_step_layout->addWidget(d8_button_, 1, 3);
  unload_step_layout->addWidget(d9_button_, 1, 4);

  unload_step_group_->setLayout(unload_step_layout);
  main_layout->addWidget(unload_step_group_);
  
  auto * home_group = new QGroupBox(QString::fromUtf8("HOME_RETURN 公共回位步骤"));
  auto * home_layout = new QGridLayout;

  home_return_group_ = home_group;
  home_return_button_ = new QPushButton(QString::fromUtf8("HOME_RETURN 回位"));
  home_return_status_value_ = new QLabel(QString::fromUtf8("未解锁"));

  home_layout->addWidget(home_return_button_, 0, 0, 1, 2);
  home_layout->addWidget(new QLabel(QString::fromUtf8("HOME_RETURN 状态：")), 1, 0);
  home_layout->addWidget(home_return_status_value_, 1, 1);

  home_group->setLayout(home_layout);
  main_layout->addWidget(home_group);

  auto * status_group = new QGroupBox(QString::fromUtf8("当前状态"));
  auto * status_layout = new QGridLayout;

  current_phase_value_ = new QLabel;
  current_step_value_ = new QLabel;
  status_value_ = new QLabel;
  exec_substate_value_ = new QLabel;
  box_state_value_ = new QLabel;
  load_state_value_ = new QLabel;

  status_layout->addWidget(new QLabel(QString::fromUtf8("当前阶段：")), 0, 0);
  status_layout->addWidget(current_phase_value_, 0, 1);

  status_layout->addWidget(new QLabel(QString::fromUtf8("当前步骤：")), 0, 2);
  status_layout->addWidget(current_step_value_, 0, 3);

  status_layout->addWidget(new QLabel(QString::fromUtf8("当前状态：")), 1, 0);
  status_layout->addWidget(status_value_, 1, 1);

  status_layout->addWidget(new QLabel(QString::fromUtf8("执行子状态：")), 1, 2);
  status_layout->addWidget(exec_substate_value_, 1, 3);

  status_layout->addWidget(new QLabel(QString::fromUtf8("箱体状态：")), 2, 0);
  status_layout->addWidget(box_state_value_, 2, 1);

  status_layout->addWidget(new QLabel(QString::fromUtf8("载荷状态：")), 2, 2);
  status_layout->addWidget(load_state_value_, 2, 3);

  status_group->setLayout(status_layout);
  main_layout->addWidget(status_group);

  auto * condition_group = new QGroupBox(QString::fromUtf8("抓取 / 释放确认条件"));
  auto * condition_layout = new QGridLayout;

  reached_box_grasp_value_ = new QLabel;
  io_double_low_value_ = new QLabel;
  operator_grasp_value_ = new QLabel;

  reached_slot_insert_value_ = new QLabel;
  io_double_high_value_ = new QLabel;
  operator_release_value_ = new QLabel;

  condition_layout->addWidget(new QLabel(QString::fromUtf8("【抓取确认】已到达抓取位：")), 0, 0);
  condition_layout->addWidget(reached_box_grasp_value_, 0, 1);

  condition_layout->addWidget(new QLabel(QString::fromUtf8("【抓取确认】IO 双低：")), 1, 0);
  condition_layout->addWidget(io_double_low_value_, 1, 1);

  condition_layout->addWidget(new QLabel(QString::fromUtf8("【抓取确认】人工确认：")), 2, 0);
  condition_layout->addWidget(operator_grasp_value_, 2, 1);

  condition_layout->addWidget(new QLabel(QString::fromUtf8("【释放确认】已到达插入位：")), 0, 2);
  condition_layout->addWidget(reached_slot_insert_value_, 0, 3);

  condition_layout->addWidget(new QLabel(QString::fromUtf8("【释放确认】IO 双高：")), 1, 2);
  condition_layout->addWidget(io_double_high_value_, 1, 3);

  condition_layout->addWidget(new QLabel(QString::fromUtf8("【释放确认】人工确认：")), 2, 2);
  condition_layout->addWidget(operator_release_value_, 2, 3);

  condition_group->setLayout(condition_layout);
  main_layout->addWidget(condition_group);

  auto * action_group = new QGroupBox(QString::fromUtf8("人工确认与调试"));
  auto * action_layout = new QHBoxLayout;

  confirm_grasp_button_ = new QPushButton(QString::fromUtf8("确认抓取成功"));
  confirm_release_button_ = new QPushButton(QString::fromUtf8("确认释放成功"));
  confirm_step_completion_button_ = new QPushButton(QString::fromUtf8("确认本步完成并解锁下一步"));
  reset_ui_button_ = new QPushButton(QString::fromUtf8("重置 UI 状态"));

  action_layout->addWidget(confirm_grasp_button_);
  action_layout->addWidget(confirm_release_button_);
  action_layout->addWidget(confirm_step_completion_button_);
  action_layout->addWidget(reset_ui_button_);

  action_group->setLayout(action_layout);
  main_layout->addWidget(action_group);

  auto * pending_group = new QGroupBox(QString::fromUtf8("规划确认执行"));
  auto * pending_layout = new QGridLayout;

  pending_plan_value_ = new QLabel(QString::fromUtf8("无"));
  multi_config_handoff_value_ = new QLabel(QString::fromUtf8("无"));

  execute_pending_button_ = new QPushButton(QString::fromUtf8("执行当前规划"));
  clear_pending_button_ = new QPushButton(QString::fromUtf8("清除当前规划"));
  handoff_to_multi_config_button_ = new QPushButton(QString::fromUtf8("转入多构型规划"));
  finish_multi_config_button_ = new QPushButton(QString::fromUtf8("我已完成多构型处理"));
  finish_multi_config_and_advance_button_ =
    new QPushButton(QString::fromUtf8("多构型已执行并推进任务"));

  pending_layout->addWidget(new QLabel(QString::fromUtf8("当前待执行规划：")), 0, 0);
  pending_layout->addWidget(pending_plan_value_, 0, 1);

  pending_layout->addWidget(new QLabel(QString::fromUtf8("当前多构型处理：")), 1, 0);
  pending_layout->addWidget(multi_config_handoff_value_, 1, 1);

  pending_layout->addWidget(execute_pending_button_, 2, 0);
  pending_layout->addWidget(clear_pending_button_, 2, 1);

  pending_layout->addWidget(handoff_to_multi_config_button_, 3, 0);
  pending_layout->addWidget(finish_multi_config_button_, 3, 1);

  pending_layout->addWidget(finish_multi_config_and_advance_button_, 4, 0, 1, 2);

  pending_group->setLayout(pending_layout);
  main_layout->addWidget(pending_group);

  auto * process_group = new QGroupBox(QString::fromUtf8("柔顺放置控制与力反馈数据"));
  auto * process_layout = new QGridLayout;

  process_control_group_ = process_group;

  start_compliant_placement_process_button_ =
    new QPushButton(QString::fromUtf8("启动柔顺放置节点"));
  stop_compliant_placement_process_button_ =
    new QPushButton(QString::fromUtf8("停止柔顺放置节点"));
  compliant_placement_process_status_value_ =
    new QLabel(QString::fromUtf8("未启动"));

  cp_state_value_ = new QLabel(QString::fromUtf8("UNKNOWN"));
  cp_active_value_ = new QLabel(QString::fromUtf8("false"));
  cp_contact_detected_value_ = new QLabel(QString::fromUtf8("false"));
  cp_jam_detected_value_ = new QLabel(QString::fromUtf8("false"));
  cp_target_reached_value_ = new QLabel(QString::fromUtf8("false"));
  cp_aborted_value_ = new QLabel(QString::fromUtf8("false"));
  cp_stop_reason_value_ = new QLabel(QString::fromUtf8("N/A"));
  cp_tau_metric_value_ = new QLabel(QString::fromUtf8("0.0"));
  cp_filtered_tau_metric_value_ = new QLabel(QString::fromUtf8("0.0"));


  tcp_force_x_value_ = new QLabel(QString::fromUtf8("0.0"));
  tcp_force_y_value_ = new QLabel(QString::fromUtf8("0.0"));
  tcp_force_z_value_ = new QLabel(QString::fromUtf8("0.0"));
  tcp_torque_x_value_ = new QLabel(QString::fromUtf8("0.0"));
  tcp_torque_y_value_ = new QLabel(QString::fromUtf8("0.0"));
  tcp_torque_z_value_ = new QLabel(QString::fromUtf8("0.0"));

  process_layout->addWidget(start_compliant_placement_process_button_, 0, 0);
  process_layout->addWidget(stop_compliant_placement_process_button_, 0, 1);
  process_layout->addWidget(new QLabel(QString::fromUtf8("柔顺放置节点状态：")), 0, 2);
  process_layout->addWidget(compliant_placement_process_status_value_, 0, 3);

  process_layout->addWidget(new QLabel(QString::fromUtf8("柔顺放置状态：")), 1, 0);
  process_layout->addWidget(cp_state_value_, 1, 1);
  process_layout->addWidget(new QLabel(QString::fromUtf8("active：")), 1, 2);
  process_layout->addWidget(cp_active_value_, 1, 3);

  process_layout->addWidget(new QLabel(QString::fromUtf8("contact_detected：")), 2, 0);
  process_layout->addWidget(cp_contact_detected_value_, 2, 1);
  process_layout->addWidget(new QLabel(QString::fromUtf8("jam_detected：")), 2, 2);
  process_layout->addWidget(cp_jam_detected_value_, 2, 3);

  process_layout->addWidget(new QLabel(QString::fromUtf8("target_reached：")), 3, 0);
  process_layout->addWidget(cp_target_reached_value_, 3, 1);
  process_layout->addWidget(new QLabel(QString::fromUtf8("aborted：")), 3, 2);
  process_layout->addWidget(cp_aborted_value_, 3, 3);

  process_layout->addWidget(new QLabel(QString::fromUtf8("stop_reason：")), 4, 0);
  process_layout->addWidget(cp_stop_reason_value_, 4, 1, 1, 3);

  process_layout->addWidget(new QLabel(QString::fromUtf8("tau_metric：")), 5, 0);
  process_layout->addWidget(cp_tau_metric_value_, 5, 1);
  process_layout->addWidget(new QLabel(QString::fromUtf8("filtered_tau_metric：")), 5, 2);
  process_layout->addWidget(cp_filtered_tau_metric_value_, 5, 3);



  process_layout->addWidget(new QLabel(QString::fromUtf8("TCP 受力/力矩：")), 6, 0, 1, 4);

  process_layout->addWidget(new QLabel(QString::fromUtf8("TCP Force X：")), 7, 0);
  process_layout->addWidget(tcp_force_x_value_, 7, 1);
  process_layout->addWidget(new QLabel(QString::fromUtf8("TCP Torque X：")), 7, 2);
  process_layout->addWidget(tcp_torque_x_value_, 7, 3);

  process_layout->addWidget(new QLabel(QString::fromUtf8("TCP Force Y：")), 8, 0);
  process_layout->addWidget(tcp_force_y_value_, 8, 1);
  process_layout->addWidget(new QLabel(QString::fromUtf8("TCP Torque Y：")), 8, 2);
  process_layout->addWidget(tcp_torque_y_value_, 8, 3);

  process_layout->addWidget(new QLabel(QString::fromUtf8("TCP Force Z：")), 9, 0);
  process_layout->addWidget(tcp_force_z_value_, 9, 1);
  process_layout->addWidget(new QLabel(QString::fromUtf8("TCP Torque Z：")), 9, 2);
  process_layout->addWidget(tcp_torque_z_value_, 9, 3);

  process_group->setLayout(process_layout);
  main_layout->addWidget(process_group);
  
  auto * log_group = new QGroupBox(QString::fromUtf8("执行日志"));
  auto * log_layout = new QVBoxLayout;

  log_text_ = new QPlainTextEdit;
  log_text_->setReadOnly(true);

  log_layout->addWidget(log_text_);
  log_group->setLayout(log_layout);
  main_layout->addWidget(log_group);

  setLayout(main_layout);

  updateFlowModeDisplay();
  updateStatusDisplay();
  updatePendingPlanDisplay();
  updateMultiConfigHandoffDisplay();
  updateHomeReturnDisplay();
  
  updateCompliantPlacementDisplay();
  updateProcessControlDisplay();
  updateTcpForceDisplay();
  
  updateButtonStates();

  connect(start_load_flow_button_, &QPushButton::clicked, this, &TaskExecutionPanel::onStartLoadFlowClicked);
  connect(start_unload_flow_button_, &QPushButton::clicked, this, &TaskExecutionPanel::onStartUnloadFlowClicked);

  connect(e0_button_, &QPushButton::clicked, this, &TaskExecutionPanel::onE0Clicked);
  connect(e1_button_, &QPushButton::clicked, this, &TaskExecutionPanel::onE1Clicked);
  connect(e2_button_, &QPushButton::clicked, this, &TaskExecutionPanel::onE2Clicked);
  connect(e3_button_, &QPushButton::clicked, this, &TaskExecutionPanel::onE3Clicked);
  connect(e4_button_, &QPushButton::clicked, this, &TaskExecutionPanel::onE4Clicked);
  connect(e5_button_, &QPushButton::clicked, this, &TaskExecutionPanel::onE5Clicked);
  connect(e6_button_, &QPushButton::clicked, this, &TaskExecutionPanel::onE6Clicked);
  connect(e7_button_, &QPushButton::clicked, this, &TaskExecutionPanel::onE7Clicked);
  connect(e8_button_, &QPushButton::clicked, this, &TaskExecutionPanel::onE8Clicked);
  connect(e9_button_, &QPushButton::clicked, this, &TaskExecutionPanel::onE9Clicked);

  connect(d0_button_, &QPushButton::clicked, this, &TaskExecutionPanel::onD0Clicked);
  connect(d1_button_, &QPushButton::clicked, this, &TaskExecutionPanel::onD1Clicked);
  connect(d2_button_, &QPushButton::clicked, this, &TaskExecutionPanel::onD2Clicked);
  connect(d3_button_, &QPushButton::clicked, this, &TaskExecutionPanel::onD3Clicked);
  connect(d4_button_, &QPushButton::clicked, this, &TaskExecutionPanel::onD4Clicked);
  connect(d5_button_, &QPushButton::clicked, this, &TaskExecutionPanel::onD5Clicked);
  connect(d6_button_, &QPushButton::clicked, this, &TaskExecutionPanel::onD6Clicked);
  connect(d7_button_, &QPushButton::clicked, this, &TaskExecutionPanel::onD7Clicked);
  connect(d8_button_, &QPushButton::clicked, this, &TaskExecutionPanel::onD8Clicked);
  connect(d9_button_, &QPushButton::clicked, this, &TaskExecutionPanel::onD9Clicked);

  connect(
    confirm_grasp_button_, &QPushButton::clicked,
    this, &TaskExecutionPanel::onConfirmGraspClicked);
  connect(
    confirm_release_button_, &QPushButton::clicked,
    this, &TaskExecutionPanel::onConfirmReleaseClicked);
  connect(
    confirm_step_completion_button_, &QPushButton::clicked,
    this, &TaskExecutionPanel::onConfirmStepCompletionClicked);
  connect(
    reset_ui_button_, &QPushButton::clicked,
    this, &TaskExecutionPanel::onResetUiClicked);

  connect(
    execute_pending_button_, &QPushButton::clicked,
    this, &TaskExecutionPanel::onExecutePendingClicked);
  connect(
    clear_pending_button_, &QPushButton::clicked,
    this, &TaskExecutionPanel::onClearPendingClicked);
  connect(
    handoff_to_multi_config_button_, &QPushButton::clicked,
    this, &TaskExecutionPanel::onHandoffToMultiConfigClicked);
  connect(
    finish_multi_config_button_, &QPushButton::clicked,
    this, &TaskExecutionPanel::onFinishMultiConfigClicked);
  connect(
    finish_multi_config_and_advance_button_, &QPushButton::clicked,
    this, &TaskExecutionPanel::onFinishMultiConfigAndAdvanceClicked);
    
  connect(
    home_return_button_, &QPushButton::clicked,
    this, &TaskExecutionPanel::onHomeReturnClicked);
    

  connect(
    start_compliant_placement_process_button_, &QPushButton::clicked,
    this, &TaskExecutionPanel::onStartCompliantPlacementProcessClicked);

  connect(
    stop_compliant_placement_process_button_, &QPushButton::clicked,
    this, &TaskExecutionPanel::onStopCompliantPlacementProcessClicked);
    

}

void TaskExecutionPanel::appendLog(const QString & text)
{
  if (!log_text_) {
    return;
  }

  auto append_fn = [this, text]() {
    if (!log_text_) {
      return;
    }
    log_text_->appendPlainText(text);
    auto * bar = log_text_->verticalScrollBar();
    if (bar) {
      bar->setValue(bar->maximum());
    }
  };

  if (QThread::currentThread() == this->thread()) {
    append_fn();
  } else {
    QMetaObject::invokeMethod(this, append_fn, Qt::QueuedConnection);
  }
}

void TaskExecutionPanel::setBoolLabel(QLabel * label, bool value)
{
  if (!label) {
    return;
  }

  if (value) {
    label->setText(QString::fromUtf8("true"));
    label->setStyleSheet("QLabel { color: green; font-weight: bold; }");
  } else {
    label->setText(QString::fromUtf8("false"));
    label->setStyleSheet("QLabel { color: red; font-weight: bold; }");
  }
}

void TaskExecutionPanel::updateProcessControlDisplay()
{
  const bool compliant_placement_running =
    compliant_placement_process_ &&
    compliant_placement_process_->state() != QProcess::NotRunning;

  if (compliant_placement_process_status_value_) {
    compliant_placement_process_status_value_->setText(
      compliant_placement_running ? QString::fromUtf8("运行中") : QString::fromUtf8("未启动"));
    compliant_placement_process_status_value_->setStyleSheet(
      compliant_placement_running ?
      "QLabel { color: green; font-weight: bold; }" :
      "QLabel { color: gray; font-weight: bold; }");
  }

  if (start_compliant_placement_process_button_) {
    start_compliant_placement_process_button_->setEnabled(!compliant_placement_running);
  }

  if (stop_compliant_placement_process_button_) {
    stop_compliant_placement_process_button_->setEnabled(
      compliant_placement_running || has_cp_state_);
  }
}
void TaskExecutionPanel::startProcess(
  QProcess *& process,
  const QString & program,
  const QStringList & arguments,
  const QString & process_name)
{
  if (process && process->state() != QProcess::NotRunning) {
    appendLog(QString::fromUtf8("%1 已在运行，无需重复启动").arg(process_name));
    updateProcessControlDisplay();
    return;
  }

  if (process) {
    process->deleteLater();
    process = nullptr;
  }

  process = new QProcess(this);
  process->setProcessChannelMode(QProcess::MergedChannels);

  connect(
    process,
    &QProcess::readyReadStandardOutput,
    this,
    [this, process, process_name]() {
      if (!process) {
        return;
      }

      const QByteArray output = process->readAllStandardOutput();
      if (output.isEmpty()) {
        return;
      }

      const QString text = QString::fromLocal8Bit(output);
      const QStringList lines = text.split('\n', Qt::SkipEmptyParts);

      for (const QString & raw_line : lines) {
        const QString line = raw_line.trimmed();
        if (line.isEmpty()) {
          continue;
        }



        appendLog(
          QString::fromUtf8("[%1] %2")
          .arg(process_name)
          .arg(line));
      }
    });

  connect(
    process,
    QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
    this,
    [this, process_name](int exit_code, QProcess::ExitStatus exit_status) {
      appendLog(
        QString::fromUtf8("[%1] 进程已结束，exit_code=%2, exit_status=%3")
        .arg(process_name)
        .arg(exit_code)
        .arg(exit_status == QProcess::NormalExit ? "NormalExit" : "CrashExit"));

      if (process_name == QString::fromUtf8("Compliant Placement Node")) {
        resetCompliantPlacementDisplayState();
      }

      updateProcessControlDisplay();
    });

  process->start(program, arguments);

  if (!process->waitForStarted(3000)) {
    appendLog(QString::fromUtf8("[%1] 启动失败").arg(process_name));
    process->deleteLater();
    process = nullptr;
    updateProcessControlDisplay();
    return;
  }

  appendLog(
    QString::fromUtf8("[%1] 启动成功：%2 %3")
    .arg(process_name)
    .arg(program)
    .arg(arguments.join(" ")));

  updateProcessControlDisplay();
}

void TaskExecutionPanel::stopProcess(
  QProcess *& process,
  const QString & process_name)
{
  if (!process || process->state() == QProcess::NotRunning) {
    appendLog(QString::fromUtf8("%1 当前未运行").arg(process_name));
    updateProcessControlDisplay();
    return;
  }

  appendLog(QString::fromUtf8("[%1] 正在停止进程").arg(process_name));
  process->terminate();

  if (!process->waitForFinished(3000)) {
    appendLog(QString::fromUtf8("[%1] terminate 超时，执行 kill").arg(process_name));
    process->kill();
    process->waitForFinished(2000);
  }

  updateProcessControlDisplay();
}

void TaskExecutionPanel::updateFlowModeDisplay()
{
  if (!current_flow_mode_value_) {
    return;
  }

  switch (current_flow_mode_) {
    case FlowMode::NONE:
      current_flow_mode_value_->setText(QString::fromUtf8("未选择"));
      current_flow_mode_value_->setStyleSheet("QLabel { color: gray; font-weight: bold; }");
      break;
    case FlowMode::LOAD:
      current_flow_mode_value_->setText(QString::fromUtf8("装入流程"));
      current_flow_mode_value_->setStyleSheet("QLabel { color: blue; font-weight: bold; }");
      break;
    case FlowMode::UNLOAD:
      current_flow_mode_value_->setText(QString::fromUtf8("取出流程"));
      current_flow_mode_value_->setStyleSheet("QLabel { color: darkgreen; font-weight: bold; }");
      break;
  }
}

QString TaskExecutionPanel::stepIdToDisplayString(uint32_t step_id) const
{
  switch (step_id) {
    case STEP_IDLE: return "STEP_IDLE";

    case STEP_E0_RESET_IO: return "STEP_E0_RESET_IO";
    case STEP_E1_PRE_GRASP: return "STEP_E1_PRE_GRASP";
    case STEP_E2_ENTER_GRASP: return "STEP_E2_ENTER_GRASP";
    case STEP_E3_GRASP: return "STEP_E3_GRASP";
    case STEP_E4_TRANSIT: return "STEP_E4_TRANSIT";
    case STEP_E5_PRE_INSERT: return "STEP_E5_PRE_INSERT";
    case STEP_E6_PRE_INSERT_ROTATED: return "STEP_E6_PRE_INSERT_ROTATED";
    case STEP_E7_FINAL_INSERT: return "STEP_E7_FINAL_INSERT";
    case STEP_E8_RELEASE: return "STEP_E8_RELEASE";
    case STEP_E9_RETREAT: return "STEP_E9_RETREAT";

    case STEP_D0_RESET_IO: return "STEP_D0_RESET_IO";
    case STEP_D1_PRE_GRASP: return "STEP_D1_PRE_GRASP";
    case STEP_D2_ENTER_GRASP: return "STEP_D2_ENTER_GRASP";
    case STEP_D3_GRASP: return "STEP_D3_GRASP";
    case STEP_D4_PRE_REMOVE_ROTATED: return "STEP_D4_PRE_REMOVE_ROTATED";
    case STEP_D5_REMOVE: return "STEP_D5_REMOVE";
    case STEP_D6_TRANSIT: return "STEP_D6_TRANSIT";
    case STEP_D7_PLACE: return "STEP_D7_PLACE";
    case STEP_D8_RELEASE: return "STEP_D8_RELEASE";
    case STEP_D9_RETREAT: return "STEP_D9_RETREAT";
    
    case STEP_HOME_RETURN: return "STEP_HOME_RETURN";

    default: return QString("UNKNOWN_STEP_%1").arg(step_id);
  }
}

std::string TaskExecutionPanel::frameForStep(uint32_t step_id) const
{
  switch (step_id) {
    // E flow
    case STEP_E1_PRE_GRASP:
      return "box_pre_grasp_tcp";
    case STEP_E2_ENTER_GRASP:
      return "box_grasp_tcp";
    case STEP_E4_TRANSIT:
      return "transit_tcp";
    case STEP_E5_PRE_INSERT:
      return "slot_pre_insert_tcp";
    case STEP_E6_PRE_INSERT_ROTATED:
      return "slot_pre_insert_rotated_tcp";
    case STEP_E7_FINAL_INSERT:
      return "slot_insert_tcp";
    case STEP_E9_RETREAT:
      return "retreat_tcp";

    // D flow
    case STEP_D1_PRE_GRASP:
      return "box_pre_grasp_tcp";
    case STEP_D2_ENTER_GRASP:
      return "box_grasp_tcp";
    case STEP_D4_PRE_REMOVE_ROTATED:
      return "slot_pre_insert_rotated_tcp";
    case STEP_D5_REMOVE:
      return "slot_pre_insert_tcp";
    case STEP_D6_TRANSIT:
      return "transit_tcp";
    case STEP_D7_PLACE:
      return "box_place_tcp";
    case STEP_D9_RETREAT:
      return "retreat_tcp";
      
    case STEP_HOME_RETURN:
      return "home_tcp";

    default:
      return "";
  }
}

void TaskExecutionPanel::setPendingPlan(uint32_t step_id, const QString & frame, bool is_cartesian)
{
  has_pending_plan_ = true;
  pending_step_id_ = step_id;
  pending_frame_ = frame;
  pending_is_cartesian_ = is_cartesian;
  updatePendingPlanDisplay();
  updateButtonStates();
}

void TaskExecutionPanel::clearPendingPlan()
{
  has_pending_plan_ = false;
  pending_step_id_ = 0;
  pending_frame_.clear();
  pending_is_cartesian_ = false;
  updatePendingPlanDisplay();
  updateButtonStates();
}

void TaskExecutionPanel::updatePendingPlanDisplay()
{
  if (!pending_plan_value_) {
    return;
  }

  if (!has_pending_plan_) {
    pending_plan_value_->setText(QString::fromUtf8("无"));
    pending_plan_value_->setStyleSheet("");
    return;
  }

  QString type_text = pending_is_cartesian_ ? QString::fromUtf8("笛卡尔") : QString::fromUtf8("普通");
  QString text = QString("%1 -> %2 [%3]")
    .arg(stepIdToDisplayString(pending_step_id_))
    .arg(pending_frame_)
    .arg(type_text);

  pending_plan_value_->setText(text);
  pending_plan_value_->setStyleSheet("QLabel { color: blue; font-weight: bold; }");
}

void TaskExecutionPanel::setMultiConfigHandoff(uint32_t step_id, const QString & frame)
{
  has_multi_config_handoff_ = true;
  multi_config_handoff_step_id_ = step_id;
  multi_config_handoff_frame_ = frame;
  updateMultiConfigHandoffDisplay();
  updateButtonStates();
}

void TaskExecutionPanel::clearMultiConfigHandoff()
{
  has_multi_config_handoff_ = false;
  multi_config_handoff_step_id_ = 0;
  multi_config_handoff_frame_.clear();
  updateMultiConfigHandoffDisplay();
  updateButtonStates();
}

void TaskExecutionPanel::updateMultiConfigHandoffDisplay()
{
  if (!multi_config_handoff_value_) {
    return;
  }

  if (!has_multi_config_handoff_) {
    multi_config_handoff_value_->setText(QString::fromUtf8("无"));
    multi_config_handoff_value_->setStyleSheet("");
    return;
  }

  const QString text = QString("%1 -> %2")
    .arg(stepIdToDisplayString(multi_config_handoff_step_id_))
    .arg(multi_config_handoff_frame_);

  multi_config_handoff_value_->setText(text);
  multi_config_handoff_value_->setStyleSheet(
    "QLabel { color: darkmagenta; font-weight: bold; }");
}

void TaskExecutionPanel::updateHomeReturnDisplay()
{
  if (!home_return_status_value_) {
    return;
  }

  if (current_phase_ == "DONE" && current_step_id_ == STEP_HOME_RETURN) {
    home_return_status_value_->setText(QString::fromUtf8("已完成"));
    home_return_status_value_->setStyleSheet(
      "QLabel { color: green; font-weight: bold; }");
    return;
  }

  if (current_step_id_ == STEP_HOME_RETURN &&
      exec_substate_ == "HOME_REACHED_WAITING_STEP_CONFIRM") {
    home_return_status_value_->setText(QString::fromUtf8("已到达 home_tcp，等待确认完成"));
    home_return_status_value_->setStyleSheet(
      "QLabel { color: darkcyan; font-weight: bold; }");
    return;
  }

  if ((current_step_id_ == STEP_E9_RETREAT || current_step_id_ == STEP_D9_RETREAT) &&
      exec_substate_ == "RETREAT_STEP_CONFIRMED") {
    home_return_status_value_->setText(QString::fromUtf8("已解锁，可执行 HOME_RETURN"));
    home_return_status_value_->setStyleSheet(
      "QLabel { color: blue; font-weight: bold; }");
    return;
  }

  home_return_status_value_->setText(QString::fromUtf8("未解锁"));
  home_return_status_value_->setStyleSheet("");
}

void TaskExecutionPanel::executePendingPlan()
{
  if (has_multi_config_handoff_) {
    appendLog(QString::fromUtf8(
      "当前步骤正在进行多构型规划处理，不能执行普通 pending 规划"));
    return;
  }

  if (!has_pending_plan_) {
    appendLog(QString::fromUtf8("当前没有待执行规划"));
    return;
  }

  if (pending_is_cartesian_) {
    appendLog(QString::fromUtf8(
      "当前待执行规划为笛卡尔规划，但后端服务暂不支持“只规划后再执行”的笛卡尔缓存执行流程"));
    return;
  }

  if (!node_ || !execute_client_) {
    appendLog(QString::fromUtf8("错误：ROS 节点或执行服务 client 尚未初始化"));
    return;
  }

  if (!execute_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("错误：服务 /cs625/execute_last_plan 不可用"));
    return;
  }

  auto exec_req =
    std::make_shared<cs625_trajectory_tools::srv::ExecuteLastPlan::Request>();

  appendLog(
    QString::fromUtf8("执行待确认规划：%1 -> %2")
    .arg(stepIdToDisplayString(pending_step_id_))
    .arg(pending_frame_));

  execute_client_->async_send_request(
    exec_req,
    [this](
      rclcpp::Client<cs625_trajectory_tools::srv::ExecuteLastPlan>::SharedFuture future_exec)
    {
      try {
        auto exec_resp = future_exec.get();
        if (!exec_resp) {
          appendLog(QString::fromUtf8("执行响应为空"));
          return;
        }

        QString msg = QString::fromStdString(exec_resp->message);
        if (!exec_resp->success) {
          appendLog(QString::fromUtf8("执行失败：%1").arg(msg));
          return;
        }

        const uint32_t step_id_to_trigger = pending_step_id_;
        const QString frame_to_trigger = pending_frame_;

        appendLog(
          QString::fromUtf8("执行成功：%1，准备调用 /trigger_step 推进任务状态")
          .arg(msg));

        clearPendingPlan();

        appendLog(
          QString::fromUtf8("发送任务状态推进：%1 -> %2")
          .arg(stepIdToDisplayString(step_id_to_trigger))
          .arg(frame_to_trigger));

        sendTriggerStepRequest(step_id_to_trigger);

      } catch (const std::exception & e) {
        appendLog(
          QString::fromUtf8("执行服务回调异常：%1")
          .arg(e.what()));
      }
    });
}

void TaskExecutionPanel::runStepWithPlanAndExecute(uint32_t step_id)
{
  if (has_multi_config_handoff_) {
    appendLog(QString::fromUtf8(
      "当前步骤已转入多构型规划处理，请先完成或结束当前多构型处理后，再发起新的普通规划"));
    return;
  }

  if (has_pending_plan_) {
    appendLog(QString::fromUtf8(
      "当前已有待确认执行的规划，请先执行或清除当前规划后，再发起新的规划"));
    return;
  }

  std::string frame = frameForStep(step_id);
  if (frame.empty()) {
    appendLog(
      QString::fromUtf8("步骤 %1 未配置目标 frame，无法规划执行")
      .arg(stepIdToDisplayString(step_id)));
    return;
  }

  if (!node_ || !plan_client_ || !execute_client_) {
    appendLog(QString::fromUtf8("错误：ROS 节点或规划/执行服务 client 尚未初始化"));
    return;
  }

  appendLog(
    QString::fromUtf8("步骤 %1：准备调用规划服务，目标 frame = %2")
    .arg(stepIdToDisplayString(step_id))
    .arg(QString::fromStdString(frame)));

  if (!plan_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("错误：服务 /cs625/plan_to_frame 不可用"));
    return;
  }

  auto plan_req = std::make_shared<cs625_trajectory_tools::srv::PlanToFrame::Request>();
  plan_req->frame_id = frame;

  appendLog(
    QString::fromUtf8("步骤 %1：发送规划请求到 frame: %2")
    .arg(stepIdToDisplayString(step_id))
    .arg(QString::fromStdString(frame)));

  plan_client_->async_send_request(
    plan_req,
    [this, step_id, frame](
      rclcpp::Client<cs625_trajectory_tools::srv::PlanToFrame>::SharedFuture future_plan)
    {
      try {
        auto resp = future_plan.get();
        if (!resp) {
          appendLog(
            QString::fromUtf8("步骤 %1：规划响应为空").arg(stepIdToDisplayString(step_id)));
          return;
        }

        QString msg = QString::fromStdString(resp->message);
        if (!resp->success) {
          appendLog(
            QString::fromUtf8("步骤 %1：规划失败（%2）：%3")
            .arg(stepIdToDisplayString(step_id))
            .arg(QString::fromStdString(frame))
            .arg(msg));
          return;
        }

        appendLog(
          QString::fromUtf8("步骤 %1：规划成功（%2），规划时间 = %3 s")
          .arg(stepIdToDisplayString(step_id))
          .arg(QString::fromStdString(frame))
          .arg(resp->planning_time));

        appendLog(
          QString::fromUtf8("步骤 %1：已缓存本次规划，请在 RViz 中检查轨迹后点击“执行当前规划”")
          .arg(stepIdToDisplayString(step_id)));

        setPendingPlan(step_id, QString::fromStdString(frame), false);

      } catch (const std::exception & e) {
        appendLog(
          QString::fromUtf8("步骤 %1：规划服务回调异常：%2")
          .arg(stepIdToDisplayString(step_id))
          .arg(e.what()));
      }
    });
}

void TaskExecutionPanel::runStepWithCartesianPlanAndExecute(uint32_t step_id)
{
  std::string frame = frameForStep(step_id);

  if (has_multi_config_handoff_) {
    appendLog(QString::fromUtf8(
      "当前步骤已转入多构型规划处理，请先完成或结束当前多构型处理后，再进行新的规划操作"));
    return;
  }

  if (frame.empty()) {
    appendLog(
      QString::fromUtf8("步骤 %1 未配置目标 frame，无法进行笛卡尔规划")
      .arg(stepIdToDisplayString(step_id)));
    return;
  }

  if (!node_ || !cartesian_plan_client_) {
    appendLog(QString::fromUtf8("错误：ROS 节点或笛卡尔规划服务 client 尚未初始化"));
    return;
  }

  appendLog(
    QString::fromUtf8("步骤 %1：准备调用笛卡尔规划服务，目标 frame = %2")
    .arg(stepIdToDisplayString(step_id))
    .arg(QString::fromStdString(frame)));

  if (!cartesian_plan_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("错误：服务 /cs625/cartesian_plan_to_frame 不可用"));
    return;
  }

  auto req =
    std::make_shared<cs625_trajectory_tools::srv::CartesianPlanToFrame::Request>();
  req->frame_id = frame;

  appendLog(
    QString::fromUtf8("步骤 %1：发送笛卡尔规划请求到 frame: %2")
    .arg(stepIdToDisplayString(step_id))
    .arg(QString::fromStdString(frame)));

  cartesian_plan_client_->async_send_request(
    req,
    [this, step_id, frame](
      rclcpp::Client<cs625_trajectory_tools::srv::CartesianPlanToFrame>::SharedFuture future)
    {
      try {
        auto resp = future.get();
        if (!resp) {
          appendLog(QString::fromUtf8("步骤 %1：笛卡尔规划响应为空").arg(stepIdToDisplayString(step_id)));
          return;
        }

        QString msg = QString::fromStdString(resp->message);
        if (!resp->success) {
          appendLog(
            QString::fromUtf8("步骤 %1：笛卡尔规划/执行失败（%2）：%3")
            .arg(stepIdToDisplayString(step_id))
            .arg(QString::fromStdString(frame))
            .arg(msg));
          return;
        }

        appendLog(
          QString::fromUtf8("步骤 %1：笛卡尔规划/执行成功（%2），fraction = %3，调用 /trigger_step 更新任务状态")
          .arg(stepIdToDisplayString(step_id))
          .arg(QString::fromStdString(frame))
          .arg(resp->fraction));

        sendTriggerStepRequest(step_id);

      } catch (const std::exception & e) {
        appendLog(
          QString::fromUtf8("步骤 %1：笛卡尔规划服务回调异常：%2")
          .arg(stepIdToDisplayString(step_id))
          .arg(e.what()));
      }
    });
}

void TaskExecutionPanel::runStepWithForceCompensatedCartesianExecute(uint32_t step_id)
{
  std::string frame = frameForStep(step_id);
  


  if (has_multi_config_handoff_) {
    appendLog(QString::fromUtf8(
      "当前步骤已转入多构型规划处理，请先完成或结束当前多构型处理后，再进行新的规划操作"));
    return;
  }
  
    appendLog(QString::fromUtf8(
    "当前力补偿笛卡尔执行不再依赖柔顺放置 active gate；"
    "cs625_compliant_placement 仅作为 report-only 监测节点。"));

  if (frame.empty()) {
    appendLog(
      QString::fromUtf8("步骤 %1 未配置目标 frame，无法进行力补偿笛卡尔执行")
      .arg(stepIdToDisplayString(step_id)));
    return;
  }

  if (!node_ || !cartesian_force_compensated_client_) {
    appendLog(QString::fromUtf8("错误：ROS 节点或力补偿笛卡尔服务 client 尚未初始化"));
    return;
  }

  appendLog(
    QString::fromUtf8("步骤 %1：准备调用力补偿笛卡尔执行服务，目标 frame = %2")
    .arg(stepIdToDisplayString(step_id))
    .arg(QString::fromStdString(frame)));

  if (!cartesian_force_compensated_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("错误：服务 /cs625/cartesian_force_compensated_to_frame 不可用"));
    return;
  }

  auto req =
    std::make_shared<cs625_trajectory_tools::srv::CartesianForceCompensatedToFrame::Request>();
  req->frame_id = frame;

  appendLog(
    QString::fromUtf8("步骤 %1：发送力补偿笛卡尔执行请求到 frame: %2")
    .arg(stepIdToDisplayString(step_id))
    .arg(QString::fromStdString(frame)));

  cartesian_force_compensated_client_->async_send_request(
    req,
    [this, step_id, frame](
      rclcpp::Client<cs625_trajectory_tools::srv::CartesianForceCompensatedToFrame>::SharedFuture future)
    {
      try {
        auto resp = future.get();
        if (!resp) {
          appendLog(QString::fromUtf8("步骤 %1：力补偿笛卡尔执行响应为空")
            .arg(stepIdToDisplayString(step_id)));
          return;
        }

        QString msg = QString::fromStdString(resp->message);
        if (!resp->success) {
          appendLog(
            QString::fromUtf8("步骤 %1：力补偿笛卡尔执行失败（%2）：%3")
            .arg(stepIdToDisplayString(step_id))
            .arg(QString::fromStdString(frame))
            .arg(msg));
          return;
        }

        appendLog(
          QString::fromUtf8("步骤 %1：力补偿笛卡尔执行成功（%2），completion_ratio=%3, compensation_count=%4, dominant_axis=%5，调用 /trigger_step 更新任务状态")
          .arg(stepIdToDisplayString(step_id))
          .arg(QString::fromStdString(frame))
          .arg(resp->completion_ratio, 0, 'f', 3)
          .arg(resp->compensation_count)
          .arg(QString::fromStdString(resp->dominant_axis)));

        sendTriggerStepRequest(step_id);

      } catch (const std::exception & e) {
        appendLog(
          QString::fromUtf8("步骤 %1：力补偿笛卡尔服务回调异常：%2")
          .arg(stepIdToDisplayString(step_id))
          .arg(e.what()));
      }
    });
}

void TaskExecutionPanel::updateStatusDisplay()
{
  if (current_phase_value_) {
    current_phase_value_->setText(current_phase_);
  }
  if (current_step_value_) {
    current_step_value_->setText(current_step_);
  }
  if (status_value_) {
    status_value_->setText(status_);
  }
  if (exec_substate_value_) {
    exec_substate_value_->setText(exec_substate_);
  }
  if (box_state_value_) {
    box_state_value_->setText(box_state_);
  }
  if (load_state_value_) {
    load_state_value_->setText(load_state_);
  }

  setBoolLabel(reached_box_grasp_value_, reached_box_grasp_);
  setBoolLabel(io_double_low_value_, io_double_low_);
  setBoolLabel(operator_grasp_value_, operator_grasp_confirmed_);

  setBoolLabel(reached_slot_insert_value_, reached_slot_insert_);
  setBoolLabel(io_double_high_value_, io_double_high_);
  setBoolLabel(operator_release_value_, operator_release_confirmed_);
}


void TaskExecutionPanel::updateButtonStates()
{
  const bool has_pending_normal_plan =
    has_pending_plan_ && !pending_is_cartesian_;

  const bool can_handoff_to_multi_config =
    has_pending_normal_plan && !has_multi_config_handoff_;

  const bool can_finish_multi_config =
    has_multi_config_handoff_;

  const bool no_plan_or_handoff_block =
    !has_pending_plan_ && !has_multi_config_handoff_;

  const bool is_load_mode = (current_flow_mode_ == FlowMode::LOAD);
  const bool is_unload_mode = (current_flow_mode_ == FlowMode::UNLOAD);
  

  // ----------------------------
  // 装入流程按钮
  // ----------------------------
  const bool can_e0 =
    is_load_mode &&
    (current_step_id_ == STEP_IDLE || current_step_id_ == STEP_E9_RETREAT);

  const bool can_e1 =
    is_load_mode &&
    (exec_substate_ == "IO_RESET_DONE") &&
    no_plan_or_handoff_block;

  const bool can_e2 =
    is_load_mode &&
    (exec_substate_ == "PRE_GRASP_REACHED") &&
    no_plan_or_handoff_block;

  const bool can_e3 =
    is_load_mode &&
    (exec_substate_ == "GRASP_POSE_REACHED") &&
    !has_multi_config_handoff_;

  const bool can_e4 =
    is_load_mode &&
    no_plan_or_handoff_block &&
    (
      (io_double_low_ &&
       operator_grasp_confirmed_ &&
       load_state_ == "LOADED") ||
      (current_step_id_ == STEP_E4_TRANSIT &&
       exec_substate_ == "TRANSIT_REACHED_WAITING_STEP_CONFIRM")
    );

  const bool can_e5 =
    is_load_mode &&
    no_plan_or_handoff_block &&
    (
      exec_substate_ == "TRANSIT_STEP_CONFIRMED" ||
      (current_step_id_ == STEP_E5_PRE_INSERT &&
       exec_substate_ == "PRE_INSERT_REACHED_WAITING_STEP_CONFIRM")
    );

  const bool can_e6 =
    is_load_mode &&
    no_plan_or_handoff_block &&
    (
      exec_substate_ == "PRE_INSERT_STEP_CONFIRMED" ||
      (current_step_id_ == STEP_E6_PRE_INSERT_ROTATED &&
       exec_substate_ == "PRE_INSERT_ROTATED_REACHED_WAITING_STEP_CONFIRM")
    );

  const bool can_e7 =
    is_load_mode &&
    no_plan_or_handoff_block &&
    (
      exec_substate_ == "PRE_INSERT_ROTATED_STEP_CONFIRMED" ||
      (current_step_id_ == STEP_E7_FINAL_INSERT &&
       exec_substate_ == "INSERT_POSE_REACHED_WAITING_STEP_CONFIRM")
    );

  const bool can_e8 =
    is_load_mode &&
    (exec_substate_ == "INSERT_STEP_CONFIRMED") &&
    no_plan_or_handoff_block;

  const bool can_e9 =
    is_load_mode &&
    no_plan_or_handoff_block &&
    (
      (io_double_high_ &&
       operator_release_confirmed_ &&
       load_state_ == "EMPTY") ||
      (current_step_id_ == STEP_E9_RETREAT &&
       exec_substate_ == "RETREAT_REACHED_WAITING_STEP_CONFIRM")
    );

  // ----------------------------
  // 取出流程按钮
  // 说明：
  // 这里先按装入流程镜像式占位约束处理，
  // 后续需与你在 task_manager 中实际定义的 D 流程 exec_substate 对齐。
  // ----------------------------
  const bool can_d0 =
    is_unload_mode &&
    (current_step_id_ == STEP_IDLE || current_step_id_ == STEP_D9_RETREAT);

  const bool can_d1 =
    is_unload_mode &&
    (exec_substate_ == "IO_RESET_DONE") &&
    no_plan_or_handoff_block;

  const bool can_d2 =
    is_unload_mode &&
    (exec_substate_ == "PRE_GRASP_REACHED") &&
    no_plan_or_handoff_block;

  const bool can_d3 =
    is_unload_mode &&
    (exec_substate_ == "GRASP_POSE_REACHED") &&
    !has_multi_config_handoff_;

  const bool can_d4 =
    is_unload_mode &&
    no_plan_or_handoff_block &&
    (
      (io_double_low_ &&
       operator_grasp_confirmed_ &&
       load_state_ == "LOADED") ||
      (current_step_id_ == STEP_D4_PRE_REMOVE_ROTATED &&
       exec_substate_ == "PRE_REMOVE_ROTATED_REACHED_WAITING_STEP_CONFIRM")
    );

  const bool can_d5 =
    is_unload_mode &&
    no_plan_or_handoff_block &&
    (
      exec_substate_ == "PRE_REMOVE_ROTATED_STEP_CONFIRMED" ||
      (current_step_id_ == STEP_D5_REMOVE &&
       exec_substate_ == "REMOVE_REACHED_WAITING_STEP_CONFIRM")
    );

  const bool can_d6 =
    is_unload_mode &&
    no_plan_or_handoff_block &&
    (
      exec_substate_ == "REMOVE_STEP_CONFIRMED" ||
      (current_step_id_ == STEP_D6_TRANSIT &&
       exec_substate_ == "TRANSIT_REACHED_WAITING_STEP_CONFIRM")
    );

  const bool can_d7 =
    is_unload_mode &&
    no_plan_or_handoff_block &&
    (
      exec_substate_ == "TRANSIT_STEP_CONFIRMED" ||
      (current_step_id_ == STEP_D7_PLACE &&
       exec_substate_ == "PLACE_POSE_REACHED_WAITING_STEP_CONFIRM")
    );

  const bool can_d8 =
    is_unload_mode &&
    no_plan_or_handoff_block &&
    (
      exec_substate_ == "PLACE_STEP_CONFIRMED" ||
      (current_step_id_ == STEP_D8_RELEASE &&
       exec_substate_ == "WAITING_RELEASE_CONFIRM")
    );

  const bool can_d9 =
    is_unload_mode &&
    no_plan_or_handoff_block &&
    (
      (io_double_high_ &&
       operator_release_confirmed_ &&
       load_state_ == "EMPTY") ||
      (current_step_id_ == STEP_D9_RETREAT &&
       exec_substate_ == "RETREAT_REACHED_WAITING_STEP_CONFIRM")
    );

  const bool can_home_return =
    no_plan_or_handoff_block &&
    (
      ((current_step_id_ == STEP_E9_RETREAT ||
        current_step_id_ == STEP_D9_RETREAT) &&
       exec_substate_ == "RETREAT_STEP_CONFIRMED") ||
      (current_step_id_ == STEP_HOME_RETURN &&
       exec_substate_ == "HOME_REACHED_WAITING_STEP_CONFIRM")
    );

  const bool can_confirm_grasp =
    !has_multi_config_handoff_ &&
    (
      (is_load_mode &&
       current_step_id_ == STEP_E3_GRASP &&
       exec_substate_ == "WAITING_GRASP_CONFIRM" &&
       reached_box_grasp_) ||
      (is_unload_mode &&
       current_step_id_ == STEP_D3_GRASP &&
       exec_substate_ == "WAITING_GRASP_CONFIRM" &&
       reached_box_grasp_)
    );

  const bool can_confirm_release =
    !has_multi_config_handoff_ &&
    (
      (is_load_mode &&
       current_step_id_ == STEP_E8_RELEASE &&
       exec_substate_ == "WAITING_RELEASE_CONFIRM" &&
       reached_slot_insert_) ||
      (is_unload_mode &&
       current_step_id_ == STEP_D8_RELEASE &&
       exec_substate_ == "WAITING_RELEASE_CONFIRM")
    );

  const bool can_confirm_step_completion =
    no_plan_or_handoff_block &&
    current_phase_ == "EXECUTION" &&
    (
      exec_substate_ == "TRANSIT_REACHED_WAITING_STEP_CONFIRM" ||
      exec_substate_ == "PRE_INSERT_REACHED_WAITING_STEP_CONFIRM" ||
      exec_substate_ == "PRE_INSERT_ROTATED_REACHED_WAITING_STEP_CONFIRM" ||
      exec_substate_ == "INSERT_POSE_REACHED_WAITING_STEP_CONFIRM" ||
      exec_substate_ == "RETREAT_REACHED_WAITING_STEP_CONFIRM" ||
      exec_substate_ == "PRE_REMOVE_ROTATED_REACHED_WAITING_STEP_CONFIRM" ||
      exec_substate_ == "REMOVE_REACHED_WAITING_STEP_CONFIRM" ||
      exec_substate_ == "PLACE_POSE_REACHED_WAITING_STEP_CONFIRM" ||
      exec_substate_ == "HOME_REACHED_WAITING_STEP_CONFIRM"
    );

  if (start_load_flow_button_) {
    start_load_flow_button_->setEnabled(!has_pending_plan_ && !has_multi_config_handoff_);
  }
  if (start_unload_flow_button_) {
    start_unload_flow_button_->setEnabled(!has_pending_plan_ && !has_multi_config_handoff_);
  }

  if (load_step_group_) {
    load_step_group_->setEnabled(is_load_mode);
  }
  if (unload_step_group_) {
    unload_step_group_->setEnabled(is_unload_mode);
  }

  if (e0_button_) e0_button_->setEnabled(can_e0);
  if (e1_button_) e1_button_->setEnabled(can_e1);
  if (e2_button_) e2_button_->setEnabled(can_e2);
  if (e3_button_) e3_button_->setEnabled(can_e3);
  if (e4_button_) e4_button_->setEnabled(can_e4);
  if (e5_button_) e5_button_->setEnabled(can_e5);
  if (e6_button_) e6_button_->setEnabled(can_e6);
  if (e7_button_) e7_button_->setEnabled(can_e7);
  if (e8_button_) e8_button_->setEnabled(can_e8);
  if (e9_button_) e9_button_->setEnabled(can_e9);

  if (d0_button_) d0_button_->setEnabled(can_d0);
  if (d1_button_) d1_button_->setEnabled(can_d1);
  if (d2_button_) d2_button_->setEnabled(can_d2);
  if (d3_button_) d3_button_->setEnabled(can_d3);
  if (d4_button_) d4_button_->setEnabled(can_d4);
  if (d5_button_) d5_button_->setEnabled(can_d5);
  if (d6_button_) d6_button_->setEnabled(can_d6);
  if (d7_button_) d7_button_->setEnabled(can_d7);
  if (d8_button_) d8_button_->setEnabled(can_d8);
  if (d9_button_) d9_button_->setEnabled(can_d9);

  if (home_return_button_) {
    home_return_button_->setEnabled(can_home_return);
  }

  if (confirm_grasp_button_) confirm_grasp_button_->setEnabled(can_confirm_grasp);
  if (confirm_release_button_) confirm_release_button_->setEnabled(can_confirm_release);
  if (confirm_step_completion_button_) {
    confirm_step_completion_button_->setEnabled(can_confirm_step_completion);
  }

  if (execute_pending_button_) {
    execute_pending_button_->setEnabled(has_pending_plan_ && !has_multi_config_handoff_);
  }
  if (clear_pending_button_) {
    clear_pending_button_->setEnabled(has_pending_plan_ && !has_multi_config_handoff_);
  }

  if (handoff_to_multi_config_button_) {
    handoff_to_multi_config_button_->setEnabled(can_handoff_to_multi_config);
  }
  if (finish_multi_config_button_) {
    finish_multi_config_button_->setEnabled(can_finish_multi_config);
  }

  if (finish_multi_config_and_advance_button_) {
    finish_multi_config_and_advance_button_->setEnabled(can_finish_multi_config);
  }

  if (reset_ui_button_) {
    reset_ui_button_->setEnabled(
      !(current_phase_ == "IDLE" && current_step_id_ == STEP_IDLE && current_flow_mode_ == FlowMode::NONE));
  }
  

  updateProcessControlDisplay();
  
}

void TaskExecutionPanel::taskStateCallback(
  const cs625_task_manager::msg::TaskState::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  current_step_id_ = msg->current_step;
  current_phase_ = QString::fromStdString(msg->current_phase);
  current_step_ = stepIdToDisplayString(msg->current_step);
  status_ = QString::fromStdString(msg->status);
  exec_substate_ = QString::fromStdString(msg->exec_substate);
  box_state_ = QString::fromStdString(msg->box_state);
  load_state_ = QString::fromStdString(msg->load_state);

  reached_box_grasp_ = msg->reached_box_grasp_tcp_pose;
  io_double_low_ = msg->io_double_low_confirmed;
  operator_grasp_confirmed_ = msg->operator_grasp_confirmed;

  reached_slot_insert_ = msg->reached_slot_insert_tcp_pose;
  io_double_high_ = msg->io_double_high_confirmed;
  operator_release_confirmed_ = msg->operator_release_confirmed;

  last_message_ = QString::fromStdString(msg->last_message);

  QMetaObject::invokeMethod(
    this,
    [this]() {
      updateFlowModeDisplay();
      updateStatusDisplay();
      updatePendingPlanDisplay();
      updateMultiConfigHandoffDisplay();
      updateHomeReturnDisplay();
      updateButtonStates();
    },
    Qt::QueuedConnection);
}

void TaskExecutionPanel::sendTriggerStepRequest(uint32_t step_id, const std::string & extra_param)
{
  if (!node_ || !trigger_step_client_) {
    appendLog(QString::fromUtf8("错误：ROS 节点或 trigger_step client 尚未初始化"));
    return;
  }

  if (!trigger_step_client_->wait_for_service(std::chrono::milliseconds(500))) {
    appendLog(QString::fromUtf8("错误：服务 /trigger_step 不可用"));
    return;
  }

  auto request = std::make_shared<cs625_task_manager::srv::TriggerStep::Request>();
  request->step_id = step_id;
  request->extra_param = extra_param;

  appendLog(
    QString::fromUtf8("发送 trigger_step 请求：step_id=%1, extra_param='%2'")
    .arg(stepIdToDisplayString(step_id))
    .arg(QString::fromStdString(extra_param)));

  auto future = trigger_step_client_->async_send_request(
    request,
    [this, step_id](
      rclcpp::Client<cs625_task_manager::srv::TriggerStep>::SharedFuture future_response)
    {
      try {
        auto response = future_response.get();
        if (!response) {
          appendLog(
            QString::fromUtf8("错误：/trigger_step 响应为空，step_id=%1")
            .arg(stepIdToDisplayString(step_id)));
          return;
        }

        appendLog(
          QString::fromUtf8("trigger_step 响应：accepted=%1, success=%2, message=%3")
          .arg(response->accepted ? "true" : "false")
          .arg(response->success ? "true" : "false")
          .arg(QString::fromStdString(response->message)));
      } catch (const std::exception & e) {
        appendLog(
          QString::fromUtf8("异常：调用 /trigger_step 失败：%1")
          .arg(e.what()));
      }
    });
  (void)future;
}

void TaskExecutionPanel::sendConfirmActionRequest(
  const std::string & service_name,
  const std::string & action_name,
  rclcpp::Client<cs625_task_manager::srv::ConfirmAction>::SharedPtr client)
{
  if (!node_ || !client) {
    appendLog(QString::fromUtf8("错误：ROS 节点或确认服务 client 尚未初始化"));
    return;
  }

  if (!client->wait_for_service(std::chrono::milliseconds(500))) {
    appendLog(
      QString::fromUtf8("错误：服务 %1 不可用")
      .arg(QString::fromStdString(service_name)));
    return;
  }

  auto request = std::make_shared<cs625_task_manager::srv::ConfirmAction::Request>();
  request->action_name = action_name;

  appendLog(
    QString::fromUtf8("发送确认请求：service=%1, action_name='%2'")
    .arg(QString::fromStdString(service_name))
    .arg(QString::fromStdString(action_name)));

  auto future = client->async_send_request(
    request,
    [this, service_name](
      rclcpp::Client<cs625_task_manager::srv::ConfirmAction>::SharedFuture future_response)
    {
      try {
        auto response = future_response.get();
        if (!response) {
          appendLog(
            QString::fromUtf8("错误：%1 响应为空")
            .arg(QString::fromStdString(service_name)));
          return;
        }

        appendLog(
          QString::fromUtf8("%1 响应：accepted=%2, success=%3, message=%4")
          .arg(QString::fromStdString(service_name))
          .arg(response->accepted ? "true" : "false")
          .arg(response->success ? "true" : "false")
          .arg(QString::fromStdString(response->message)));
      } catch (const std::exception & e) {
        appendLog(
          QString::fromUtf8("异常：调用 %1 失败：%2")
          .arg(QString::fromStdString(service_name))
          .arg(e.what()));
      }
    });
  (void)future;
}

void TaskExecutionPanel::sendPrepareUnloadFlowRequest()
{
  if (!node_ || !prepare_unload_flow_client_) {
    appendLog(QString::fromUtf8("错误：ROS 节点或 /prepare_unload_flow client 尚未初始化"));
    return;
  }

  if (!prepare_unload_flow_client_->wait_for_service(1s)) {
    appendLog(QString::fromUtf8("错误：服务 /prepare_unload_flow 不可用"));
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  appendLog(QString::fromUtf8(
    "开始调用 /prepare_unload_flow：将基于最近一次 box 精定位结果重建取出流程所需 slot 链"));

  prepare_unload_flow_client_->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future_response)
    {
      try {
        auto response = future_response.get();
        if (!response) {
          appendLog(QString::fromUtf8("错误：/prepare_unload_flow 响应为空"));
          return;
        }

        appendLog(
          QString::fromUtf8("/prepare_unload_flow 响应：success=%1, message=%2")
          .arg(response->success ? "true" : "false")
          .arg(QString::fromStdString(response->message)));

        if (!response->success) {
          appendLog(QString::fromUtf8(
            "启动取出流程失败：请先确认已收到 S3，并且 tcp_server 正在运行"));
          return;
        }

        current_flow_mode_ = FlowMode::UNLOAD;
        clearPendingPlan();
        clearMultiConfigHandoff();

	appendLog(QString::fromUtf8(
  "取出流程准备完成，已切换到“取出流程”模式，可继续执行 D0~D9，并在撤出完成后执行 HOME_RETURN。"));

        updateFlowModeDisplay();
        updateButtonStates();

      } catch (const std::exception & e) {
        appendLog(
          QString::fromUtf8("异常：调用 /prepare_unload_flow 失败：%1")
          .arg(e.what()));
      }
    });
}

void TaskExecutionPanel::onStartLoadFlowClicked()
{
  clearPendingPlan();
  clearMultiConfigHandoff();
  current_flow_mode_ = FlowMode::LOAD;
  updateFlowModeDisplay();
  updateButtonStates();
  appendLog(QString::fromUtf8(
  "已切换到“装入流程”模式，可继续执行 E0~E9，并在撤出完成后执行 HOME_RETURN。"));
}

void TaskExecutionPanel::onStartUnloadFlowClicked()
{
  appendLog(QString::fromUtf8(
    "准备启动取出流程：将先调用 /prepare_unload_flow 恢复 slot 链与相关碰撞跟随目标。"));
  sendPrepareUnloadFlowRequest();
}

void TaskExecutionPanel::onE0Clicked()
{
  sendTriggerStepRequest(STEP_E0_RESET_IO);
}

void TaskExecutionPanel::onE1Clicked()
{
  runStepWithPlanAndExecute(STEP_E1_PRE_GRASP);
}

void TaskExecutionPanel::onE2Clicked()
{
  runStepWithPlanAndExecute(STEP_E2_ENTER_GRASP);
}

void TaskExecutionPanel::onE3Clicked()
{
  sendTriggerStepRequest(STEP_E3_GRASP);
}

void TaskExecutionPanel::onE4Clicked()
{
  appendLog(QString::fromUtf8(
    "E4 将规划到 transit_tcp。"
    "请先在 GoalPlannerPanel 中通过滑条调整并更新 transit_tcp，"
    "然后在这里发起规划，检查轨迹后点击“执行当前规划”。"));
  runStepWithPlanAndExecute(STEP_E4_TRANSIT);
}

void TaskExecutionPanel::onE5Clicked()
{
  runStepWithPlanAndExecute(STEP_E5_PRE_INSERT);
}

void TaskExecutionPanel::onE6Clicked()
{
  appendLog(QString::fromUtf8(
    "E6 将执行力补偿笛卡尔动作。"
    "cs625_compliant_placement 当前仅用于监测与数据显示，不再作为执行 gate。"));
  runStepWithForceCompensatedCartesianExecute(STEP_E6_PRE_INSERT_ROTATED);
}

void TaskExecutionPanel::onE7Clicked()
{
  runStepWithPlanAndExecute(STEP_E7_FINAL_INSERT);
}

void TaskExecutionPanel::onE8Clicked()
{
  sendTriggerStepRequest(STEP_E8_RELEASE);
}

void TaskExecutionPanel::onE9Clicked()
{
  appendLog(QString::fromUtf8(
    "E9 将规划到 retreat_tcp。"
    "请先在 GoalPlannerPanel 中通过滑条调整并更新 retreat_tcp，"
    "然后在这里发起规划，检查轨迹后点击“执行当前规划”。"));
  runStepWithPlanAndExecute(STEP_E9_RETREAT);
}

void TaskExecutionPanel::onD0Clicked()
{
  sendTriggerStepRequest(STEP_D0_RESET_IO);
}

void TaskExecutionPanel::onD1Clicked()
{
  runStepWithPlanAndExecute(STEP_D1_PRE_GRASP);
}

void TaskExecutionPanel::onD2Clicked()
{
  runStepWithPlanAndExecute(STEP_D2_ENTER_GRASP);
}

void TaskExecutionPanel::onD3Clicked()
{
  sendTriggerStepRequest(STEP_D3_GRASP);
}

void TaskExecutionPanel::onD4Clicked()
{
  runStepWithPlanAndExecute(STEP_D4_PRE_REMOVE_ROTATED);
}

void TaskExecutionPanel::onD5Clicked()
{
  appendLog(QString::fromUtf8(
    "D5 将执行力补偿笛卡尔动作。"
    "cs625_compliant_placement 当前仅用于监测与数据显示，不再作为执行 gate。"));
  runStepWithForceCompensatedCartesianExecute(STEP_D5_REMOVE);
}

void TaskExecutionPanel::onD6Clicked()
{
  appendLog(QString::fromUtf8(
    "D6 将规划到 transit_tcp。"
    "请先在 GoalPlannerPanel 中通过滑条调整并更新 transit_tcp，"
    "然后在这里发起规划，检查轨迹后点击“执行当前规划”。"));
  runStepWithPlanAndExecute(STEP_D6_TRANSIT);
}

void TaskExecutionPanel::onD7Clicked()
{
  appendLog(QString::fromUtf8(
    "D7 将规划到 box_place_tcp。"
    "请先确认 GoalPlannerPanel / MultiConfigPlanningPanel 中已接入 box_place_tcp，"
    "并确保其默认位姿或手动设置结果已正确。"));
  runStepWithPlanAndExecute(STEP_D7_PLACE);
}

void TaskExecutionPanel::onD8Clicked()
{
  sendTriggerStepRequest(STEP_D8_RELEASE);
}

void TaskExecutionPanel::onD9Clicked()
{
  appendLog(QString::fromUtf8(
    "D9 将规划到 retreat_tcp。"
    "请先在 GoalPlannerPanel 中通过滑条调整并更新 retreat_tcp，"
    "然后在这里发起规划，检查轨迹后点击“执行当前规划”。"));
  runStepWithPlanAndExecute(STEP_D9_RETREAT);
}

void TaskExecutionPanel::onHomeReturnClicked()
{
  appendLog(QString::fromUtf8(
    "HOME_RETURN 将规划到 home_tcp。"
    "请在 RViz 中检查轨迹后点击“执行当前规划”。"));
  runStepWithPlanAndExecute(STEP_HOME_RETURN);
}

void TaskExecutionPanel::onConfirmGraspClicked()
{
  sendConfirmActionRequest("/confirm_grasp", "confirm_grasp", confirm_grasp_client_);
}

void TaskExecutionPanel::onConfirmReleaseClicked()
{
  sendConfirmActionRequest("/confirm_release", "confirm_release", confirm_release_client_);
}

void TaskExecutionPanel::onConfirmStepCompletionClicked()
{
  appendLog(QString::fromUtf8("发送“确认本步完成并解锁下一步”请求"));
  sendConfirmActionRequest(
    "/confirm_step_completion",
    "confirm_step_completion",
    confirm_step_completion_client_);
}

void TaskExecutionPanel::onResetUiClicked()
{
  if (current_phase_ == "IDLE" && current_step_id_ == STEP_IDLE && current_flow_mode_ == FlowMode::NONE) {
    appendLog(QString::fromUtf8("当前已经处于 STEP_IDLE / IDLE / 未选择流程，无需重置"));
    return;
  }

  appendLog(QString::fromUtf8("发送重置请求：切换到 STEP_IDLE，并清空当前流程选择"));
  clearPendingPlan();
  clearMultiConfigHandoff();
  current_flow_mode_ = FlowMode::NONE;
  updateFlowModeDisplay();
  updateButtonStates();
  sendTriggerStepRequest(STEP_IDLE);
}


void TaskExecutionPanel::onStartCompliantPlacementProcessClicked()
{
  const bool already_running =
    compliant_placement_process_ &&
    compliant_placement_process_->state() != QProcess::NotRunning;

  if (!already_running) {
    startProcess(
      compliant_placement_process_,
      "ros2",
      QStringList() << "launch" << "cs625_compliant_placement" << "compliant_placement.launch.py",
      QString::fromUtf8("Compliant Placement Node"));
    appendLog(QString::fromUtf8(
      "已启动 cs625_compliant_placement（report-only 模式，仅做监测与数据显示）。"));
  } else {
    appendLog(QString::fromUtf8(
      "Compliant Placement Node 已在运行（report-only 模式，无需重复启动）。"));
  }

  updateProcessControlDisplay();
  updateButtonStates();
}

void TaskExecutionPanel::onStopCompliantPlacementProcessClicked()
{
  stopProcess(compliant_placement_process_, QString::fromUtf8("Compliant Placement Node"));
  resetCompliantPlacementDisplayState();
  updateProcessControlDisplay();
  updateButtonStates();
}

void TaskExecutionPanel::setCompliantPlacementStateFromMsg(
  const cs625_compliant_placement::msg::CompliantPlacementState & msg)
{
  has_cp_state_ = true;
  cp_state_ = QString::fromStdString(msg.state);
  cp_active_ = msg.active;
  cp_contact_detected_ = msg.contact_detected;
  cp_jam_detected_ = msg.jam_detected;
  cp_target_reached_ = msg.target_reached;
  cp_aborted_ = msg.aborted;
  cp_stop_reason_ = QString::fromStdString(msg.stop_reason);
  cp_tau_metric_ = msg.tau_metric;
  cp_filtered_tau_metric_ = msg.filtered_tau_metric;

}



void TaskExecutionPanel::updateCompliantPlacementDisplay()
{
  if (cp_state_value_) {
    cp_state_value_->setText(cp_state_);
  }

  setBoolLabel(cp_active_value_, cp_active_);
  setBoolLabel(cp_contact_detected_value_, cp_contact_detected_);
  setBoolLabel(cp_jam_detected_value_, cp_jam_detected_);
  setBoolLabel(cp_target_reached_value_, cp_target_reached_);
  setBoolLabel(cp_aborted_value_, cp_aborted_);

  if (cp_stop_reason_value_) {
    cp_stop_reason_value_->setText(cp_stop_reason_);
  }

  if (cp_tau_metric_value_) {
    cp_tau_metric_value_->setText(QString::number(cp_tau_metric_, 'f', 4));
  }

  if (cp_filtered_tau_metric_value_) {
    cp_filtered_tau_metric_value_->setText(QString::number(cp_filtered_tau_metric_, 'f', 4));
  }


}

void TaskExecutionPanel::resetCompliantPlacementDisplayState()
{
  has_cp_state_ = false;

  cp_state_ = QString::fromUtf8("UNKNOWN");
  cp_active_ = false;
  cp_contact_detected_ = false;
  cp_jam_detected_ = false;
  cp_target_reached_ = false;
  cp_aborted_ = false;
  cp_stop_reason_ = QString::fromUtf8("N/A");
  cp_tau_metric_ = 0.0;
  cp_filtered_tau_metric_ = 0.0;

  updateCompliantPlacementDisplay();
}

void TaskExecutionPanel::updateTcpForceDisplay()
{
  if (tcp_force_x_value_) {
    tcp_force_x_value_->setText(QString::number(tcp_force_x_, 'f', 6));
  }

  if (tcp_force_y_value_) {
    tcp_force_y_value_->setText(QString::number(tcp_force_y_, 'f', 6));
  }

  if (tcp_force_z_value_) {
    tcp_force_z_value_->setText(QString::number(tcp_force_z_, 'f', 6));
  }

  if (tcp_torque_x_value_) {
    tcp_torque_x_value_->setText(QString::number(tcp_torque_x_, 'f', 6));
  }

  if (tcp_torque_y_value_) {
    tcp_torque_y_value_->setText(QString::number(tcp_torque_y_, 'f', 6));
  }

  if (tcp_torque_z_value_) {
    tcp_torque_z_value_->setText(QString::number(tcp_torque_z_, 'f', 6));
  }
}

void TaskExecutionPanel::compliantPlacementStateCallback(
  const cs625_compliant_placement::msg::CompliantPlacementState::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  setCompliantPlacementStateFromMsg(*msg);

  QMetaObject::invokeMethod(
    this,
    [this]() {
      updateCompliantPlacementDisplay();
      updateButtonStates();
    },
    Qt::QueuedConnection);
}

void TaskExecutionPanel::tcpForceCallback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  if (msg->data.size() < 6) {
    return;
  }

  has_tcp_force_ = true;
  tcp_force_x_ = msg->data[0];
  tcp_force_y_ = msg->data[1];
  tcp_force_z_ = msg->data[2];
  tcp_torque_x_ = msg->data[3];
  tcp_torque_y_ = msg->data[4];
  tcp_torque_z_ = msg->data[5];

  QMetaObject::invokeMethod(
    this,
    [this]() {
      updateTcpForceDisplay();
    },
    Qt::QueuedConnection);
}


void TaskExecutionPanel::onExecutePendingClicked()
{
  executePendingPlan();
}

void TaskExecutionPanel::onClearPendingClicked()
{
  if (!has_pending_plan_) {
    appendLog(QString::fromUtf8("当前没有待清除的规划"));
    return;
  }

  appendLog(
    QString::fromUtf8("已清除待执行规划：%1 -> %2")
    .arg(stepIdToDisplayString(pending_step_id_))
    .arg(pending_frame_));

  clearPendingPlan();
}

void TaskExecutionPanel::onHandoffToMultiConfigClicked()
{
  if (has_multi_config_handoff_) {
    appendLog(QString::fromUtf8("当前已经处于多构型规划处理状态"));
    return;
  }

  if (!has_pending_plan_) {
    appendLog(QString::fromUtf8(
      "当前没有待处理的普通规划。请先发起普通规划，并确认目标步骤后，再转入多构型规划"));
    return;
  }

  if (pending_is_cartesian_) {
    appendLog(QString::fromUtf8(
      "当前待处理规划为笛卡尔规划，暂不支持转入多构型规划"));
    return;
  }

  const uint32_t step_id = pending_step_id_;
  const QString frame = pending_frame_;

  appendLog(QString::fromUtf8(
    "检测到当前普通规划可能不满足实际需求，已转入多构型规划处理：%1 -> %2")
    .arg(stepIdToDisplayString(step_id))
    .arg(frame));

  appendLog(QString::fromUtf8(
    "已清除当前普通 pending 规划，避免与多构型处理流程冲突。"));

  clearPendingPlan();
  setMultiConfigHandoff(step_id, frame);

  appendLog(QString::fromUtf8("请切换到 MultiConfigPlanningPanel，并按以下步骤操作："));
  appendLog(QString::fromUtf8("1. 选择目标 %1").arg(frame));
  appendLog(QString::fromUtf8("2. 点击“读取目标 Pose”"));
  appendLog(QString::fromUtf8("3. 点击“求全部 IK 解”"));
  appendLog(QString::fromUtf8("4. 选择合适构型并点击“预览选中解”"));
  appendLog(QString::fromUtf8("5. 点击“保存为 selected_multi_config_target”"));
  appendLog(QString::fromUtf8("6. 点击“预览路径（PlanToTarget，不执行）”检查结果"));
  appendLog(QString::fromUtf8("7. 如结果合适，在 MultiConfigPlanningPanel 中执行最近规划"));
  appendLog(QString::fromUtf8(
    "完成后请返回 TaskExecutionPanel："
    "如果只想结束当前引导状态，请点击“我已完成多构型处理”；"
    "如果已在 MultiConfigPlanningPanel 中完成真实执行并希望推进任务，请点击“多构型已执行并推进任务”。"));
}

void TaskExecutionPanel::onFinishMultiConfigClicked()
{
  if (!has_multi_config_handoff_) {
    appendLog(QString::fromUtf8("当前没有进行中的多构型处理"));
    return;
  }

  appendLog(QString::fromUtf8(
    "多构型规划处理已标记完成：%1 -> %2")
    .arg(stepIdToDisplayString(multi_config_handoff_step_id_))
    .arg(multi_config_handoff_frame_));

  appendLog(QString::fromUtf8(
    "注意：此操作只会结束引导状态，不会自动推进任务步骤。"
    "请确认 MultiConfigPlanningPanel 中的规划与执行已完成，再根据当前任务状态继续操作。"));

  clearMultiConfigHandoff();
}

void TaskExecutionPanel::onFinishMultiConfigAndAdvanceClicked()
{
  if (!has_multi_config_handoff_) {
    appendLog(QString::fromUtf8("当前没有进行中的多构型处理，无法推进任务"));
    return;
  }

  const uint32_t step_id = multi_config_handoff_step_id_;
  const QString frame = multi_config_handoff_frame_;

  appendLog(QString::fromUtf8(
    "多构型规划处理已确认执行完成：%1 -> %2，准备推进任务状态")
    .arg(stepIdToDisplayString(step_id))
    .arg(frame));

  clearMultiConfigHandoff();

  appendLog(QString::fromUtf8(
    "发送任务状态推进（多构型完成）：%1 -> %2")
    .arg(stepIdToDisplayString(step_id))
    .arg(frame));

  sendTriggerStepRequest(step_id);
}

void TaskExecutionPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void TaskExecutionPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

}  // namespace elite_dashboard_rviz_plugin

PLUGINLIB_EXPORT_CLASS(
  elite_dashboard_rviz_plugin::TaskExecutionPanel,
  rviz_common::Panel)
