#pragma once

#include <memory>
#include <string>

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QPlainTextEdit>
#include <QGroupBox>
#include <QProcess>

#include "cs625_compliant_placement/msg/compliant_placement_state.hpp"

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>

#include "std_srvs/srv/trigger.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

#include "cs625_task_manager/msg/task_state.hpp"
#include "cs625_task_manager/srv/trigger_step.hpp"
#include "cs625_task_manager/srv/confirm_action.hpp"

#include "cs625_trajectory_tools/srv/plan_to_frame.hpp"
#include "cs625_trajectory_tools/srv/execute_last_plan.hpp"
#include "cs625_trajectory_tools/srv/cartesian_plan_to_frame.hpp"
#include "cs625_trajectory_tools/srv/cartesian_force_compensated_to_frame.hpp"

namespace elite_dashboard_rviz_plugin
{

class TaskExecutionPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TaskExecutionPanel(QWidget * parent = nullptr);
  ~TaskExecutionPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onStartLoadFlowClicked();
  void onStartUnloadFlowClicked();

  void onE0Clicked();
  void onE1Clicked();
  void onE2Clicked();
  void onE3Clicked();
  void onE4Clicked();
  void onE5Clicked();
  void onE6Clicked();
  void onE7Clicked();
  void onE8Clicked();
  void onE9Clicked();

  void onD0Clicked();
  void onD1Clicked();
  void onD2Clicked();
  void onD3Clicked();
  void onD4Clicked();
  void onD5Clicked();
  void onD6Clicked();
  void onD7Clicked();
  void onD8Clicked();
  void onD9Clicked();

  void onHomeReturnClicked();

  void onConfirmGraspClicked();
  void onConfirmReleaseClicked();
  void onConfirmStepCompletionClicked();
  void onResetUiClicked();

  void onStartCompliantPlacementProcessClicked();
  void onStopCompliantPlacementProcessClicked();

  void onExecutePendingClicked();
  void onClearPendingClicked();

  void onHandoffToMultiConfigClicked();
  void onFinishMultiConfigClicked();
  void onFinishMultiConfigAndAdvanceClicked();

private:
  enum class FlowMode
  {
    NONE = 0,
    LOAD,
    UNLOAD
  };

  static constexpr uint32_t STEP_IDLE = 0;

  static constexpr uint32_t STEP_E0_RESET_IO = 1;
  static constexpr uint32_t STEP_E1_PRE_GRASP = 2;
  static constexpr uint32_t STEP_E2_ENTER_GRASP = 3;
  static constexpr uint32_t STEP_E3_GRASP = 4;
  static constexpr uint32_t STEP_E4_TRANSIT = 5;
  static constexpr uint32_t STEP_E5_PRE_INSERT = 6;
  static constexpr uint32_t STEP_E6_PRE_INSERT_ROTATED = 7;
  static constexpr uint32_t STEP_E7_FINAL_INSERT = 8;
  static constexpr uint32_t STEP_E8_RELEASE = 9;
  static constexpr uint32_t STEP_E9_RETREAT = 10;

  static constexpr uint32_t STEP_D0_RESET_IO = 101;
  static constexpr uint32_t STEP_D1_PRE_GRASP = 102;
  static constexpr uint32_t STEP_D2_ENTER_GRASP = 103;
  static constexpr uint32_t STEP_D3_GRASP = 104;
  static constexpr uint32_t STEP_D4_PRE_REMOVE_ROTATED = 105;
  static constexpr uint32_t STEP_D5_REMOVE = 106;
  static constexpr uint32_t STEP_D6_TRANSIT = 107;
  static constexpr uint32_t STEP_D7_PLACE = 108;
  static constexpr uint32_t STEP_D8_RELEASE = 109;
  static constexpr uint32_t STEP_D9_RETREAT = 110;

  static constexpr uint32_t STEP_HOME_RETURN = 1000;

  void initUI();
  void initRosNode();
  void appendLog(const QString & text);

  void updateStatusDisplay();
  void updateButtonStates();
  void updateFlowModeDisplay();
  void updateHomeReturnDisplay();

  void setBoolLabel(QLabel * label, bool value);

  void updateProcessControlDisplay();
  void resetCompliantPlacementDisplayState();

  void startProcess(
    QProcess *& process,
    const QString & program,
    const QStringList & arguments,
    const QString & process_name);

  void stopProcess(
    QProcess *& process,
    const QString & process_name);

  void sendTriggerStepRequest(uint32_t step_id, const std::string & extra_param = "");
  void sendConfirmActionRequest(
    const std::string & service_name,
    const std::string & action_name,
    rclcpp::Client<cs625_task_manager::srv::ConfirmAction>::SharedPtr client);

  void sendPrepareUnloadFlowRequest();

  QString stepIdToDisplayString(uint32_t step_id) const;
  std::string frameForStep(uint32_t step_id) const;

  void runStepWithPlanAndExecute(uint32_t step_id);
  void runStepWithCartesianPlanAndExecute(uint32_t step_id);
  void runStepWithForceCompensatedCartesianExecute(uint32_t step_id);

  void setPendingPlan(uint32_t step_id, const QString & frame, bool is_cartesian);
  void clearPendingPlan();
  void updatePendingPlanDisplay();
  void executePendingPlan();

  void setMultiConfigHandoff(uint32_t step_id, const QString & frame);
  void clearMultiConfigHandoff();
  void updateMultiConfigHandoffDisplay();

  void taskStateCallback(const cs625_task_manager::msg::TaskState::SharedPtr msg);

  void compliantPlacementStateCallback(
    const cs625_compliant_placement::msg::CompliantPlacementState::SharedPtr msg);

  void tcpForceCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  void updateCompliantPlacementDisplay();
  void updateTcpForceDisplay();

  void setCompliantPlacementStateFromMsg(
    const cs625_compliant_placement::msg::CompliantPlacementState & msg);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<cs625_task_manager::msg::TaskState>::SharedPtr task_state_sub_;

  rclcpp::Client<cs625_task_manager::srv::TriggerStep>::SharedPtr trigger_step_client_;
  rclcpp::Client<cs625_task_manager::srv::ConfirmAction>::SharedPtr confirm_grasp_client_;
  rclcpp::Client<cs625_task_manager::srv::ConfirmAction>::SharedPtr confirm_release_client_;
  rclcpp::Client<cs625_task_manager::srv::ConfirmAction>::SharedPtr
    confirm_step_completion_client_;

  rclcpp::Client<cs625_trajectory_tools::srv::PlanToFrame>::SharedPtr plan_client_;
  rclcpp::Client<cs625_trajectory_tools::srv::ExecuteLastPlan>::SharedPtr execute_client_;
  rclcpp::Client<cs625_trajectory_tools::srv::CartesianPlanToFrame>::SharedPtr
    cartesian_plan_client_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr prepare_unload_flow_client_;

  rclcpp::Subscription<cs625_compliant_placement::msg::CompliantPlacementState>::SharedPtr
    compliant_placement_state_sub_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tcp_force_sub_;

  rclcpp::Client<cs625_trajectory_tools::srv::CartesianForceCompensatedToFrame>::SharedPtr
    cartesian_force_compensated_client_;

  QGroupBox * load_step_group_{nullptr};
  QGroupBox * unload_step_group_{nullptr};

  QPushButton * start_load_flow_button_{nullptr};
  QPushButton * start_unload_flow_button_{nullptr};

  QPushButton * e0_button_{nullptr};
  QPushButton * e1_button_{nullptr};
  QPushButton * e2_button_{nullptr};
  QPushButton * e3_button_{nullptr};
  QPushButton * e4_button_{nullptr};
  QPushButton * e5_button_{nullptr};
  QPushButton * e6_button_{nullptr};
  QPushButton * e7_button_{nullptr};
  QPushButton * e8_button_{nullptr};
  QPushButton * e9_button_{nullptr};

  QPushButton * d0_button_{nullptr};
  QPushButton * d1_button_{nullptr};
  QPushButton * d2_button_{nullptr};
  QPushButton * d3_button_{nullptr};
  QPushButton * d4_button_{nullptr};
  QPushButton * d5_button_{nullptr};
  QPushButton * d6_button_{nullptr};
  QPushButton * d7_button_{nullptr};
  QPushButton * d8_button_{nullptr};
  QPushButton * d9_button_{nullptr};

  QGroupBox * home_return_group_{nullptr};
  QPushButton * home_return_button_{nullptr};
  QLabel * home_return_status_value_{nullptr};

  QPushButton * confirm_grasp_button_{nullptr};
  QPushButton * confirm_release_button_{nullptr};
  QPushButton * confirm_step_completion_button_{nullptr};
  QPushButton * reset_ui_button_{nullptr};

  QPushButton * execute_pending_button_{nullptr};
  QPushButton * clear_pending_button_{nullptr};
  QPushButton * handoff_to_multi_config_button_{nullptr};
  QPushButton * finish_multi_config_button_{nullptr};
  QPushButton * finish_multi_config_and_advance_button_{nullptr};

  QLabel * current_flow_mode_value_{nullptr};
  QLabel * current_phase_value_{nullptr};
  QLabel * current_step_value_{nullptr};
  QLabel * status_value_{nullptr};
  QLabel * exec_substate_value_{nullptr};
  QLabel * box_state_value_{nullptr};
  QLabel * load_state_value_{nullptr};

  QLabel * reached_box_grasp_value_{nullptr};
  QLabel * io_double_low_value_{nullptr};
  QLabel * operator_grasp_value_{nullptr};

  QLabel * reached_slot_insert_value_{nullptr};
  QLabel * io_double_high_value_{nullptr};
  QLabel * operator_release_value_{nullptr};

  QLabel * pending_plan_value_{nullptr};
  QLabel * multi_config_handoff_value_{nullptr};

  QGroupBox * process_control_group_{nullptr};

  QPushButton * start_compliant_placement_process_button_{nullptr};
  QPushButton * stop_compliant_placement_process_button_{nullptr};
  QLabel * compliant_placement_process_status_value_{nullptr};

  QLabel * cp_state_value_{nullptr};
  QLabel * cp_active_value_{nullptr};
  QLabel * cp_contact_detected_value_{nullptr};
  QLabel * cp_jam_detected_value_{nullptr};
  QLabel * cp_target_reached_value_{nullptr};
  QLabel * cp_aborted_value_{nullptr};
  QLabel * cp_stop_reason_value_{nullptr};
  QLabel * cp_tau_metric_value_{nullptr};
  QLabel * cp_filtered_tau_metric_value_{nullptr};

  QLabel * tcp_force_x_value_{nullptr};
  QLabel * tcp_force_y_value_{nullptr};
  QLabel * tcp_force_z_value_{nullptr};
  QLabel * tcp_torque_x_value_{nullptr};
  QLabel * tcp_torque_y_value_{nullptr};
  QLabel * tcp_torque_z_value_{nullptr};

  QProcess * compliant_placement_process_{nullptr};

  QPlainTextEdit * log_text_{nullptr};

  FlowMode current_flow_mode_{FlowMode::NONE};

  uint32_t current_step_id_{0};
  QString current_phase_{"IDLE"};
  QString current_step_{"STEP_IDLE"};
  QString status_{"IDLE"};
  QString exec_substate_{"WAITING_TRIGGER"};
  QString box_state_{"NOT_GRASPED"};
  QString load_state_{"EMPTY"};

  bool reached_box_grasp_{false};
  bool io_double_low_{false};
  bool operator_grasp_confirmed_{false};

  bool reached_slot_insert_{false};
  bool io_double_high_{false};
  bool operator_release_confirmed_{false};

  QString last_message_{"No task_state received yet"};

  bool has_pending_plan_{false};
  uint32_t pending_step_id_{0};
  QString pending_frame_;
  bool pending_is_cartesian_{false};

  bool has_multi_config_handoff_{false};
  uint32_t multi_config_handoff_step_id_{0};
  QString multi_config_handoff_frame_;

  bool has_cp_state_{false};

  QString cp_state_{"UNKNOWN"};
  bool cp_active_{false};
  bool cp_contact_detected_{false};
  bool cp_jam_detected_{false};
  bool cp_target_reached_{false};
  bool cp_aborted_{false};
  QString cp_stop_reason_{"N/A"};
  double cp_tau_metric_{0.0};
  double cp_filtered_tau_metric_{0.0};

  bool has_tcp_force_{false};
  double tcp_force_x_{0.0};
  double tcp_force_y_{0.0};
  double tcp_force_z_{0.0};
  double tcp_torque_x_{0.0};
  double tcp_torque_y_{0.0};
  double tcp_torque_z_{0.0};
};

}  // namespace elite_dashboard_rviz_plugin
