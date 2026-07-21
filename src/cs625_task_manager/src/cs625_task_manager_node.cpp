#include "cs625_task_manager/task_manager_node.hpp"

#include <chrono>
#include <functional>
#include <future>
#include <utility>

#include "cs625_task_manager/task_definitions.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace cs625_task_manager
{

TaskManagerNode::TaskManagerNode()
: Node("cs625_task_manager_node"),
  last_io_state_valid_(false),
  last_tool_0_(false),
  last_tool_1_(false),
  last_io_double_low_confirmed_(false),
  last_io_double_high_confirmed_(false)
{
  task_state_pub_ = this->create_publisher<cs625_task_manager::msg::TaskState>(
    "task_state", 10);

  grasp_state_event_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/cs625/grasp_state_event", 10);

  trigger_step_srv_ = this->create_service<cs625_task_manager::srv::TriggerStep>(
    "trigger_step",
    std::bind(
      &TaskManagerNode::handle_trigger_step,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  confirm_grasp_srv_ = this->create_service<cs625_task_manager::srv::ConfirmAction>(
    "confirm_grasp",
    std::bind(
      &TaskManagerNode::handle_confirm_grasp,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  confirm_release_srv_ = this->create_service<cs625_task_manager::srv::ConfirmAction>(
    "confirm_release",
    std::bind(
      &TaskManagerNode::handle_confirm_release,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  confirm_step_completion_srv_ =
    this->create_service<cs625_task_manager::srv::ConfirmAction>(
      "confirm_step_completion",
      std::bind(
        &TaskManagerNode::handle_confirm_step_completion,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

  io_states_sub_ = this->create_subscription<eli_common_interface::msg::IOState>(
    "/io_and_status_controller/io_states",
    10,
    std::bind(&TaskManagerNode::handle_io_states, this, std::placeholders::_1));

  set_io_client_ = this->create_client<eli_common_interface::srv::SetIO>(
    "/io_and_status_controller/set_io");

  initialize_state();

  publish_timer_ = this->create_wall_timer(
    500ms,
    std::bind(&TaskManagerNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "cs625_task_manager_node started.");
}

void TaskManagerNode::initialize_state()
{
  current_state_.current_step = constants::STEP_IDLE;
  current_state_.current_phase = constants::PHASE_IDLE;
  current_state_.status = constants::STATUS_IDLE;
  current_state_.exec_substate = constants::EXEC_SUBSTATE_WAITING_TRIGGER;
  current_state_.box_state = constants::BOX_STATE_NOT_GRASPED;
  current_state_.load_state = constants::LOAD_STATE_EMPTY;

  reset_execution_flags();

  current_state_.last_message = "Task manager initialized";
}

void TaskManagerNode::reset_execution_flags()
{
  current_state_.reached_box_grasp_tcp_pose = false;
  current_state_.io_double_low_confirmed = false;
  current_state_.operator_grasp_confirmed = false;
  current_state_.grasp_confirmed = false;

  current_state_.reached_slot_insert_tcp_pose = false;
  current_state_.io_double_high_confirmed = false;
  current_state_.operator_release_confirmed = false;
  current_state_.release_confirmed = false;
}

void TaskManagerNode::publish_state()
{
  task_state_pub_->publish(current_state_);
}

void TaskManagerNode::publish_grasp_state_event(const std::string & event_name)
{
  if (!grasp_state_event_pub_) {
    RCLCPP_WARN(this->get_logger(), "grasp_state_event publisher is not initialized");
    return;
  }

  std_msgs::msg::String msg;
  msg.data = event_name;
  grasp_state_event_pub_->publish(msg);

  RCLCPP_INFO(
    this->get_logger(),
    "Published grasp state event: %s",
    event_name.c_str());
}

void TaskManagerNode::timer_callback()
{
  publish_state();
}

bool TaskManagerNode::is_step_supported(uint32_t step_id) const
{
  return
    step_id == constants::STEP_IDLE ||

    step_id == constants::STEP_E0_RESET_IO ||
    step_id == constants::STEP_E1_PRE_GRASP ||
    step_id == constants::STEP_E2_ENTER_GRASP ||
    step_id == constants::STEP_E3_GRASP ||
    step_id == constants::STEP_E4_TRANSIT ||
    step_id == constants::STEP_E5_PRE_INSERT ||
    step_id == constants::STEP_E6_PRE_INSERT_ROTATED ||
    step_id == constants::STEP_E7_FINAL_INSERT ||
    step_id == constants::STEP_E8_RELEASE ||
    step_id == constants::STEP_E9_RETREAT ||

    step_id == constants::STEP_D0_RESET_IO ||
    step_id == constants::STEP_D1_PRE_GRASP ||
    step_id == constants::STEP_D2_ENTER_GRASP ||
    step_id == constants::STEP_D3_GRASP ||
    step_id == constants::STEP_D4_PRE_REMOVE_ROTATED ||
    step_id == constants::STEP_D5_REMOVE ||
    step_id == constants::STEP_D6_TRANSIT ||
    step_id == constants::STEP_D7_PLACE ||
    step_id == constants::STEP_D8_RELEASE ||
    step_id == constants::STEP_D9_RETREAT ||
    
    step_id == constants::STEP_HOME_RETURN;
}

std::string TaskManagerNode::step_to_string(uint32_t step_id) const
{
  if (step_id == constants::STEP_IDLE) {
    return "STEP_IDLE";
  }

  if (step_id == constants::STEP_E0_RESET_IO) {
    return "STEP_E0_RESET_IO";
  }
  if (step_id == constants::STEP_E1_PRE_GRASP) {
    return "STEP_E1_PRE_GRASP";
  }
  if (step_id == constants::STEP_E2_ENTER_GRASP) {
    return "STEP_E2_ENTER_GRASP";
  }
  if (step_id == constants::STEP_E3_GRASP) {
    return "STEP_E3_GRASP";
  }
  if (step_id == constants::STEP_E4_TRANSIT) {
    return "STEP_E4_TRANSIT";
  }
  if (step_id == constants::STEP_E5_PRE_INSERT) {
    return "STEP_E5_PRE_INSERT";
  }
  if (step_id == constants::STEP_E6_PRE_INSERT_ROTATED) {
    return "STEP_E6_PRE_INSERT_ROTATED";
  }
  if (step_id == constants::STEP_E7_FINAL_INSERT) {
    return "STEP_E7_FINAL_INSERT";
  }
  if (step_id == constants::STEP_E8_RELEASE) {
    return "STEP_E8_RELEASE";
  }
  if (step_id == constants::STEP_E9_RETREAT) {
    return "STEP_E9_RETREAT";
  }

  if (step_id == constants::STEP_D0_RESET_IO) {
    return "STEP_D0_RESET_IO";
  }
  if (step_id == constants::STEP_D1_PRE_GRASP) {
    return "STEP_D1_PRE_GRASP";
  }
  if (step_id == constants::STEP_D2_ENTER_GRASP) {
    return "STEP_D2_ENTER_GRASP";
  }
  if (step_id == constants::STEP_D3_GRASP) {
    return "STEP_D3_GRASP";
  }
  if (step_id == constants::STEP_D4_PRE_REMOVE_ROTATED) {
    return "STEP_D4_PRE_REMOVE_ROTATED";
  }
  if (step_id == constants::STEP_D5_REMOVE) {
    return "STEP_D5_REMOVE";
  }
  if (step_id == constants::STEP_D6_TRANSIT) {
    return "STEP_D6_TRANSIT";
  }
  if (step_id == constants::STEP_D7_PLACE) {
    return "STEP_D7_PLACE";
  }
  if (step_id == constants::STEP_D8_RELEASE) {
    return "STEP_D8_RELEASE";
  }
  if (step_id == constants::STEP_D9_RETREAT) {
    return "STEP_D9_RETREAT";
  }
  if (step_id == constants::STEP_HOME_RETURN) {
    return "STEP_HOME_RETURN";
  }

  return "UNKNOWN_STEP";
}

bool TaskManagerNode::can_trigger_step(uint32_t step_id, std::string & reason) const
{
  if (!is_step_supported(step_id)) {
    reason = "Unsupported step_id: " + std::to_string(step_id);
    return false;
  }

  if (step_id == constants::STEP_IDLE) {
    return true;
  }

  // ----------------------------
  // E flow
  // ----------------------------
  if (step_id == constants::STEP_E0_RESET_IO) {
    if (current_state_.current_step == constants::STEP_IDLE ||
      current_state_.current_step == constants::STEP_E9_RETREAT)
    {
      return true;
    }
    reason = "E0_RESET_IO is only allowed from STEP_IDLE or STEP_E9_RETREAT";
    return false;
  }

  if (step_id == constants::STEP_E1_PRE_GRASP) {
    if (current_state_.exec_substate == constants::EXEC_SUBSTATE_IO_RESET_DONE) {
      return true;
    }
    reason = "E1_PRE_GRASP requires exec_substate=IO_RESET_DONE";
    return false;
  }

  if (step_id == constants::STEP_E2_ENTER_GRASP) {
    if (current_state_.exec_substate == constants::EXEC_SUBSTATE_PRE_GRASP_REACHED) {
      return true;
    }
    reason = "E2_ENTER_GRASP requires exec_substate=PRE_GRASP_REACHED";
    return false;
  }

  if (step_id == constants::STEP_E3_GRASP) {
    if (current_state_.exec_substate == constants::EXEC_SUBSTATE_GRASP_POSE_REACHED) {
      return true;
    }
    reason = "E3_GRASP requires exec_substate=GRASP_POSE_REACHED";
    return false;
  }

  if (step_id == constants::STEP_E4_TRANSIT) {
    if (current_state_.grasp_confirmed &&
      current_state_.load_state == constants::LOAD_STATE_LOADED)
    {
      return true;
    }
    if (
      current_state_.current_step == constants::STEP_E4_TRANSIT &&
      current_state_.exec_substate ==
      constants::EXEC_SUBSTATE_TRANSIT_REACHED_WAITING_STEP_CONFIRM)
    {
      return true;
    }
    reason =
      "E4_TRANSIT requires "
      "(grasp_confirmed=true and load_state=LOADED) or "
      "(current_step=STEP_E4_TRANSIT and exec_substate=TRANSIT_REACHED_WAITING_STEP_CONFIRM)";
    return false;
  }

  if (step_id == constants::STEP_E5_PRE_INSERT) {
    if (current_state_.exec_substate == constants::EXEC_SUBSTATE_TRANSIT_STEP_CONFIRMED) {
      return true;
    }
    if (
      current_state_.current_step == constants::STEP_E5_PRE_INSERT &&
      current_state_.exec_substate ==
      constants::EXEC_SUBSTATE_PRE_INSERT_REACHED_WAITING_STEP_CONFIRM)
    {
      return true;
    }
    reason =
      "E5_PRE_INSERT requires "
      "exec_substate=TRANSIT_STEP_CONFIRMED or "
      "(current_step=STEP_E5_PRE_INSERT and "
      "exec_substate=PRE_INSERT_REACHED_WAITING_STEP_CONFIRM)";
    return false;
  }

  if (step_id == constants::STEP_E6_PRE_INSERT_ROTATED) {
    if (current_state_.exec_substate == constants::EXEC_SUBSTATE_PRE_INSERT_STEP_CONFIRMED) {
      return true;
    }
    if (
      current_state_.current_step == constants::STEP_E6_PRE_INSERT_ROTATED &&
      current_state_.exec_substate ==
      constants::EXEC_SUBSTATE_PRE_INSERT_ROTATED_REACHED_WAITING_STEP_CONFIRM)
    {
      return true;
    }
    reason =
      "E6_PRE_INSERT_ROTATED requires "
      "exec_substate=PRE_INSERT_STEP_CONFIRMED or "
      "(current_step=STEP_E6_PRE_INSERT_ROTATED and "
      "exec_substate=PRE_INSERT_ROTATED_REACHED_WAITING_STEP_CONFIRM)";
    return false;
  }

  if (step_id == constants::STEP_E7_FINAL_INSERT) {
    if (current_state_.exec_substate ==
      constants::EXEC_SUBSTATE_PRE_INSERT_ROTATED_STEP_CONFIRMED)
    {
      return true;
    }
    if (
      current_state_.current_step == constants::STEP_E7_FINAL_INSERT &&
      current_state_.exec_substate ==
      constants::EXEC_SUBSTATE_INSERT_POSE_REACHED_WAITING_STEP_CONFIRM)
    {
      return true;
    }
    reason =
      "E7_FINAL_INSERT requires "
      "exec_substate=PRE_INSERT_ROTATED_STEP_CONFIRMED or "
      "(current_step=STEP_E7_FINAL_INSERT and "
      "exec_substate=INSERT_POSE_REACHED_WAITING_STEP_CONFIRM)";
    return false;
  }

  if (step_id == constants::STEP_E8_RELEASE) {
    if (current_state_.exec_substate == constants::EXEC_SUBSTATE_INSERT_STEP_CONFIRMED) {
      return true;
    }
    reason = "E8_RELEASE requires exec_substate=INSERT_STEP_CONFIRMED";
    return false;
  }

  if (step_id == constants::STEP_E9_RETREAT) {
    if (current_state_.release_confirmed &&
      current_state_.load_state == constants::LOAD_STATE_EMPTY)
    {
      return true;
    }
    if (
      current_state_.current_step == constants::STEP_E9_RETREAT &&
      current_state_.exec_substate ==
      constants::EXEC_SUBSTATE_RETREAT_REACHED_WAITING_STEP_CONFIRM)
    {
      return true;
    }
    reason =
      "E9_RETREAT requires "
      "(release_confirmed=true and load_state=EMPTY) or "
      "(current_step=STEP_E9_RETREAT and exec_substate=RETREAT_REACHED_WAITING_STEP_CONFIRM)";
    return false;
  }

  // ----------------------------
  // D flow
  // ----------------------------
  if (step_id == constants::STEP_D0_RESET_IO) {
    if (current_state_.current_step == constants::STEP_IDLE ||
      current_state_.current_step == constants::STEP_D9_RETREAT)
    {
      return true;
    }
    reason = "D0_RESET_IO is only allowed from STEP_IDLE or STEP_D9_RETREAT";
    return false;
  }

  if (step_id == constants::STEP_D1_PRE_GRASP) {
    if (current_state_.exec_substate == constants::EXEC_SUBSTATE_IO_RESET_DONE) {
      return true;
    }
    reason = "D1_PRE_GRASP requires exec_substate=IO_RESET_DONE";
    return false;
  }

  if (step_id == constants::STEP_D2_ENTER_GRASP) {
    if (current_state_.exec_substate == constants::EXEC_SUBSTATE_PRE_GRASP_REACHED) {
      return true;
    }
    reason = "D2_ENTER_GRASP requires exec_substate=PRE_GRASP_REACHED";
    return false;
  }

  if (step_id == constants::STEP_D3_GRASP) {
    if (current_state_.exec_substate == constants::EXEC_SUBSTATE_GRASP_POSE_REACHED) {
      return true;
    }
    reason = "D3_GRASP requires exec_substate=GRASP_POSE_REACHED";
    return false;
  }

  if (step_id == constants::STEP_D4_PRE_REMOVE_ROTATED) {
    if (current_state_.grasp_confirmed &&
      current_state_.load_state == constants::LOAD_STATE_LOADED)
    {
      return true;
    }
    if (
      current_state_.current_step == constants::STEP_D4_PRE_REMOVE_ROTATED &&
      current_state_.exec_substate ==
      constants::EXEC_SUBSTATE_PRE_REMOVE_ROTATED_REACHED_WAITING_STEP_CONFIRM)
    {
      return true;
    }
    reason =
      "D4_PRE_REMOVE_ROTATED requires "
      "(grasp_confirmed=true and load_state=LOADED) or "
      "(current_step=STEP_D4_PRE_REMOVE_ROTATED and "
      "exec_substate=PRE_REMOVE_ROTATED_REACHED_WAITING_STEP_CONFIRM)";
    return false;
  }

  if (step_id == constants::STEP_D5_REMOVE) {
    if (current_state_.exec_substate ==
      constants::EXEC_SUBSTATE_PRE_REMOVE_ROTATED_STEP_CONFIRMED)
    {
      return true;
    }
    if (
      current_state_.current_step == constants::STEP_D5_REMOVE &&
      current_state_.exec_substate ==
      constants::EXEC_SUBSTATE_REMOVE_REACHED_WAITING_STEP_CONFIRM)
    {
      return true;
    }
    reason =
      "D5_REMOVE requires "
      "exec_substate=PRE_REMOVE_ROTATED_STEP_CONFIRMED or "
      "(current_step=STEP_D5_REMOVE and "
      "exec_substate=REMOVE_REACHED_WAITING_STEP_CONFIRM)";
    return false;
  }

  if (step_id == constants::STEP_D6_TRANSIT) {
    if (current_state_.exec_substate == constants::EXEC_SUBSTATE_REMOVE_STEP_CONFIRMED) {
      return true;
    }
    if (
      current_state_.current_step == constants::STEP_D6_TRANSIT &&
      current_state_.exec_substate ==
      constants::EXEC_SUBSTATE_TRANSIT_REACHED_WAITING_STEP_CONFIRM)
    {
      return true;
    }
    reason =
      "D6_TRANSIT requires "
      "exec_substate=REMOVE_STEP_CONFIRMED or "
      "(current_step=STEP_D6_TRANSIT and "
      "exec_substate=TRANSIT_REACHED_WAITING_STEP_CONFIRM)";
    return false;
  }

  if (step_id == constants::STEP_D7_PLACE) {
    if (current_state_.exec_substate == constants::EXEC_SUBSTATE_TRANSIT_STEP_CONFIRMED) {
      return true;
    }
    if (
      current_state_.current_step == constants::STEP_D7_PLACE &&
      current_state_.exec_substate ==
      constants::EXEC_SUBSTATE_PLACE_POSE_REACHED_WAITING_STEP_CONFIRM)
    {
      return true;
    }
    reason =
      "D7_PLACE requires "
      "exec_substate=TRANSIT_STEP_CONFIRMED or "
      "(current_step=STEP_D7_PLACE and "
      "exec_substate=PLACE_POSE_REACHED_WAITING_STEP_CONFIRM)";
    return false;
  }

  if (step_id == constants::STEP_D8_RELEASE) {
    if (current_state_.exec_substate == constants::EXEC_SUBSTATE_PLACE_STEP_CONFIRMED) {
      return true;
    }
    reason = "D8_RELEASE requires exec_substate=PLACE_STEP_CONFIRMED";
    return false;
  }

  if (step_id == constants::STEP_D9_RETREAT) {
    if (current_state_.release_confirmed &&
      current_state_.load_state == constants::LOAD_STATE_EMPTY)
    {
      return true;
    }
    if (
      current_state_.current_step == constants::STEP_D9_RETREAT &&
      current_state_.exec_substate ==
      constants::EXEC_SUBSTATE_RETREAT_REACHED_WAITING_STEP_CONFIRM)
    {
      return true;
    }
    reason =
      "D9_RETREAT requires "
      "(release_confirmed=true and load_state=EMPTY) or "
      "(current_step=STEP_D9_RETREAT and exec_substate=RETREAT_REACHED_WAITING_STEP_CONFIRM)";
    return false;
  }
  
  if (step_id == constants::STEP_HOME_RETURN) {
    if (
      current_state_.exec_substate ==    constants::EXEC_SUBSTATE_RETREAT_STEP_CONFIRMED &&
      (
        current_state_.current_step == constants::STEP_E9_RETREAT ||
        current_state_.current_step == constants::STEP_D9_RETREAT
      ))
    {
      return true;
    }

    if (
      current_state_.current_step == constants::STEP_HOME_RETURN &&
      current_state_.exec_substate ==
        constants::EXEC_SUBSTATE_HOME_REACHED_WAITING_STEP_CONFIRM)
    {
      return true;
    }

    reason =
      "STEP_HOME_RETURN requires "
      "(current_step=STEP_E9_RETREAT or STEP_D9_RETREAT) and "
      "exec_substate=RETREAT_STEP_CONFIRMED, or "
      "(current_step=STEP_HOME_RETURN and "
      "exec_substate=HOME_REACHED_WAITING_STEP_CONFIRM)";
    return false;
  }

  reason = "Unhandled step validation branch";
  return false;
}

bool TaskManagerNode::can_confirm_grasp(std::string & reason) const
{
  if (current_state_.current_step != constants::STEP_E3_GRASP &&
    current_state_.current_step != constants::STEP_D3_GRASP)
  {
    reason = "confirm_grasp is only allowed when current_step=STEP_E3_GRASP or STEP_D3_GRASP";
    return false;
  }

  if (current_state_.exec_substate != constants::EXEC_SUBSTATE_WAITING_GRASP_CONFIRM) {
    reason = "confirm_grasp requires exec_substate=WAITING_GRASP_CONFIRM";
    return false;
  }

  if (!current_state_.reached_box_grasp_tcp_pose) {
    reason = "confirm_grasp requires reached_box_grasp_tcp_pose=true";
    return false;
  }

  return true;
}

bool TaskManagerNode::can_confirm_release(std::string & reason) const
{
  if (current_state_.current_step != constants::STEP_E8_RELEASE &&
    current_state_.current_step != constants::STEP_D8_RELEASE)
  {
    reason =
      "confirm_release is only allowed when current_step=STEP_E8_RELEASE or STEP_D8_RELEASE";
    return false;
  }

  if (current_state_.exec_substate != constants::EXEC_SUBSTATE_WAITING_RELEASE_CONFIRM) {
    reason = "confirm_release requires exec_substate=WAITING_RELEASE_CONFIRM";
    return false;
  }

  if (!current_state_.reached_slot_insert_tcp_pose) {
    reason = "confirm_release requires reached_slot_insert_tcp_pose=true";
    return false;
  }

  return true;
}

bool TaskManagerNode::can_confirm_step_completion(
  uint32_t & next_step_id,
  std::string & reason) const
{
  next_step_id = 0;
  reason.clear();

  if (current_state_.current_phase != constants::PHASE_EXECUTION) {
    reason = "confirm_step_completion requires current_phase=EXECUTION";
    return false;
  }

  // ----------------------------
  // E flow
  // ----------------------------
  if (current_state_.current_step == constants::STEP_E4_TRANSIT) {
    if (current_state_.exec_substate !=
      constants::EXEC_SUBSTATE_TRANSIT_REACHED_WAITING_STEP_CONFIRM)
    {
      reason =
        "STEP_E4_TRANSIT requires exec_substate=TRANSIT_REACHED_WAITING_STEP_CONFIRM";
      return false;
    }
    next_step_id = constants::STEP_E5_PRE_INSERT;
    return true;
  }

  if (current_state_.current_step == constants::STEP_E5_PRE_INSERT) {
    if (current_state_.exec_substate !=
      constants::EXEC_SUBSTATE_PRE_INSERT_REACHED_WAITING_STEP_CONFIRM)
    {
      reason =
        "STEP_E5_PRE_INSERT requires exec_substate=PRE_INSERT_REACHED_WAITING_STEP_CONFIRM";
      return false;
    }
    next_step_id = constants::STEP_E6_PRE_INSERT_ROTATED;
    return true;
  }

  if (current_state_.current_step == constants::STEP_E6_PRE_INSERT_ROTATED) {
    if (current_state_.exec_substate !=
      constants::EXEC_SUBSTATE_PRE_INSERT_ROTATED_REACHED_WAITING_STEP_CONFIRM)
    {
      reason =
        "STEP_E6_PRE_INSERT_ROTATED requires "
        "exec_substate=PRE_INSERT_ROTATED_REACHED_WAITING_STEP_CONFIRM";
      return false;
    }
    next_step_id = constants::STEP_E7_FINAL_INSERT;
    return true;
  }

  if (current_state_.current_step == constants::STEP_E7_FINAL_INSERT) {
    if (current_state_.exec_substate !=
      constants::EXEC_SUBSTATE_INSERT_POSE_REACHED_WAITING_STEP_CONFIRM)
    {
      reason =
        "STEP_E7_FINAL_INSERT requires exec_substate=INSERT_POSE_REACHED_WAITING_STEP_CONFIRM";
      return false;
    }
    next_step_id = constants::STEP_E8_RELEASE;
    return true;
  }

  if (current_state_.current_step == constants::STEP_E9_RETREAT) {
    if (current_state_.exec_substate !=
      constants::EXEC_SUBSTATE_RETREAT_REACHED_WAITING_STEP_CONFIRM)
    {
      reason =
        "STEP_E9_RETREAT requires   exec_substate=RETREAT_REACHED_WAITING_STEP_CONFIRM";
      return false;
    }
    next_step_id = constants::STEP_HOME_RETURN;
    return true;
  }
  // ----------------------------
  // D flow
  // ----------------------------
  if (current_state_.current_step == constants::STEP_D4_PRE_REMOVE_ROTATED) {
    if (current_state_.exec_substate !=
      constants::EXEC_SUBSTATE_PRE_REMOVE_ROTATED_REACHED_WAITING_STEP_CONFIRM)
    {
      reason =
        "STEP_D4_PRE_REMOVE_ROTATED requires "
        "exec_substate=PRE_REMOVE_ROTATED_REACHED_WAITING_STEP_CONFIRM";
      return false;
    }
    next_step_id = constants::STEP_D5_REMOVE;
    return true;
  }

  if (current_state_.current_step == constants::STEP_D5_REMOVE) {
    if (current_state_.exec_substate !=
      constants::EXEC_SUBSTATE_REMOVE_REACHED_WAITING_STEP_CONFIRM)
    {
      reason =
        "STEP_D5_REMOVE requires exec_substate=REMOVE_REACHED_WAITING_STEP_CONFIRM";
      return false;
    }
    next_step_id = constants::STEP_D6_TRANSIT;
    return true;
  }

  if (current_state_.current_step == constants::STEP_D6_TRANSIT) {
    if (current_state_.exec_substate !=
      constants::EXEC_SUBSTATE_TRANSIT_REACHED_WAITING_STEP_CONFIRM)
    {
      reason =
        "STEP_D6_TRANSIT requires exec_substate=TRANSIT_REACHED_WAITING_STEP_CONFIRM";
      return false;
    }
    next_step_id = constants::STEP_D7_PLACE;
    return true;
  }

  if (current_state_.current_step == constants::STEP_D7_PLACE) {
    if (current_state_.exec_substate !=
      constants::EXEC_SUBSTATE_PLACE_POSE_REACHED_WAITING_STEP_CONFIRM)
    {
      reason =
        "STEP_D7_PLACE requires exec_substate=PLACE_POSE_REACHED_WAITING_STEP_CONFIRM";
      return false;
    }
    next_step_id = constants::STEP_D8_RELEASE;
    return true;
  }

  if (current_state_.current_step == constants::STEP_D9_RETREAT) {
    if (current_state_.exec_substate !=
      constants::EXEC_SUBSTATE_RETREAT_REACHED_WAITING_STEP_CONFIRM)
    {
      reason =
        "STEP_D9_RETREAT requires   exec_substate=RETREAT_REACHED_WAITING_STEP_CONFIRM";
      return false;
    }
    next_step_id = constants::STEP_HOME_RETURN;
    return true;
  }
  
  if (current_state_.current_step == constants::STEP_HOME_RETURN) {
    if (current_state_.exec_substate !=
      constants::EXEC_SUBSTATE_HOME_REACHED_WAITING_STEP_CONFIRM)
    {
      reason =
        "STEP_HOME_RETURN requires exec_substate=HOME_REACHED_WAITING_STEP_CONFIRM";
      return false;
    }
    next_step_id = 0;
    return true;
  }
  
  reason =
    "confirm_step_completion is not supported for current step " +
    step_to_string(current_state_.current_step);
  return false;
}

void TaskManagerNode::update_io_confirmation_from_state(
  const eli_common_interface::msg::IOState & msg)
{
  if (msg.tool_in.size() < 2) {
    current_state_.io_double_low_confirmed = false;
    current_state_.io_double_high_confirmed = false;

    last_io_state_valid_ = false;

    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      3000,
      "Received IOState but tool_in size is %zu, expected at least 2",
      msg.tool_in.size());
    return;
  }

  const bool tool_0 = static_cast<bool>(msg.tool_in[0]);
  const bool tool_1 = static_cast<bool>(msg.tool_in[1]);

  current_state_.io_double_low_confirmed = (!tool_0 && !tool_1);
  current_state_.io_double_high_confirmed = (tool_0 && tool_1);

  const bool io_state_changed =
    !last_io_state_valid_ ||
    tool_0 != last_tool_0_ ||
    tool_1 != last_tool_1_ ||
    current_state_.io_double_low_confirmed != last_io_double_low_confirmed_ ||
    current_state_.io_double_high_confirmed != last_io_double_high_confirmed_;

  if (io_state_changed) {
    RCLCPP_INFO(
      this->get_logger(),
      "IOState changed: size=%zu, tool_in[0]=%d, tool_in[1]=%d -> low=%d, high=%d",
      msg.tool_in.size(),
      static_cast<int>(tool_0),
      static_cast<int>(tool_1),
      static_cast<int>(current_state_.io_double_low_confirmed),
      static_cast<int>(current_state_.io_double_high_confirmed));
  }

  last_io_state_valid_ = true;
  last_tool_0_ = tool_0;
  last_tool_1_ = tool_1;
  last_io_double_low_confirmed_ = current_state_.io_double_low_confirmed;
  last_io_double_high_confirmed_ = current_state_.io_double_high_confirmed;
}

void TaskManagerNode::recompute_derived_state()
{
  const bool new_grasp_confirmed =
    current_state_.reached_box_grasp_tcp_pose &&
    current_state_.operator_grasp_confirmed &&
    current_state_.io_double_low_confirmed;

  const bool new_release_confirmed =
    current_state_.reached_slot_insert_tcp_pose &&
    current_state_.operator_release_confirmed &&
    current_state_.io_double_high_confirmed;

  const bool grasp_just_completed =
    (!current_state_.grasp_confirmed && new_grasp_confirmed);

  const bool release_just_completed =
    (!current_state_.release_confirmed && new_release_confirmed);

  current_state_.grasp_confirmed = new_grasp_confirmed;
  current_state_.release_confirmed = new_release_confirmed;

  if (current_state_.release_confirmed) {
    current_state_.box_state = constants::BOX_STATE_RELEASED;
    current_state_.load_state = constants::LOAD_STATE_EMPTY;
  } else if (current_state_.grasp_confirmed) {
    current_state_.box_state = constants::BOX_STATE_GRASPED;
    current_state_.load_state = constants::LOAD_STATE_LOADED;
  } else {
    current_state_.box_state = constants::BOX_STATE_NOT_GRASPED;
    current_state_.load_state = constants::LOAD_STATE_EMPTY;
  }

  if (grasp_just_completed) {
    current_state_.status = constants::STATUS_SUCCESS;
    current_state_.exec_substate = constants::EXEC_SUBSTATE_BOX_GRASPED;
    current_state_.last_message = "Grasp confirmed by operator + IO double low";
    publish_grasp_state_event("grasp_confirmed");
  } else if (
    (current_state_.current_step == constants::STEP_E3_GRASP ||
     current_state_.current_step == constants::STEP_D3_GRASP) &&
    current_state_.operator_grasp_confirmed &&
    !current_state_.grasp_confirmed)
  {
    current_state_.status = constants::STATUS_WAITING_CONFIRMATION;
    current_state_.exec_substate = constants::EXEC_SUBSTATE_WAITING_GRASP_CONFIRM;
    current_state_.last_message = "Operator confirmed grasp, waiting for IO double low";
  }

  if (release_just_completed) {
    current_state_.status = constants::STATUS_SUCCESS;
    current_state_.exec_substate = constants::EXEC_SUBSTATE_BOX_RELEASED;
    current_state_.last_message = "Release confirmed by operator + IO double high";
    publish_grasp_state_event("release_confirmed");
  } else if (
    (current_state_.current_step == constants::STEP_E8_RELEASE ||
     current_state_.current_step == constants::STEP_D8_RELEASE) &&
    current_state_.operator_release_confirmed &&
    !current_state_.release_confirmed)
  {
    current_state_.status = constants::STATUS_WAITING_CONFIRMATION;
    current_state_.exec_substate = constants::EXEC_SUBSTATE_WAITING_RELEASE_CONFIRM;
    current_state_.last_message = "Operator confirmed release, waiting for IO double high";
  }
}

void TaskManagerNode::handle_io_states(
  const eli_common_interface::msg::IOState::SharedPtr msg)
{
  update_io_confirmation_from_state(*msg);
  recompute_derived_state();
  publish_state();
}

void TaskManagerNode::apply_grasp_confirmation()
{
  current_state_.operator_grasp_confirmed = true;
  recompute_derived_state();

  if (!current_state_.grasp_confirmed) {
    current_state_.status = constants::STATUS_WAITING_CONFIRMATION;
    current_state_.last_message =
      "Operator confirmed grasp, waiting for IO double low";
  }
}

void TaskManagerNode::apply_release_confirmation()
{
  current_state_.operator_release_confirmed = true;
  recompute_derived_state();

  if (!current_state_.release_confirmed) {
    current_state_.status = constants::STATUS_WAITING_CONFIRMATION;
    current_state_.last_message =
      "Operator confirmed release, waiting for IO double high";
  }
}

bool TaskManagerNode::call_set_tool_io(int8_t pin, bool on, std::string & message)
{
  message.clear();

  if (!set_io_client_) {
    message = "SetIO client not initialized";
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    return false;
  }

  if (!set_io_client_->wait_for_service(1s)) {
    message = "Service /io_and_status_controller/set_io not available within 1s";
    RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
    return false;
  }

  auto req = std::make_shared<eli_common_interface::srv::SetIO::Request>();
  req->fun = eli_common_interface::srv::SetIO::Request::FUN_SET_TOOL_DIG_OUT;
  req->pin = pin;
  req->analog_type = eli_common_interface::srv::SetIO::Request::ANALOG_CURRENT;
  req->state = on ?
    eli_common_interface::srv::SetIO::Request::STATE_ON :
    eli_common_interface::srv::SetIO::Request::STATE_OFF;

  RCLCPP_INFO(
    this->get_logger(),
    "Calling SetIO (fire-and-forget): pin=%d, state=%s",
    static_cast<int>(pin),
    on ? "ON" : "OFF");

  try {
    (void)set_io_client_->async_send_request(req);
  } catch (const std::exception & e) {
    message =
      "SetIO async_send_request exception for pin " +
      std::to_string(static_cast<int>(pin)) + ": " + e.what();
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    return false;
  }

  message =
    "SetIO request sent for pin " + std::to_string(static_cast<int>(pin)) +
    ", state=" + (on ? "ON" : "OFF");
  RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
  return true;
}

bool TaskManagerNode::set_tool_io_pair(bool pin0_on, bool pin1_on, std::string & message)
{
  std::string msg0;
  std::string msg1;

  if (!call_set_tool_io(0, pin0_on, msg0)) {
    message = "Failed to set pin0: " + msg0;
    return false;
  }

  if (!call_set_tool_io(1, pin1_on, msg1)) {
    message = "Failed to set pin1: " + msg1;
    return false;
  }

  message =
    "Tool IO pair set success: pin0=" + std::string(pin0_on ? "ON" : "OFF") +
    ", pin1=" + std::string(pin1_on ? "ON" : "OFF");
  return true;
}

void TaskManagerNode::apply_step_transition(uint32_t step_id, const std::string & extra_param)
{
  (void)extra_param;

  current_state_.current_step = step_id;
  current_state_.status = constants::STATUS_STEP_TRIGGERED;

  if (step_id == constants::STEP_IDLE) {
    initialize_state();
    current_state_.last_message = "Switched to STEP_IDLE";
    publish_grasp_state_event("reset");
    return;
  }

  current_state_.current_phase = constants::PHASE_EXECUTION;

  // ----------------------------
  // E flow
  // ----------------------------
  if (step_id == constants::STEP_E0_RESET_IO) {
    reset_execution_flags();
    current_state_.box_state = constants::BOX_STATE_NOT_GRASPED;
    current_state_.load_state = constants::LOAD_STATE_EMPTY;
    current_state_.exec_substate = constants::EXEC_SUBSTATE_IO_RESET_DONE;
    current_state_.last_message = "Triggered E0_RESET_IO";
    return;
  }

  if (step_id == constants::STEP_E1_PRE_GRASP) {
    current_state_.exec_substate = constants::EXEC_SUBSTATE_PRE_GRASP_REACHED;
    current_state_.last_message = "Triggered E1_PRE_GRASP";
    return;
  }

  if (step_id == constants::STEP_E2_ENTER_GRASP) {
    current_state_.exec_substate = constants::EXEC_SUBSTATE_GRASP_POSE_REACHED;
    current_state_.reached_box_grasp_tcp_pose = true;
    current_state_.last_message = "Triggered E2_ENTER_GRASP";
    recompute_derived_state();
    return;
  }

  if (step_id == constants::STEP_E3_GRASP) {
    current_state_.status = constants::STATUS_WAITING_CONFIRMATION;
    current_state_.exec_substate = constants::EXEC_SUBSTATE_WAITING_GRASP_CONFIRM;
    current_state_.last_message =
      "Triggered E3_GRASP, IO switched to double low, waiting for grasp confirmation";
    return;
  }

  if (step_id == constants::STEP_E4_TRANSIT) {
    current_state_.status = constants::STATUS_WAITING_CONFIRMATION;
    current_state_.exec_substate =
      constants::EXEC_SUBSTATE_TRANSIT_REACHED_WAITING_STEP_CONFIRM;
    current_state_.last_message =
      "Triggered E4_TRANSIT, reached transit pose, waiting for step completion confirmation";
    return;
  }

  if (step_id == constants::STEP_E5_PRE_INSERT) {
    current_state_.status = constants::STATUS_WAITING_CONFIRMATION;
    current_state_.exec_substate =
      constants::EXEC_SUBSTATE_PRE_INSERT_REACHED_WAITING_STEP_CONFIRM;
    current_state_.last_message =
      "Triggered E5_PRE_INSERT, reached pre-insert pose, waiting for step completion confirmation";
    return;
  }

  if (step_id == constants::STEP_E6_PRE_INSERT_ROTATED) {
    current_state_.status = constants::STATUS_WAITING_CONFIRMATION;
    current_state_.exec_substate =
      constants::EXEC_SUBSTATE_PRE_INSERT_ROTATED_REACHED_WAITING_STEP_CONFIRM;
    current_state_.last_message =
      "Triggered E6_PRE_INSERT_ROTATED, reached rotated pre-insert pose, waiting for step completion confirmation";
    return;
  }

  if (step_id == constants::STEP_E7_FINAL_INSERT) {
    current_state_.status = constants::STATUS_WAITING_CONFIRMATION;
    current_state_.exec_substate =
      constants::EXEC_SUBSTATE_INSERT_POSE_REACHED_WAITING_STEP_CONFIRM;
    current_state_.reached_slot_insert_tcp_pose = true;
    current_state_.last_message =
      "Triggered E7_FINAL_INSERT, reached insert pose, waiting for step completion confirmation";
    recompute_derived_state();
    current_state_.status = constants::STATUS_WAITING_CONFIRMATION;
    current_state_.exec_substate =
      constants::EXEC_SUBSTATE_INSERT_POSE_REACHED_WAITING_STEP_CONFIRM;
    return;
  }

  if (step_id == constants::STEP_E8_RELEASE) {
    current_state_.status = constants::STATUS_WAITING_CONFIRMATION;
    current_state_.exec_substate = constants::EXEC_SUBSTATE_WAITING_RELEASE_CONFIRM;
    current_state_.last_message =
      "Triggered E8_RELEASE, IO switched to double high, waiting for release confirmation";
    return;
  }

  if (step_id == constants::STEP_E9_RETREAT) {
    current_state_.status = constants::STATUS_WAITING_CONFIRMATION;
    current_state_.exec_substate =
      constants::EXEC_SUBSTATE_RETREAT_REACHED_WAITING_STEP_CONFIRM;
    current_state_.last_message =
      "Triggered E9_RETREAT, reached retreat pose, waiting for step completion confirmation";
    return;
  }

  // ----------------------------
  // D flow
  // ----------------------------
  if (step_id == constants::STEP_D0_RESET_IO) {
    reset_execution_flags();
    current_state_.box_state = constants::BOX_STATE_NOT_GRASPED;
    current_state_.load_state = constants::LOAD_STATE_EMPTY;
    current_state_.exec_substate = constants::EXEC_SUBSTATE_IO_RESET_DONE;
    current_state_.last_message = "Triggered D0_RESET_IO";
    return;
  }

  if (step_id == constants::STEP_D1_PRE_GRASP) {
    current_state_.exec_substate = constants::EXEC_SUBSTATE_PRE_GRASP_REACHED;
    current_state_.last_message = "Triggered D1_PRE_GRASP";
    return;
  }

  if (step_id == constants::STEP_D2_ENTER_GRASP) {
    current_state_.exec_substate = constants::EXEC_SUBSTATE_GRASP_POSE_REACHED;
    current_state_.reached_box_grasp_tcp_pose = true;
    current_state_.last_message = "Triggered D2_ENTER_GRASP";
    recompute_derived_state();
    return;
  }

  if (step_id == constants::STEP_D3_GRASP) {
    current_state_.status = constants::STATUS_WAITING_CONFIRMATION;
    current_state_.exec_substate = constants::EXEC_SUBSTATE_WAITING_GRASP_CONFIRM;
    current_state_.last_message =
      "Triggered D3_GRASP, IO switched to double low, waiting for grasp confirmation";
    return;
  }

  if (step_id == constants::STEP_D4_PRE_REMOVE_ROTATED) {
    current_state_.status = constants::STATUS_WAITING_CONFIRMATION;
    current_state_.exec_substate =
      constants::EXEC_SUBSTATE_PRE_REMOVE_ROTATED_REACHED_WAITING_STEP_CONFIRM;
    current_state_.last_message =
      "Triggered D4_PRE_REMOVE_ROTATED, reached rotated pre-remove pose, waiting for step completion confirmation";
    return;
  }

  if (step_id == constants::STEP_D5_REMOVE) {
    current_state_.status = constants::STATUS_WAITING_CONFIRMATION;
    current_state_.exec_substate =
      constants::EXEC_SUBSTATE_REMOVE_REACHED_WAITING_STEP_CONFIRM;
    current_state_.last_message =
      "Triggered D5_REMOVE, reached remove pose, waiting for step completion confirmation";
    return;
  }

  if (step_id == constants::STEP_D6_TRANSIT) {
    current_state_.status = constants::STATUS_WAITING_CONFIRMATION;
    current_state_.exec_substate =
      constants::EXEC_SUBSTATE_TRANSIT_REACHED_WAITING_STEP_CONFIRM;
    current_state_.last_message =
      "Triggered D6_TRANSIT, reached transit pose, waiting for step completion confirmation";
    return;
  }

  if (step_id == constants::STEP_D7_PLACE) {
    current_state_.status = constants::STATUS_WAITING_CONFIRMATION;
    current_state_.exec_substate =
      constants::EXEC_SUBSTATE_PLACE_POSE_REACHED_WAITING_STEP_CONFIRM;
    current_state_.reached_slot_insert_tcp_pose = true;
    current_state_.last_message =
      "Triggered D7_PLACE, reached place pose, waiting for step completion confirmation";
    recompute_derived_state();
    current_state_.status = constants::STATUS_WAITING_CONFIRMATION;
    current_state_.exec_substate =
      constants::EXEC_SUBSTATE_PLACE_POSE_REACHED_WAITING_STEP_CONFIRM;
    return;
  }

  if (step_id == constants::STEP_D8_RELEASE) {
    current_state_.status = constants::STATUS_WAITING_CONFIRMATION;
    current_state_.exec_substate = constants::EXEC_SUBSTATE_WAITING_RELEASE_CONFIRM;
    current_state_.last_message =
      "Triggered D8_RELEASE, IO switched to double high, waiting for release confirmation";
    return;
  }

  if (step_id == constants::STEP_D9_RETREAT) {
    current_state_.status = constants::STATUS_WAITING_CONFIRMATION;
    current_state_.exec_substate =
      constants::EXEC_SUBSTATE_RETREAT_REACHED_WAITING_STEP_CONFIRM;
    current_state_.last_message =
      "Triggered D9_RETREAT, reached retreat pose, waiting for step completion confirmation";
    return;
  }
  
  if (step_id == constants::STEP_HOME_RETURN) {
    current_state_.status = constants::STATUS_WAITING_CONFIRMATION;
    current_state_.exec_substate =
      constants::EXEC_SUBSTATE_HOME_REACHED_WAITING_STEP_CONFIRM;
    current_state_.last_message =
      "Triggered HOME_RETURN, reached home pose, waiting for step completion   confirmation";
    return;
  }
  
}

void TaskManagerNode::handle_trigger_step(
  const std::shared_ptr<cs625_task_manager::srv::TriggerStep::Request> request,
  std::shared_ptr<cs625_task_manager::srv::TriggerStep::Response> response)
{
  const auto step_id = request->step_id;
  const auto extra_param = request->extra_param;

  std::string reason;

  if (!can_trigger_step(step_id, reason)) {
    response->accepted = false;
    response->success = false;
    response->message = reason;

    current_state_.status = constants::STATUS_REJECTED;
    current_state_.last_message = reason;

    if (!is_step_supported(step_id)) {
      current_state_.current_phase = constants::PHASE_ERROR;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Rejected trigger_step request, step=%s(%u), extra_param='%s', reason='%s'",
      step_to_string(step_id).c_str(),
      step_id,
      extra_param.c_str(),
      reason.c_str());

    publish_state();
    return;
  }

  if (step_id == constants::STEP_E3_GRASP || step_id == constants::STEP_D3_GRASP) {
    std::string io_message;
    const bool io_ok = set_tool_io_pair(false, false, io_message);

    if (!io_ok) {
      response->accepted = false;
      response->success = false;
      response->message = step_to_string(step_id) + " rejected because SetIO failed: " + io_message;

      current_state_.status = constants::STATUS_REJECTED;
      current_state_.last_message = response->message;

      RCLCPP_WARN(
        this->get_logger(),
        "Rejected trigger_step request, step=%s(%u), extra_param='%s', reason='%s'",
        step_to_string(step_id).c_str(),
        step_id,
        extra_param.c_str(),
        response->message.c_str());

      publish_state();
      return;
    }
  }

  if (step_id == constants::STEP_E8_RELEASE || step_id == constants::STEP_D8_RELEASE) {
    std::string io_message;
    const bool io_ok = set_tool_io_pair(true, true, io_message);

    if (!io_ok) {
      response->accepted = false;
      response->success = false;
      response->message = step_to_string(step_id) + " rejected because SetIO failed: " + io_message;

      current_state_.status = constants::STATUS_REJECTED;
      current_state_.last_message = response->message;

      RCLCPP_WARN(
        this->get_logger(),
        "Rejected trigger_step request, step=%s(%u), extra_param='%s', reason='%s'",
        step_to_string(step_id).c_str(),
        step_id,
        extra_param.c_str(),
        response->message.c_str());

      publish_state();
      return;
    }
  }

  apply_step_transition(step_id, extra_param);

  response->accepted = true;
  response->success = true;
  response->message =
    "Accepted " + step_to_string(step_id) + "(" + std::to_string(step_id) +
    "), extra_param='" + extra_param + "'";

  RCLCPP_INFO(
    this->get_logger(),
    "Accepted trigger_step request, step=%s(%u), extra_param='%s'",
    step_to_string(step_id).c_str(),
    step_id,
    extra_param.c_str());

  publish_state();
}

void TaskManagerNode::handle_confirm_grasp(
  const std::shared_ptr<cs625_task_manager::srv::ConfirmAction::Request> request,
  std::shared_ptr<cs625_task_manager::srv::ConfirmAction::Response> response)
{
  (void)request;

  std::string reason;
  if (!can_confirm_grasp(reason)) {
    response->accepted = false;
    response->success = false;
    response->message = reason;

    current_state_.status = constants::STATUS_REJECTED;
    current_state_.last_message = reason;

    RCLCPP_WARN(
      this->get_logger(),
      "Rejected confirm_grasp request, reason='%s'",
      reason.c_str());

    publish_state();
    return;
  }

  apply_grasp_confirmation();

  response->accepted = true;
  response->success = true;
  response->message = current_state_.grasp_confirmed ?
    "Accepted confirm_grasp, grasp fully confirmed" :
    "Accepted confirm_grasp, waiting for IO double low";

  RCLCPP_INFO(this->get_logger(), "Accepted confirm_grasp request");
  publish_state();
}

void TaskManagerNode::handle_confirm_release(
  const std::shared_ptr<cs625_task_manager::srv::ConfirmAction::Request> request,
  std::shared_ptr<cs625_task_manager::srv::ConfirmAction::Response> response)
{
  (void)request;

  std::string reason;
  if (!can_confirm_release(reason)) {
    response->accepted = false;
    response->success = false;
    response->message = reason;

    current_state_.status = constants::STATUS_REJECTED;
    current_state_.last_message = reason;

    RCLCPP_WARN(
      this->get_logger(),
      "Rejected confirm_release request, reason='%s'",
      reason.c_str());

    publish_state();
    return;
  }

  apply_release_confirmation();

  response->accepted = true;
  response->success = true;
  response->message = current_state_.release_confirmed ?
    "Accepted confirm_release, release fully confirmed" :
    "Accepted confirm_release, waiting for IO double high";

  RCLCPP_INFO(this->get_logger(), "Accepted confirm_release request");
  publish_state();
}

void TaskManagerNode::handle_confirm_step_completion(
  const std::shared_ptr<cs625_task_manager::srv::ConfirmAction::Request> request,
  std::shared_ptr<cs625_task_manager::srv::ConfirmAction::Response> response)
{
  (void)request;

  uint32_t next_step_id = 0;
  std::string reason;
  if (!can_confirm_step_completion(next_step_id, reason)) {
    response->accepted = false;
    response->success = false;
    response->message = reason;

    current_state_.status = constants::STATUS_REJECTED;
    current_state_.last_message = reason;

    RCLCPP_WARN(
      this->get_logger(),
      "Rejected confirm_step_completion request, reason='%s'",
      reason.c_str());

    publish_state();
    return;
  }

  if (current_state_.current_step == constants::STEP_E4_TRANSIT) {
    current_state_.status = constants::STATUS_SUCCESS;
    current_state_.exec_substate = constants::EXEC_SUBSTATE_TRANSIT_STEP_CONFIRMED;
    current_state_.last_message =
      "Accepted confirm_step_completion, E4_TRANSIT confirmed, E5 unlocked";
  } else if (current_state_.current_step == constants::STEP_E5_PRE_INSERT) {
    current_state_.status = constants::STATUS_SUCCESS;
    current_state_.exec_substate = constants::EXEC_SUBSTATE_PRE_INSERT_STEP_CONFIRMED;
    current_state_.last_message =
      "Accepted confirm_step_completion, E5_PRE_INSERT confirmed, E6 unlocked";
  } else if (current_state_.current_step == constants::STEP_E6_PRE_INSERT_ROTATED) {
    current_state_.status = constants::STATUS_SUCCESS;
    current_state_.exec_substate =
      constants::EXEC_SUBSTATE_PRE_INSERT_ROTATED_STEP_CONFIRMED;
    current_state_.last_message =
      "Accepted confirm_step_completion, E6_PRE_INSERT_ROTATED confirmed, E7 unlocked";
  } else if (current_state_.current_step == constants::STEP_E7_FINAL_INSERT) {
    current_state_.status = constants::STATUS_SUCCESS;
    current_state_.exec_substate = constants::EXEC_SUBSTATE_INSERT_STEP_CONFIRMED;
    current_state_.last_message =
      "Accepted confirm_step_completion, E7_FINAL_INSERT confirmed, E8 unlocked";
  } else if (current_state_.current_step == constants::STEP_E9_RETREAT) {
    current_state_.status = constants::STATUS_SUCCESS;
    current_state_.exec_substate = constants::EXEC_SUBSTATE_RETREAT_STEP_CONFIRMED;
    current_state_.last_message =
      "Accepted confirm_step_completion, E9_RETREAT confirmed, HOME_RETURN unlocked";

  } else if (current_state_.current_step == constants::STEP_D4_PRE_REMOVE_ROTATED) {
    current_state_.status = constants::STATUS_SUCCESS;
    current_state_.exec_substate =
      constants::EXEC_SUBSTATE_PRE_REMOVE_ROTATED_STEP_CONFIRMED;
    current_state_.last_message =
      "Accepted confirm_step_completion, D4_PRE_REMOVE_ROTATED confirmed, D5 unlocked";
  } else if (current_state_.current_step == constants::STEP_D5_REMOVE) {
    current_state_.status = constants::STATUS_SUCCESS;
    current_state_.exec_substate = constants::EXEC_SUBSTATE_REMOVE_STEP_CONFIRMED;
    current_state_.last_message =
      "Accepted confirm_step_completion, D5_REMOVE confirmed, D6 unlocked";
  } else if (current_state_.current_step == constants::STEP_D6_TRANSIT) {
    current_state_.status = constants::STATUS_SUCCESS;
    current_state_.exec_substate = constants::EXEC_SUBSTATE_TRANSIT_STEP_CONFIRMED;
    current_state_.last_message =
      "Accepted confirm_step_completion, D6_TRANSIT confirmed, D7 unlocked";
  } else if (current_state_.current_step == constants::STEP_D7_PLACE) {
    current_state_.status = constants::STATUS_SUCCESS;
    current_state_.exec_substate = constants::EXEC_SUBSTATE_PLACE_STEP_CONFIRMED;
    current_state_.last_message =
      "Accepted confirm_step_completion, D7_PLACE confirmed, D8 unlocked";
  } else if (current_state_.current_step == constants::STEP_HOME_RETURN) {
    current_state_.status = constants::STATUS_COMPLETED;
    current_state_.exec_substate = constants::EXEC_SUBSTATE_HOME_STEP_CONFIRMED;
    current_state_.current_phase = constants::PHASE_DONE;
    current_state_.last_message =
      "Accepted confirm_step_completion, HOME_RETURN confirmed, task finished";
      
  } else if (current_state_.current_step == constants::STEP_D9_RETREAT) {
    current_state_.status = constants::STATUS_SUCCESS;
    current_state_.exec_substate = constants::EXEC_SUBSTATE_RETREAT_STEP_CONFIRMED;
    current_state_.last_message =
      "Accepted confirm_step_completion, D9_RETREAT confirmed, HOME_RETURN unlocked";
      
  } else {
    response->accepted = false;
    response->success = false;
    response->message =
      "confirm_step_completion reached unexpected branch for current step " +
      step_to_string(current_state_.current_step);

    current_state_.status = constants::STATUS_REJECTED;
    current_state_.last_message = response->message;

    RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
    publish_state();
    return;
  }

  response->accepted = true;
  response->success = true;
  response->message = current_state_.last_message;

  RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  publish_state();
}

}  // namespace cs625_task_manager
