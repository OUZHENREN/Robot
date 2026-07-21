#ifndef CS625_TASK_MANAGER__TASK_MANAGER_NODE_HPP_
#define CS625_TASK_MANAGER__TASK_MANAGER_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "cs625_task_manager/msg/task_state.hpp"
#include "cs625_task_manager/srv/trigger_step.hpp"
#include "cs625_task_manager/srv/confirm_action.hpp"
#include "eli_common_interface/msg/io_state.hpp"
#include "eli_common_interface/srv/set_io.hpp"
#include "std_msgs/msg/string.hpp"

namespace cs625_task_manager
{

class TaskManagerNode : public rclcpp::Node
{
public:
  TaskManagerNode();

private:
  void initialize_state();
  void publish_state();
  void publish_grasp_state_event(const std::string & event_name);
  void timer_callback();

  void reset_execution_flags();

  bool is_step_supported(uint32_t step_id) const;
  bool can_trigger_step(uint32_t step_id, std::string & reason) const;
  void apply_step_transition(uint32_t step_id, const std::string & extra_param);
  std::string step_to_string(uint32_t step_id) const;

  bool can_confirm_grasp(std::string & reason) const;
  bool can_confirm_release(std::string & reason) const;
  bool can_confirm_step_completion(uint32_t & next_step_id, std::string & reason) const;

  void apply_grasp_confirmation();
  void apply_release_confirmation();

  void handle_trigger_step(
    const std::shared_ptr<cs625_task_manager::srv::TriggerStep::Request> request,
    std::shared_ptr<cs625_task_manager::srv::TriggerStep::Response> response);

  void handle_confirm_grasp(
    const std::shared_ptr<cs625_task_manager::srv::ConfirmAction::Request> request,
    std::shared_ptr<cs625_task_manager::srv::ConfirmAction::Response> response);

  void handle_confirm_release(
    const std::shared_ptr<cs625_task_manager::srv::ConfirmAction::Request> request,
    std::shared_ptr<cs625_task_manager::srv::ConfirmAction::Response> response);

  void handle_confirm_step_completion(
    const std::shared_ptr<cs625_task_manager::srv::ConfirmAction::Request> request,
    std::shared_ptr<cs625_task_manager::srv::ConfirmAction::Response> response);

  void handle_io_states(
    const eli_common_interface::msg::IOState::SharedPtr msg);

  void update_io_confirmation_from_state(
    const eli_common_interface::msg::IOState & msg);

  void recompute_derived_state();

  bool call_set_tool_io(int8_t pin, bool on, std::string & message);
  bool set_tool_io_pair(bool pin0_on, bool pin1_on, std::string & message);

  rclcpp::Publisher<cs625_task_manager::msg::TaskState>::SharedPtr task_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr grasp_state_event_pub_;

  rclcpp::Service<cs625_task_manager::srv::TriggerStep>::SharedPtr trigger_step_srv_;
  rclcpp::Service<cs625_task_manager::srv::ConfirmAction>::SharedPtr confirm_grasp_srv_;
  rclcpp::Service<cs625_task_manager::srv::ConfirmAction>::SharedPtr confirm_release_srv_;
  rclcpp::Service<cs625_task_manager::srv::ConfirmAction>::SharedPtr
    confirm_step_completion_srv_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Subscription<eli_common_interface::msg::IOState>::SharedPtr io_states_sub_;
  rclcpp::Client<eli_common_interface::srv::SetIO>::SharedPtr set_io_client_;

  cs625_task_manager::msg::TaskState current_state_;

  bool last_io_state_valid_;
  bool last_tool_0_;
  bool last_tool_1_;
  bool last_io_double_low_confirmed_;
  bool last_io_double_high_confirmed_;
};

}  // namespace cs625_task_manager

#endif  // CS625_TASK_MANAGER__TASK_MANAGER_NODE_HPP_
