#include "rclcpp/rclcpp.hpp"
#include "cs625_task_manager/task_manager_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cs625_task_manager::TaskManagerNode>());
  rclcpp::shutdown();
  return 0;
}
