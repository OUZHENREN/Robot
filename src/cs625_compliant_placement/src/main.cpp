#include "rclcpp/rclcpp.hpp"
#include "cs625_compliant_placement/compliant_placement_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cs625_compliant_placement::CompliantPlacementNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
