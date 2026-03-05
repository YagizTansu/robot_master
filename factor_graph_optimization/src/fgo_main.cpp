#include <rclcpp/rclcpp.hpp>
#include "factor_graph_optimization/fgo_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<factor_graph_optimization::FgoNode>());
  rclcpp::shutdown();
  return 0;
}
