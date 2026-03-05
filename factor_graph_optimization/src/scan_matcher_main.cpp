#include <rclcpp/rclcpp.hpp>
#include "factor_graph_optimization/scan_matcher.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<factor_graph_optimization::ScanMatcherNode>());
  rclcpp::shutdown();
  return 0;
}
