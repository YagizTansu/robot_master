#include "robot_custom_behaviour_tree_nodes/hardware_diagnostics_check_action.hpp"

namespace robot_custom_behaviour_tree_nodes
{

HardwareDiagnosticsCheckAction::HardwareDiagnosticsCheckAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf),
  received_first_msg_(false)
{
  node_ = rclcpp::Node::make_shared("hardware_diagnostics_check_action_node");

  if (!getInput("diagnostics_topic", diagnostics_topic_)) {
    diagnostics_topic_ = "/diagnostics_agg";
  }

  diag_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    diagnostics_topic_,
    rclcpp::QoS(10).best_effort(),
    std::bind(&HardwareDiagnosticsCheckAction::diagnosticsCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(),
    "HardwareDiagnosticsCheckAction initialized on topic: %s", diagnostics_topic_.c_str());
}

void HardwareDiagnosticsCheckAction::diagnosticsCallback(
  const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
  last_diag_msg_ = msg;
  last_msg_time_ = node_->get_clock()->now();
  received_first_msg_ = true;
}

BT::NodeStatus HardwareDiagnosticsCheckAction::tick()
{
  double timeout_sec = 3.0;
  int fail_on_level = 2;
  getInput("timeout_sec", timeout_sec);
  getInput("fail_on_level", fail_on_level);

  rclcpp::spin_some(node_);

  // Fail-safe: no diagnostics received at all
  if (!received_first_msg_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "No diagnostics received on '%s'. Hardware status unknown — UNSAFE.",
      diagnostics_topic_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Fail-safe: diagnostics message is stale
  double age_sec = (node_->get_clock()->now() - last_msg_time_).seconds();
  if (age_sec > timeout_sec) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
      "Diagnostics topic stale! Last message was %.2f s ago (limit: %.2f s).",
      age_sec, timeout_sec);
    return BT::NodeStatus::FAILURE;
  }

  // Check each component's status level
  for (const auto & status : last_diag_msg_->status) {
    if (static_cast<int>(status.level) >= fail_on_level) {
      const char * level_str = (status.level == 2) ? "ERROR" : "STALE";
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
        "Hardware fault detected! Component: '%s', Level: %s, Message: '%s'",
        status.name.c_str(), level_str, status.message.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
    "Hardware diagnostics OK. %zu components checked.", last_diag_msg_->status.size());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_custom_behaviour_tree_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<
    robot_custom_behaviour_tree_nodes::HardwareDiagnosticsCheckAction
  >("HardwareDiagnosticsCheck");
}
