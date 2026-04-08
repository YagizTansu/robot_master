#include "robot_custom_behaviour_tree_nodes/thermal_check_action.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>

namespace robot_custom_behaviour_tree_nodes
{

ThermalCheckAction::ThermalCheckAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  node_ = rclcpp::Node::make_shared("thermal_check_node");

  if (!getInput("diagnostics_topic", diagnostics_topic_)) {
    diagnostics_topic_ = "/diagnostics_agg";
  }

  diag_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    diagnostics_topic_,
    rclcpp::QoS(10).best_effort(),
    std::bind(&ThermalCheckAction::diagnosticsCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(),
    "ThermalCheckAction initialized on topic: '%s'", diagnostics_topic_.c_str());
}

void ThermalCheckAction::diagnosticsCallback(
  const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
  last_diag_msg_ = msg;
  last_msg_time_ = node_->get_clock()->now();
  received_first_msg_ = true;
}

bool ThermalCheckAction::extractTemperature(
  const diagnostic_msgs::msg::DiagnosticStatus & status,
  double & out_temp) const
{
  for (const auto & kv : status.values) {
    std::string key_lower = kv.key;
    std::transform(key_lower.begin(), key_lower.end(), key_lower.begin(), ::tolower);

    if (key_lower.find("temp") != std::string::npos) {
      try {
        out_temp = std::stod(kv.value);
        return true;
      } catch (const std::exception &) {
        // Value not parseable as float — skip
      }
    }
  }
  return false;
}

BT::NodeStatus ThermalCheckAction::tick()
{
  double timeout_sec = 3.0;
  double max_temp_celsius = 70.0;
  std::string component_name;

  getInput("timeout_sec", timeout_sec);
  getInput("max_temp_celsius", max_temp_celsius);
  getInput("component_name", component_name);

  rclcpp::spin_some(node_);

  // Fail-safe: no diagnostics received
  if (!received_first_msg_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "ThermalCheck: no diagnostics received on '%s'. Thermal status unknown.",
      diagnostics_topic_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Fail-safe: stale diagnostics
  double age_sec = (node_->get_clock()->now() - last_msg_time_).seconds();
  if (age_sec > timeout_sec) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
      "ThermalCheck: diagnostics STALE — last message %.2f s ago (limit: %.2f s).",
      age_sec, timeout_sec);
    return BT::NodeStatus::FAILURE;
  }

  // Find matching component (case-insensitive substring match)
  for (const auto & status : last_diag_msg_->status) {
    std::string name_lower = status.name;
    std::string comp_lower = component_name;
    std::transform(name_lower.begin(), name_lower.end(), name_lower.begin(), ::tolower);
    std::transform(comp_lower.begin(), comp_lower.end(), comp_lower.begin(), ::tolower);

    if (name_lower.find(comp_lower) == std::string::npos) {
      continue;
    }

    // Component level check (ERROR or STALE)
    if (status.level >= 2) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
        "ThermalCheck: component '%s' is in %s state — halting.",
        status.name.c_str(), (status.level == 2) ? "ERROR" : "STALE");
      return BT::NodeStatus::FAILURE;
    }

    // Numeric temperature key-value check
    double temp = 0.0;
    if (extractTemperature(status, temp)) {
      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "ThermalCheck: '%s' temperature = %.1f°C (limit: %.1f°C).",
        status.name.c_str(), temp, max_temp_celsius);

      if (temp >= max_temp_celsius) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
          "ThermalCheck: OVERHEAT! '%s' temperature %.1f°C exceeds limit %.1f°C.",
          status.name.c_str(), temp, max_temp_celsius);
        return BT::NodeStatus::FAILURE;
      }
    }

    return BT::NodeStatus::SUCCESS;
  }

  // Component not found in diagnostics — non-fatal, log debug only
  RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
    "ThermalCheck: component '%s' not found in diagnostics. Skipping.",
    component_name.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_custom_behaviour_tree_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<
    robot_custom_behaviour_tree_nodes::ThermalCheckAction>("ThermalCheck");
}
