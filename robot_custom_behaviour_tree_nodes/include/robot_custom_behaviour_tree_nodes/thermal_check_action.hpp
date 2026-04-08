#ifndef ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__THERMAL_CHECK_ACTION_HPP_
#define ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__THERMAL_CHECK_ACTION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_custom_behaviour_tree_nodes
{

/**
 * @brief BT Condition node that monitors component temperature via diagnostics.
 *
 * Subscribes to a DiagnosticArray topic and locates the status entry whose name
 * contains component_name (case-insensitive substring match). Returns FAILURE when:
 *   - No diagnostics message received (fail-safe).
 *   - The last message is older than timeout_sec.
 *   - The matching component's key-value pairs contain a temperature reading
 *     that exceeds max_temp_celsius.
 *   - The matching component's DiagnosticStatus level is ERROR(2) or STALE(3).
 *
 * Input Ports:
 *   - diagnostics_topic  : Aggregated diagnostics topic (default: "/diagnostics_agg")
 *   - component_name     : Substring to match against DiagnosticStatus names
 *   - max_temp_celsius   : Temperature threshold in degrees Celsius (default: 70.0)
 *   - timeout_sec        : Maximum allowed age of last diagnostics message (default: 3.0)
 */
class ThermalCheckAction : public BT::ConditionNode
{
public:
  ThermalCheckAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  ~ThermalCheckAction() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("diagnostics_topic", "/diagnostics_agg",
                                 "DiagnosticArray topic (aggregated diagnostics)"),
      BT::InputPort<std::string>("component_name", "motor_driver",
                                 "Substring to match in DiagnosticStatus name"),
      BT::InputPort<double>("max_temp_celsius", 70.0,
                            "Temperature threshold in degrees Celsius"),
      BT::InputPort<double>("timeout_sec", 3.0,
                            "Maximum allowed age (s) of last diagnostics message")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr last_diag_msg_;
  rclcpp::Time last_msg_time_;
  bool received_first_msg_{false};
  std::string diagnostics_topic_;

  void diagnosticsCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);

  /// Returns true (and sets out_temp) when a temperature key is found in the values.
  bool extractTemperature(
    const diagnostic_msgs::msg::DiagnosticStatus & status,
    double & out_temp) const;
};

}  // namespace robot_custom_behaviour_tree_nodes

#endif  // ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__THERMAL_CHECK_ACTION_HPP_
