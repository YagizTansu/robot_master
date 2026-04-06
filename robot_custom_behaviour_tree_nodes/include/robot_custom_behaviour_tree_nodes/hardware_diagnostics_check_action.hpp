#ifndef ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__HARDWARE_DIAGNOSTICS_CHECK_ACTION_HPP_
#define ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__HARDWARE_DIAGNOSTICS_CHECK_ACTION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

namespace robot_custom_behaviour_tree_nodes
{

/**
 * @brief BT Node to monitor hardware health via ROS 2 diagnostics.
 *
 * Subscribes to a DiagnosticArray topic (typically /diagnostics_agg, the output
 * of diagnostic_aggregator) and returns FAILURE when:
 *   - No message has been received yet (fail-safe).
 *   - The last message arrived more than timeout_sec ago (topic stale).
 *   - Any DiagnosticStatus in the array has level >= fail_on_level.
 *     Levels: 0=OK, 1=WARN, 2=ERROR, 3=STALE.
 *
 * Input Ports:
 *   - diagnostics_topic : Topic name        (default: "/diagnostics_agg")
 *   - timeout_sec       : Maximum message age (default: 3.0)
 *   - fail_on_level     : Minimum level that triggers FAILURE (default: 2 = ERROR)
 */
class HardwareDiagnosticsCheckAction : public BT::ConditionNode
{
public:
  HardwareDiagnosticsCheckAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  ~HardwareDiagnosticsCheckAction() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("diagnostics_topic", "/diagnostics_agg",
                                 "DiagnosticArray topic (aggregated diagnostics)"),
      BT::InputPort<double>("timeout_sec", 3.0,
                            "Maximum allowed age (seconds) of the last diagnostics message"),
      BT::InputPort<int>("fail_on_level", 2,
                         "Minimum DiagnosticStatus level to trigger FAILURE (0=OK,1=WARN,2=ERROR,3=STALE)")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr last_diag_msg_;
  rclcpp::Time last_msg_time_;
  bool received_first_msg_;
  std::string diagnostics_topic_;

  void diagnosticsCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
};

}  // namespace robot_custom_behaviour_tree_nodes

#endif  // ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__HARDWARE_DIAGNOSTICS_CHECK_ACTION_HPP_
