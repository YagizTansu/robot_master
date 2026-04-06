#ifndef ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__FLEET_MANAGER_CHECK_ACTION_HPP_
#define ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__FLEET_MANAGER_CHECK_ACTION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "std_msgs/msg/bool.hpp"

namespace robot_custom_behaviour_tree_nodes
{

/**
 * @brief BT Node to verify fleet manager connectivity via a heartbeat topic.
 *
 * The fleet manager is expected to publish a std_msgs/Bool message (value=true)
 * at a regular interval on the heartbeat topic. This node returns FAILURE when:
 *   - No message has been received within timeout_sec (fleet unreachable).
 *   - The received heartbeat value is false (fleet signalling not-ready).
 *
 * Unlike safety-critical nodes, fleet disconnection is treated as a WARNING-level
 * event: the BT wraps this node in a Fallback with AlwaysSuccess so that the robot
 * can continue operating autonomously when the fleet manager is temporarily offline.
 *
 * Input Ports:
 *   - heartbeat_topic : Topic name     (default: "/fleet_manager/heartbeat")
 *   - timeout_sec     : Maximum allowed silence before marking fleet offline (default: 5.0)
 */
class FleetManagerCheckAction : public BT::ConditionNode
{
public:
  FleetManagerCheckAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  ~FleetManagerCheckAction() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("heartbeat_topic", "/fleet_manager/heartbeat",
                                 "Topic for fleet manager heartbeat (std_msgs/Bool)"),
      BT::InputPort<double>("timeout_sec", 5.0,
                            "Seconds of silence before fleet is considered offline")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heartbeat_sub_;
  std_msgs::msg::Bool::SharedPtr last_heartbeat_msg_;
  rclcpp::Time last_msg_time_;
  bool received_first_msg_;
  std::string heartbeat_topic_;

  void heartbeatCallback(const std_msgs::msg::Bool::SharedPtr msg);
};

}  // namespace robot_custom_behaviour_tree_nodes

#endif  // ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__FLEET_MANAGER_CHECK_ACTION_HPP_
