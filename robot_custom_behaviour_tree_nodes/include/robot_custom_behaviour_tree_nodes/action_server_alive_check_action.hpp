#ifndef ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__ACTION_SERVER_ALIVE_CHECK_ACTION_HPP_
#define ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__ACTION_SERVER_ALIVE_CHECK_ACTION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace robot_custom_behaviour_tree_nodes
{

/**
 * @brief BT Condition node that verifies a ROS 2 action server is reachable.
 *
 * Checks the ROS 2 service graph for the action server's send_goal endpoint
 * ({action_name}/_action/send_goal). No subscription is created; the check is
 * a non-blocking graph query suitable for tight BT monitor loops.
 *
 * Returns FAILURE when:
 *   - The send_goal service of the action server is not found in the graph.
 *
 * Input Ports:
 *   - action_name : Full action name (e.g. "/navigate_to_pose")
 *   - timeout_sec : Duration after which the server is considered offline for
 *                   escalated error logging (does not block; default: 3.0 s)
 */
class ActionServerAliveCheckAction : public BT::ConditionNode
{
public:
  ActionServerAliveCheckAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  ~ActionServerAliveCheckAction() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("action_name", "/navigate_to_pose",
                                 "Full ROS 2 action name to check"),
      BT::InputPort<double>("timeout_sec", 3.0,
                            "Seconds offline before escalated error log")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::string action_name_;
  rclcpp::Time first_failure_time_;
  bool tracking_failure_{false};
};

}  // namespace robot_custom_behaviour_tree_nodes

#endif  // ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__ACTION_SERVER_ALIVE_CHECK_ACTION_HPP_
