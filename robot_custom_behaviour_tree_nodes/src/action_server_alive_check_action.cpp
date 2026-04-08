#include "robot_custom_behaviour_tree_nodes/action_server_alive_check_action.hpp"

namespace robot_custom_behaviour_tree_nodes
{

ActionServerAliveCheckAction::ActionServerAliveCheckAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  node_ = rclcpp::Node::make_shared("action_server_alive_check_node");

  if (!getInput("action_name", action_name_)) {
    action_name_ = "/navigate_to_pose";
  }

  RCLCPP_INFO(node_->get_logger(),
    "ActionServerAliveCheckAction initialized — checking action: '%s'",
    action_name_.c_str());
}

BT::NodeStatus ActionServerAliveCheckAction::tick()
{
  double timeout_sec = 3.0;
  getInput("timeout_sec", timeout_sec);

  rclcpp::spin_some(node_);

  // Action servers register a send_goal service at {action_name}/_action/send_goal.
  // count_services() is a non-blocking DDS graph query — safe in tight loops.
  const std::string send_goal_svc = action_name_ + "/_action/send_goal";
  bool server_alive = node_->count_services(send_goal_svc) > 0;

  if (!server_alive) {
    if (!tracking_failure_) {
      tracking_failure_ = true;
      first_failure_time_ = node_->get_clock()->now();
      RCLCPP_WARN(node_->get_logger(),
        "ActionServerAliveCheck: action server '%s' is not reachable.",
        action_name_.c_str());
    } else {
      double down_sec = (node_->get_clock()->now() - first_failure_time_).seconds();
      if (down_sec >= timeout_sec) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
          "ActionServerAliveCheck: action server '%s' has been DOWN for %.1f s!",
          action_name_.c_str(), down_sec);
      }
    }
    return BT::NodeStatus::FAILURE;
  }

  if (tracking_failure_) {
    RCLCPP_INFO(node_->get_logger(),
      "ActionServerAliveCheck: action server '%s' is back online.",
      action_name_.c_str());
    tracking_failure_ = false;
  }

  RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
    "ActionServerAliveCheck: '%s' OK.", action_name_.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_custom_behaviour_tree_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<
    robot_custom_behaviour_tree_nodes::ActionServerAliveCheckAction>("ActionServerAliveCheck");
}
