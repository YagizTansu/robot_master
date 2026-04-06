#include "robot_custom_behaviour_tree_nodes/fleet_manager_check_action.hpp"

namespace robot_custom_behaviour_tree_nodes
{

FleetManagerCheckAction::FleetManagerCheckAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf),
  received_first_msg_(false)
{
  node_ = rclcpp::Node::make_shared("fleet_manager_check_action_node");

  if (!getInput("heartbeat_topic", heartbeat_topic_)) {
    heartbeat_topic_ = "/fleet_manager/heartbeat";
  }

  heartbeat_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    heartbeat_topic_,
    rclcpp::QoS(10).best_effort(),
    std::bind(&FleetManagerCheckAction::heartbeatCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(),
    "FleetManagerCheckAction initialized on topic: %s", heartbeat_topic_.c_str());
}

void FleetManagerCheckAction::heartbeatCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  last_heartbeat_msg_ = msg;
  last_msg_time_ = node_->get_clock()->now();
  received_first_msg_ = true;
}

BT::NodeStatus FleetManagerCheckAction::tick()
{
  double timeout_sec = 5.0;
  getInput("timeout_sec", timeout_sec);

  rclcpp::spin_some(node_);

  // No heartbeat ever received
  if (!received_first_msg_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "Fleet manager heartbeat not received on '%s'. Fleet offline or not started.",
      heartbeat_topic_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Heartbeat timeout — fleet went offline
  double age_sec = (node_->get_clock()->now() - last_msg_time_).seconds();
  if (age_sec > timeout_sec) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
      "Fleet manager heartbeat timed out! Last heartbeat was %.2f s ago (limit: %.2f s). "
      "Continuing in autonomous mode.",
      age_sec, timeout_sec);
    return BT::NodeStatus::FAILURE;
  }

  // Fleet explicitly signalled not-ready
  if (!last_heartbeat_msg_->data) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
      "Fleet manager heartbeat is false — fleet not ready.");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
    "Fleet manager connected and ready.");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_custom_behaviour_tree_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<
    robot_custom_behaviour_tree_nodes::FleetManagerCheckAction
  >("FleetManagerCheck");
}
