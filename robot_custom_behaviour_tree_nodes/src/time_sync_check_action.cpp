#include "robot_custom_behaviour_tree_nodes/time_sync_check_action.hpp"

#include <cmath>
#include <sys/timex.h>

// adjtimex() TIME_ERROR constant — clock not synchronised to any reference
#ifndef TIME_ERROR
#define TIME_ERROR 5
#endif

namespace robot_custom_behaviour_tree_nodes
{

TimeSyncCheckAction::TimeSyncCheckAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  node_ = rclcpp::Node::make_shared("time_sync_check_node");

  RCLCPP_INFO(node_->get_logger(), "TimeSyncCheckAction initialized.");
}

BT::NodeStatus TimeSyncCheckAction::tick()
{
  double max_offset_ms = 100.0;
  getInput("max_offset_ms", max_offset_ms);

  struct timex tx {};
  int state = adjtimex(&tx);

  // TIME_ERROR (5) means the kernel NTP discipline is not synchronised
  if (state == TIME_ERROR) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "TimeSyncCheck: NTP not synchronised (adjtimex state=%d). "
      "TF extrapolation errors may occur.", state);
    return BT::NodeStatus::FAILURE;
  }

  // tx.offset is in microseconds normally, or nanoseconds when STA_NANO is set
  double offset_ms;
  if (tx.status & STA_NANO) {
    offset_ms = std::abs(static_cast<double>(tx.offset)) / 1.0e6;  // ns → ms
  } else {
    offset_ms = std::abs(static_cast<double>(tx.offset)) / 1.0e3;  // us → ms
  }

  if (offset_ms > max_offset_ms) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "TimeSyncCheck: NTP offset %.2f ms exceeds limit %.2f ms. "
      "TF transforms may be unreliable.", offset_ms, max_offset_ms);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
    "TimeSyncCheck: NTP OK — offset %.3f ms (limit: %.1f ms).",
    offset_ms, max_offset_ms);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_custom_behaviour_tree_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<
    robot_custom_behaviour_tree_nodes::TimeSyncCheckAction>("TimeSyncCheck");
}
