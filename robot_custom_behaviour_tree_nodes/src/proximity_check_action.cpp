#include "robot_custom_behaviour_tree_nodes/proximity_check_action.hpp"

#include <cmath>
#include <limits>

namespace robot_custom_behaviour_tree_nodes
{

ProximityCheckAction::ProximityCheckAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  node_ = rclcpp::Node::make_shared("proximity_check_node");

  if (!getInput("laser_topic", laser_topic_)) {
    laser_topic_ = "/scan";
  }

  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    laser_topic_,
    rclcpp::QoS(10).best_effort(),
    std::bind(&ProximityCheckAction::scanCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(),
    "ProximityCheckAction initialized on topic: '%s'", laser_topic_.c_str());
}

void ProximityCheckAction::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  last_scan_msg_ = msg;
  last_msg_time_ = node_->get_clock()->now();
  received_first_msg_ = true;
}

BT::NodeStatus ProximityCheckAction::tick()
{
  double safety_radius_m = 0.4;
  double warning_radius_m = 0.8;
  double timeout_sec = 1.0;

  getInput("safety_radius_m", safety_radius_m);
  getInput("warning_radius_m", warning_radius_m);
  getInput("timeout_sec", timeout_sec);

  rclcpp::spin_some(node_);

  // Fail-safe: no scan received
  if (!received_first_msg_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "ProximityCheck: no laser scan received on '%s'.", laser_topic_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Fail-safe: stale scan
  double age_sec = (node_->get_clock()->now() - last_msg_time_).seconds();
  if (age_sec > timeout_sec) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
      "ProximityCheck: laser scan STALE on '%s' (%.2f s ago).",
      laser_topic_.c_str(), age_sec);
    return BT::NodeStatus::FAILURE;
  }

  // Scan all valid range readings
  double min_range = std::numeric_limits<double>::max();
  for (const float r : last_scan_msg_->ranges) {
    if (!std::isfinite(r)) {
      continue;
    }
    if (r < last_scan_msg_->range_min || r > last_scan_msg_->range_max) {
      continue;
    }
    if (static_cast<double>(r) < min_range) {
      min_range = static_cast<double>(r);
    }
  }

  if (min_range == std::numeric_limits<double>::max()) {
    // All readings were invalid — treat as clear
    return BT::NodeStatus::SUCCESS;
  }

  if (min_range < safety_radius_m) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "ProximityCheck: obstacle in SAFETY zone! Closest: %.2f m (limit: %.2f m).",
      min_range, safety_radius_m);
    return BT::NodeStatus::FAILURE;
  }

  if (min_range < warning_radius_m) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "ProximityCheck: obstacle in WARNING zone. Closest: %.2f m (limit: %.2f m).",
      min_range, warning_radius_m);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
    "ProximityCheck: clear. Closest obstacle: %.2f m.", min_range);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_custom_behaviour_tree_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<
    robot_custom_behaviour_tree_nodes::ProximityCheckAction>("ProximityCheck");
}
