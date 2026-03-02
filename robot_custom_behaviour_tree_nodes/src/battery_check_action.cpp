#include "robot_custom_behaviour_tree_nodes/battery_check_action.hpp"

namespace robot_custom_behaviour_tree_nodes
{

BatteryCheckAction::BatteryCheckAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf),
  initialized_(false)
{
  // Create ROS node in constructor
  node_ = rclcpp::Node::make_shared("battery_check_action_node");
  
  // Get battery topic parameter
  if (!getInput("battery_topic", battery_topic_)) {
    battery_topic_ = "/battery_status";
  }
  
  // Create battery subscriber
  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    battery_topic_,
    rclcpp::QoS(10).best_effort(),
    std::bind(&BatteryCheckAction::batteryCallback, this, std::placeholders::_1));
    
  RCLCPP_INFO(node_->get_logger(), 
    "BatteryCheckAction initialized with topic: %s", battery_topic_.c_str());
  
  initialized_ = true;
}

void BatteryCheckAction::batteryCallback(
  const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  last_battery_msg_ = msg;
}

BT::NodeStatus BatteryCheckAction::tick()
{
  // Get input parameters
  double min_battery_percentage;
  if (!getInput("min_battery_percentage", min_battery_percentage)) {
    min_battery_percentage = 20.0;
  }
  
  // Process ROS 2 messages (non-blocking)
  rclcpp::spin_some(node_);
  
  // If no battery message received yet, continue navigation (SUCCESS)
  if (!last_battery_msg_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "No battery message received yet. Continuing navigation...");
    return BT::NodeStatus::SUCCESS;
  }
  
  // Calculate battery percentage (0-100 range)
  double battery_percentage = last_battery_msg_->percentage * 100.0;
  
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
    "Battery Check - Current: %.1f%%, Minimum Required: %.1f%%", 
    battery_percentage, min_battery_percentage);
  
  // Check if battery is sufficient - FAILURE only if below minimum threshold
  if (battery_percentage < min_battery_percentage) {
    RCLCPP_ERROR(node_->get_logger(), 
      "Battery level too low! %.1f%% < %.1f%% - STOPPING NAVIGATION", 
      battery_percentage, min_battery_percentage);
    return BT::NodeStatus::FAILURE;
  }
  
  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_custom_behaviour_tree_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robot_custom_behaviour_tree_nodes::BatteryCheckAction>("BatteryCheck");
}
