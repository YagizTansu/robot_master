#include "robot_custom_behaviour_tree_nodes/emergency_stop_action.hpp"

namespace robot_custom_behaviour_tree_nodes
{

EmergencyStopAction::EmergencyStopAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf),
  initialized_(false),
  last_emergency_state_(false)
{
  // Create ROS node in constructor
  node_ = rclcpp::Node::make_shared("emergency_stop_action_node");
  
  // Get emergency topic parameter
  if (!getInput("emergency_topic", emergency_topic_)) {
    emergency_topic_ = "/emergency_stop";
  }
  
  // Create emergency stop subscriber
  emergency_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    emergency_topic_,
    rclcpp::QoS(10).reliable(),
    std::bind(&EmergencyStopAction::emergencyCallback, this, std::placeholders::_1));
    
  RCLCPP_INFO(node_->get_logger(), 
    "EmergencyStopAction initialized with topic: %s", emergency_topic_.c_str());
  
  initialized_ = true;
}

void EmergencyStopAction::emergencyCallback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  last_emergency_msg_ = msg;
  
  // Log state changes
  if (msg->data != last_emergency_state_) {
    if (msg->data) {
      RCLCPP_WARN(node_->get_logger(), "EMERGENCY STOP ACTIVATED! Robot will halt.");
    } else {
      RCLCPP_INFO(node_->get_logger(), "Emergency stop released. Robot can continue.");
    }
    last_emergency_state_ = msg->data;
  }
}

BT::NodeStatus EmergencyStopAction::tick()
{
  // Get input parameters
  bool button_pressed_value;
  if (!getInput("button_pressed_value", button_pressed_value)) {
    button_pressed_value = true;
  }
  
  // Process ROS 2 messages (non-blocking)
  rclcpp::spin_some(node_);
  
  // If no emergency message received yet, assume it's safe to continue (SUCCESS)
  if (!last_emergency_msg_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "No emergency stop message received yet. Assuming safe to continue...");
    return BT::NodeStatus::SUCCESS;
  }
  
  // Check if emergency button is pressed
  bool is_emergency_active = (last_emergency_msg_->data == button_pressed_value);
  
  if (is_emergency_active) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "EMERGENCY STOP IS ACTIVE! Waiting for release...");
    return BT::NodeStatus::FAILURE;
  }
  
  RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
    "Emergency stop check passed. Continue operation.");
  
  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_custom_behaviour_tree_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robot_custom_behaviour_tree_nodes::EmergencyStopAction>("EmergencyStop");
}
