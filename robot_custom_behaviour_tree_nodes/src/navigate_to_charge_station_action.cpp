#include "robot_custom_behaviour_tree_nodes/navigate_to_charge_station_action.hpp"

namespace robot_custom_behaviour_tree_nodes
{

NavigateToChargeStationAction::NavigateToChargeStationAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(xml_tag_name, conf),
  goal_sent_(false),
  goal_result_available_(false),
  result_code_(rclcpp_action::ResultCode::UNKNOWN)
{
  // Create ROS node
  node_ = rclcpp::Node::make_shared("navigate_to_charge_station_action_node");
  
  // Create action client for Nav2's NavigateToPose
  action_client_ = rclcpp_action::create_client<NavigateToPose>(
    node_,
    "navigate_to_pose");
  
  RCLCPP_INFO(node_->get_logger(), "NavigateToChargeStationAction initialized");
}

BT::NodeStatus NavigateToChargeStationAction::onStart()
{
  RCLCPP_INFO(node_->get_logger(), "NavigateToChargeStationAction started");
  
  // Reset state
  goal_sent_ = false;
  goal_result_available_ = false;
  result_code_ = rclcpp_action::ResultCode::UNKNOWN;
  goal_handle_.reset();
  
  // Get action timeout from input port
  if (!getInput("action_timeout", action_timeout_)) {
    action_timeout_ = 300.0;  // Default 5 minutes
  }
  
  // Get server timeout
  double server_timeout = 10.0;
  getInput("server_timeout", server_timeout);
  
  // Check if action server is available
  if (!waitForActionServer(server_timeout)) {
    RCLCPP_ERROR(node_->get_logger(), 
      "NavigateToPose action server not available after waiting for %.1f seconds", 
      server_timeout);
    return BT::NodeStatus::FAILURE;
  }
  
  // Get charge station pose from blackboard
  geometry_msgs::msg::PoseStamped charge_station_pose;
  if (!getInput("charge_station_pose", charge_station_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get charge_station_pose from blackboard");
    return BT::NodeStatus::FAILURE;
  }
  
  RCLCPP_INFO(node_->get_logger(), 
    "Navigating to charge station at [%.2f, %.2f] in frame '%s'",
    charge_station_pose.pose.position.x,
    charge_station_pose.pose.position.y,
    charge_station_pose.header.frame_id.c_str());
  
  // Send navigation goal
  if (!sendNavigationGoal(charge_station_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to send navigation goal");
    return BT::NodeStatus::FAILURE;
  }
  
  start_time_ = node_->get_clock()->now();
  goal_sent_ = true;
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToChargeStationAction::onRunning()
{
  // Spin the node to process callbacks
  rclcpp::spin_some(node_);
  
  // Check for timeout
  auto elapsed_time = (node_->get_clock()->now() - start_time_).seconds();
  if (elapsed_time > action_timeout_) {
    RCLCPP_ERROR(node_->get_logger(), 
      "Navigation timeout after %.1f seconds", elapsed_time);
    
    // Cancel the goal if it's still active
    if (goal_handle_) {
      auto cancel_future = action_client_->async_cancel_goal(goal_handle_);
      RCLCPP_WARN(node_->get_logger(), "Cancelling navigation goal due to timeout");
    }
    
    return BT::NodeStatus::FAILURE;
  }
  
  // Check if result is available
  if (goal_result_available_) {
    if (result_code_ == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(node_->get_logger(), 
        "Successfully navigated to charge station in %.1f seconds", elapsed_time);
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(node_->get_logger(), 
        "Navigation failed with result code: %d", static_cast<int>(result_code_));
      return BT::NodeStatus::FAILURE;
    }
  }
  
  // Still running
  return BT::NodeStatus::RUNNING;
}

void NavigateToChargeStationAction::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "NavigateToChargeStationAction halted");
  
  // Cancel the goal if it's active
  if (goal_handle_ && goal_sent_ && !goal_result_available_) {
    auto cancel_future = action_client_->async_cancel_goal(goal_handle_);
    RCLCPP_INFO(node_->get_logger(), "Cancelling navigation goal");
  }
  
  // Reset state
  goal_sent_ = false;
  goal_result_available_ = false;
  goal_handle_.reset();
}

bool NavigateToChargeStationAction::waitForActionServer(double timeout)
{
  RCLCPP_INFO(node_->get_logger(), 
    "Waiting for NavigateToPose action server (timeout: %.1f seconds)...", timeout);
  
  auto start = node_->get_clock()->now();
  
  while (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(), "ROS node interrupted while waiting for action server");
      return false;
    }
    
    auto elapsed = (node_->get_clock()->now() - start).seconds();
    if (elapsed > timeout) {
      return false;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Waiting... (%.1f/%.1f seconds)", elapsed, timeout);
  }
  
  RCLCPP_INFO(node_->get_logger(), "NavigateToPose action server is available");
  return true;
}

bool NavigateToChargeStationAction::sendNavigationGoal(
  const geometry_msgs::msg::PoseStamped & target_pose)
{
  // Create goal message
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = target_pose;
  
  // Set goal options with callbacks
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  
  send_goal_options.goal_response_callback =
    std::bind(&NavigateToChargeStationAction::goalResponseCallback, this, std::placeholders::_1);
  
  send_goal_options.feedback_callback =
    std::bind(&NavigateToChargeStationAction::feedbackCallback, this, 
      std::placeholders::_1, std::placeholders::_2);
  
  send_goal_options.result_callback =
    std::bind(&NavigateToChargeStationAction::resultCallback, this, std::placeholders::_1);
  
  // Send goal asynchronously
  RCLCPP_INFO(node_->get_logger(), "Sending navigation goal to action server");
  auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);
  
  // Wait for goal to be accepted (with timeout)
  auto wait_result = rclcpp::spin_until_future_complete(
    node_, 
    goal_handle_future, 
    std::chrono::seconds(5));
  
  if (wait_result != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to send goal to action server");
    return false;
  }
  
  goal_handle_ = goal_handle_future.get();
  if (!goal_handle_) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by action server");
    return false;
  }
  
  RCLCPP_INFO(node_->get_logger(), "Goal accepted by action server");
  return true;
}

void NavigateToChargeStationAction::goalResponseCallback(
  const GoalHandleNavigateToPose::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    goal_result_available_ = true;
    result_code_ = rclcpp_action::ResultCode::UNKNOWN;
  } else {
    RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void NavigateToChargeStationAction::feedbackCallback(
  GoalHandleNavigateToPose::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  // Log current position
  RCLCPP_DEBUG(node_->get_logger(), 
    "Current position: [%.2f, %.2f], Distance remaining: %.2f, ETA: %.1f seconds",
    feedback->current_pose.pose.position.x,
    feedback->current_pose.pose.position.y,
    feedback->distance_remaining,
    feedback->estimated_time_remaining.sec + feedback->estimated_time_remaining.nanosec * 1e-9);
}

void NavigateToChargeStationAction::resultCallback(
  const GoalHandleNavigateToPose::WrappedResult & result)
{
  goal_result_available_ = true;
  result_code_ = result.code;
  
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "Navigation succeeded!");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "Navigation was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(node_->get_logger(), "Navigation was canceled");
      break;
    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code: %d", static_cast<int>(result.code));
      break;
  }
}

}  // namespace robot_custom_behaviour_tree_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robot_custom_behaviour_tree_nodes::NavigateToChargeStationAction>(
    "NavigateToChargeStation");
}
