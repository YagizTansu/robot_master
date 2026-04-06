#include "robot_custom_behaviour_tree_nodes/localization_quality_check_action.hpp"

namespace robot_custom_behaviour_tree_nodes
{

LocalizationQualityCheckAction::LocalizationQualityCheckAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf),
  received_first_msg_(false)
{
  node_ = rclcpp::Node::make_shared("localization_quality_check_action_node");

  if (!getInput("amcl_topic", amcl_topic_)) {
    amcl_topic_ = "/amcl_pose";
  }

  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    amcl_topic_,
    rclcpp::QoS(10).best_effort(),
    std::bind(&LocalizationQualityCheckAction::poseCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(),
    "LocalizationQualityCheckAction initialized on topic: %s", amcl_topic_.c_str());
}

void LocalizationQualityCheckAction::poseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  last_pose_msg_ = msg;
  last_msg_time_ = node_->get_clock()->now();
  received_first_msg_ = true;
}

BT::NodeStatus LocalizationQualityCheckAction::tick()
{
  double max_covariance = 0.5;
  double timeout_sec = 2.0;
  getInput("max_covariance", max_covariance);
  getInput("timeout_sec", timeout_sec);

  rclcpp::spin_some(node_);

  // Fail-safe: no pose received at all
  if (!received_first_msg_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "No localization message received on '%s'. Robot pose unknown — UNSAFE.",
      amcl_topic_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Fail-safe: pose message is stale
  double age_sec = (node_->get_clock()->now() - last_msg_time_).seconds();
  if (age_sec > timeout_sec) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
      "Localization topic stale! Last message was %.2f s ago (limit: %.2f s).",
      age_sec, timeout_sec);
    return BT::NodeStatus::FAILURE;
  }

  // Check X variance (index 0) and Y variance (index 7) from the 6x6 covariance matrix
  const auto & cov = last_pose_msg_->pose.covariance;
  double cov_x = cov[0];
  double cov_y = cov[7];

  if (cov_x > max_covariance || cov_y > max_covariance) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
      "Poor localization quality! cov_x=%.4f, cov_y=%.4f (max: %.4f). "
      "Navigation unsafe until localization improves.",
      cov_x, cov_y, max_covariance);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
    "Localization quality OK. cov_x=%.4f, cov_y=%.4f", cov_x, cov_y);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_custom_behaviour_tree_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<
    robot_custom_behaviour_tree_nodes::LocalizationQualityCheckAction
  >("LocalizationQualityCheck");
}
