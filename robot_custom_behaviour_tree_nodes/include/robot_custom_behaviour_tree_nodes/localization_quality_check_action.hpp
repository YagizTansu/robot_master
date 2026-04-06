#ifndef ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__LOCALIZATION_QUALITY_CHECK_ACTION_HPP_
#define ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__LOCALIZATION_QUALITY_CHECK_ACTION_HPP_

#include <string>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace robot_custom_behaviour_tree_nodes
{

/**
 * @brief BT Node to verify AMCL localization quality via covariance bounds.
 *
 * Subscribes to a PoseWithCovarianceStamped topic (e.g. /amcl_pose) and
 * returns FAILURE when:
 *   - No message has been received yet (fail-safe: unknown pose = unsafe).
 *   - The last message arrived more than timeout_sec ago (topic stale).
 *   - The X or Y position covariance exceeds max_covariance.
 *
 * Input Ports:
 *   - amcl_topic        : Topic name  (default: "/amcl_pose")
 *   - max_covariance    : Maximum acceptable positional covariance (default: 0.5)
 *   - timeout_sec       : Maximum allowed age of the last pose message (default: 2.0)
 */
class LocalizationQualityCheckAction : public BT::ConditionNode
{
public:
  LocalizationQualityCheckAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  ~LocalizationQualityCheckAction() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("amcl_topic", "/amcl_pose",
                                 "PoseWithCovarianceStamped topic for localization"),
      BT::InputPort<double>("max_covariance", 0.5,
                            "Maximum acceptable X/Y positional covariance"),
      BT::InputPort<double>("timeout_sec", 2.0,
                            "Maximum allowed age (seconds) of the last pose message")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_pose_msg_;
  rclcpp::Time last_msg_time_;
  bool received_first_msg_;
  std::string amcl_topic_;

  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
};

}  // namespace robot_custom_behaviour_tree_nodes

#endif  // ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__LOCALIZATION_QUALITY_CHECK_ACTION_HPP_
