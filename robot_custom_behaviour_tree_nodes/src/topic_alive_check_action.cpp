#include "robot_custom_behaviour_tree_nodes/topic_alive_check_action.hpp"

#include <algorithm>
#include <string>

namespace robot_custom_behaviour_tree_nodes
{

std::atomic<int> TopicAliveCheckAction::instance_counter_{0};

TopicAliveCheckAction::TopicAliveCheckAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  int id = instance_counter_.fetch_add(1);
  node_ = rclcpp::Node::make_shared("topic_alive_check_node_" + std::to_string(id));

  if (!getInput("topic", topic_) || topic_.empty()) {
    RCLCPP_ERROR(node_->get_logger(),
      "TopicAliveCheck '%s': required input port 'topic' is not set!",
      xml_tag_name.c_str());
  }

  RCLCPP_INFO(node_->get_logger(),
    "TopicAliveCheckAction[%d] initialized — monitoring topic: '%s'",
    id, topic_.c_str());
}

void TopicAliveCheckAction::trySubscribe()
{
  auto topic_types = node_->get_topic_names_and_types();
  auto it = topic_types.find(topic_);

  if (it == topic_types.end() || it->second.empty()) {
    RCLCPP_DEBUG(node_->get_logger(),
      "Topic '%s' not yet found in the graph.", topic_.c_str());
    return;
  }

  const std::string & type = it->second[0];

  sub_ = node_->create_generic_subscription(
    topic_,
    type,
    rclcpp::QoS(10).best_effort(),
    [this](std::shared_ptr<rclcpp::SerializedMessage>) {
      last_msg_time_ = node_->get_clock()->now();
      received_first_msg_ = true;
    });

  subscribed_ = true;
  RCLCPP_INFO(node_->get_logger(),
    "TopicAliveCheck: subscribed to '%s' [type: %s]", topic_.c_str(), type.c_str());
}

BT::NodeStatus TopicAliveCheckAction::tick()
{
  double timeout_sec = 2.0;
  getInput("timeout_sec", timeout_sec);

  rclcpp::spin_some(node_);

  if (!subscribed_) {
    trySubscribe();
  }

  // No publisher visible in graph at all
  if (!subscribed_ && node_->count_publishers(topic_) == 0) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "TopicAliveCheck: no publisher found on '%s'.", topic_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!received_first_msg_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "TopicAliveCheck: waiting for first message on '%s'.", topic_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  double age_sec = (node_->get_clock()->now() - last_msg_time_).seconds();
  if (age_sec > timeout_sec) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
      "TopicAliveCheck: topic '%s' is STALE — last message %.2f s ago (limit: %.2f s).",
      topic_.c_str(), age_sec, timeout_sec);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
    "TopicAliveCheck: '%s' OK (age: %.3f s).", topic_.c_str(), age_sec);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_custom_behaviour_tree_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<
    robot_custom_behaviour_tree_nodes::TopicAliveCheckAction>("TopicAliveCheck");
}
