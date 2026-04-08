#ifndef ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__TOPIC_ALIVE_CHECK_ACTION_HPP_
#define ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__TOPIC_ALIVE_CHECK_ACTION_HPP_

#include <atomic>
#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace robot_custom_behaviour_tree_nodes
{

/**
 * @brief BT Condition node that verifies a ROS 2 topic is actively publishing.
 *
 * Uses a generic subscription (type resolved at runtime from the graph) to track
 * the wall-clock time of the last received message. Returns FAILURE when:
 *   - No publisher is present on the topic.
 *   - No message has been received within timeout_sec.
 *   - The topic type cannot be resolved from the ROS 2 graph.
 *
 * Because the node creates a separate rclcpp::Node per instance, multiple
 * TopicAliveCheck nodes can coexist without name collisions (static counter).
 *
 * Input Ports:
 *   - topic       : Full topic name to monitor (e.g. "/scan")
 *   - timeout_sec : Maximum allowed age of the last message (default: 2.0 s)
 */
class TopicAliveCheckAction : public BT::ConditionNode
{
public:
  TopicAliveCheckAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  ~TopicAliveCheckAction() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "", "ROS 2 topic to check for liveness"),
      BT::InputPort<double>("timeout_sec", 2.0, "Max allowed age (s) of last received message")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::GenericSubscription::SharedPtr sub_;
  rclcpp::Time last_msg_time_;
  bool received_first_msg_{false};
  bool subscribed_{false};
  std::string topic_;

  static std::atomic<int> instance_counter_;

  /// Lazily resolves the topic type from the ROS 2 graph and creates a
  /// GenericSubscription. Called once per instance on the first tick.
  void trySubscribe();
};

}  // namespace robot_custom_behaviour_tree_nodes

#endif  // ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__TOPIC_ALIVE_CHECK_ACTION_HPP_
