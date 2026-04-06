#ifndef ROBOT_CUSTOM_BEHAVIOUR_EXECUTOR__MONITOR_BT_NODE_HPP_
#define ROBOT_CUSTOM_BEHAVIOUR_EXECUTOR__MONITOR_BT_NODE_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/loggers/abstract_logger.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <string>

namespace robot_custom_behaviour_executor
{

/**
 * @brief Publishes every BT node status change as a ROS 2 std_msgs/String
 * on the topic /bt_node_status with format "node_name|STATUS".
 * Used by bt_visualizer_pkg for real-time live monitoring.
 */
class ROS2StatusLogger : public BT::StatusChangeLogger
{
public:
  ROS2StatusLogger(
    BT::Tree & tree,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher)
  : BT::StatusChangeLogger(tree.rootNode()),
    publisher_(publisher)
  {}

  void callback(
    BT::Duration /*timestamp*/,
    const BT::TreeNode & node,
    BT::NodeStatus /*prev*/,
    BT::NodeStatus new_status) override
  {
    // Don't publish IDLE — it is a transient reset state between ticks.
    // Publishing IDLE would cause the visualizer color to flicker back to
    // the original theme color before Tkinter can render the real status.
    if (new_status == BT::NodeStatus::IDLE) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = node.name() + "|" + BT::toStr(new_status);
    publisher_->publish(msg);
  }

  void flush() override {}

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

class MonitorBTNode : public rclcpp::Node
{
public:
  MonitorBTNode();
  ~MonitorBTNode() = default;

private:
  void tickTree();

  BT::Tree tree_;
  BT::BehaviorTreeFactory factory_;
  // Publisher declared before loggers so it outlives them on destruction
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bt_status_publisher_;
  std::unique_ptr<BT::Groot2Publisher> groot2_publisher_;
  std::unique_ptr<ROS2StatusLogger> ros2_status_logger_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robot_custom_behaviour_executor

#endif  // ROBOT_CUSTOM_BEHAVIOUR_EXECUTOR__MONITOR_BT_NODE_HPP_
