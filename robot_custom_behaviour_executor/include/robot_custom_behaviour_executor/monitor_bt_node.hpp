#ifndef ROBOT_CUSTOM_BEHAVIOUR_EXECUTOR__MONITOR_BT_NODE_HPP_
#define ROBOT_CUSTOM_BEHAVIOUR_EXECUTOR__MONITOR_BT_NODE_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace robot_custom_behaviour_executor
{

class MonitorBTNode : public rclcpp::Node
{
public:
  MonitorBTNode();
  ~MonitorBTNode() = default;

private:
  void tickTree();

  BT::Tree tree_;
  BT::BehaviorTreeFactory factory_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robot_custom_behaviour_executor

#endif  // ROBOT_CUSTOM_BEHAVIOUR_EXECUTOR__MONITOR_BT_NODE_HPP_
