#ifndef ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__EMERGENCY_STOP_ACTION_HPP_
#define ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__EMERGENCY_STOP_ACTION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "std_msgs/msg/bool.hpp"

namespace robot_custom_behaviour_tree_nodes
{

/**
 * @brief Custom BT Node to check emergency stop button status
 * 
 * This node monitors an emergency stop button and returns FAILURE if the button
 * is pressed (true), forcing the robot to wait. Returns SUCCESS when button is released.
 * 
 * Input Ports:
 *   - emergency_topic: Topic publishing emergency stop state (default: "/emergency_stop")
 *   - button_pressed_value: Value indicating button is pressed (default: true)
 */
class EmergencyStopAction : public BT::ConditionNode
{
public:
  EmergencyStopAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  ~EmergencyStopAction() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("emergency_topic", "/emergency_stop", 
                                  "Topic to subscribe for emergency stop state"),
      BT::InputPort<bool>("button_pressed_value", true, 
                         "Value indicating button is pressed (true or false)")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;
  std_msgs::msg::Bool::SharedPtr last_emergency_msg_;
  bool initialized_;
  std::string emergency_topic_;
  bool last_emergency_state_;
  
  void emergencyCallback(const std_msgs::msg::Bool::SharedPtr msg);
};

}  // namespace robot_custom_behaviour_tree_nodes

#endif  // ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__EMERGENCY_STOP_ACTION_HPP_
