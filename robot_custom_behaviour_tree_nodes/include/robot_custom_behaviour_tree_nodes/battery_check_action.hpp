#ifndef ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__BATTERY_CHECK_ACTION_HPP_
#define ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__BATTERY_CHECK_ACTION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "sensor_msgs/msg/battery_state.hpp"

namespace robot_custom_behaviour_tree_nodes
{

/**
 * @brief Custom BT Node to check battery level
 * 
 * This node monitors the battery level and returns FAILURE if below
 * a specified threshold, otherwise returns SUCCESS.
 * 
 * Input Ports:
 *   - battery_topic: Topic publishing battery state (default: "/battery_status")
 *   - min_battery_percentage: Minimum acceptable battery percentage (default: 20.0)
 */
class BatteryCheckAction : public BT::ConditionNode
{
public:
  BatteryCheckAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  ~BatteryCheckAction() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("battery_topic", "/battery_status", 
                                  "Topic to subscribe for battery state"),
      BT::InputPort<double>("min_battery_percentage", 20.0, 
                           "Minimum battery percentage required")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  sensor_msgs::msg::BatteryState::SharedPtr last_battery_msg_;
  bool initialized_;
  std::string battery_topic_;
  
  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
};

}  // namespace robot_custom_behaviour_tree_nodes

#endif  // ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__BATTERY_CHECK_ACTION_HPP_
