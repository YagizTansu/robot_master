#ifndef ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__PROXIMITY_CHECK_ACTION_HPP_
#define ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__PROXIMITY_CHECK_ACTION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot_custom_behaviour_tree_nodes
{

/**
 * @brief BT Condition node that detects obstacles within a safety radius.
 *
 * Subscribes to a LaserScan topic and checks every valid range reading against
 * two configurable radii:
 *   - warning_radius_m  : Outer perimeter — logs WARN, returns FAILURE.
 *   - safety_radius_m   : Inner perimeter — logs ERROR, returns FAILURE.
 *
 * Returns SUCCESS when no obstacle is detected within warning_radius_m.
 * Returns FAILURE on first obstacle found (caller's Fallback handles graceful
 * degradation — see BT layer 6).
 *
 * Fail-safe: no scan message received → FAILURE.
 *
 * Input Ports:
 *   - laser_topic      : LaserScan topic (default: "/scan")
 *   - safety_radius_m  : Inner safety zone radius in metres (default: 0.4)
 *   - warning_radius_m : Outer warning zone radius in metres (default: 0.8)
 *   - timeout_sec      : Maximum allowed age of last scan (default: 1.0 s)
 */
class ProximityCheckAction : public BT::ConditionNode
{
public:
  ProximityCheckAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  ~ProximityCheckAction() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("laser_topic", "/scan", "LaserScan topic to monitor"),
      BT::InputPort<double>("safety_radius_m", 0.4,
                            "Inner safety zone radius in metres — triggers ERROR log"),
      BT::InputPort<double>("warning_radius_m", 0.8,
                            "Outer warning zone radius in metres — triggers WARN log"),
      BT::InputPort<double>("timeout_sec", 1.0,
                            "Maximum allowed age (s) of last laser scan message")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_msg_;
  rclcpp::Time last_msg_time_;
  bool received_first_msg_{false};
  std::string laser_topic_;

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

}  // namespace robot_custom_behaviour_tree_nodes

#endif  // ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__PROXIMITY_CHECK_ACTION_HPP_
