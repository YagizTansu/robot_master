#ifndef ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__TIME_SYNC_CHECK_ACTION_HPP_
#define ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__TIME_SYNC_CHECK_ACTION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace robot_custom_behaviour_tree_nodes
{

/**
 * @brief BT Condition node that verifies NTP clock synchronisation.
 *
 * Uses the Linux adjtimex() syscall to read the kernel NTP discipline offset.
 * This is the authoritative source of NTP health on Linux without requiring
 * any external process or topic.
 *
 * Returns FAILURE (with WARN log) when:
 *   - adjtimex() reports TIME_ERROR (clock not synchronised).
 *   - The absolute offset exceeds max_offset_ms milliseconds.
 *
 * The caller's Fallback uses AlwaysSuccess so the robot continues operating
 * (NTP may self-recover within seconds).
 *
 * Input Ports:
 *   - max_offset_ms : Maximum acceptable |NTP offset| in milliseconds (default: 100.0)
 */
class TimeSyncCheckAction : public BT::ConditionNode
{
public:
  TimeSyncCheckAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  ~TimeSyncCheckAction() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("max_offset_ms", 100.0,
                            "Maximum acceptable absolute NTP offset in milliseconds")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace robot_custom_behaviour_tree_nodes

#endif  // ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__TIME_SYNC_CHECK_ACTION_HPP_
