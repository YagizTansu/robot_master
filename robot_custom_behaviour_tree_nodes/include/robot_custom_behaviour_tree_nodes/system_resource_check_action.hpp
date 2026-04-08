#ifndef ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__SYSTEM_RESOURCE_CHECK_ACTION_HPP_
#define ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__SYSTEM_RESOURCE_CHECK_ACTION_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace robot_custom_behaviour_tree_nodes
{

/**
 * @brief BT Condition node that monitors CPU, RAM, and disk usage.
 *
 * Reads system metrics directly from Linux /proc pseudo-filesystem and statvfs:
 *   - CPU  : /proc/stat  — computes inter-tick delta for accurate utilisation
 *   - RAM  : /proc/meminfo — uses MemAvailable for realistic free-memory estimate
 *   - Disk : statvfs("/")  — checks available blocks against min_disk_free_mb
 *
 * Returns FAILURE (with WARN log) when any threshold is exceeded.
 * The caller's Fallback uses AlwaysSuccess so the robot continues operating.
 *
 * Input Ports:
 *   - max_cpu_percent    : CPU utilisation limit (default: 90.0 %)
 *   - max_memory_percent : RAM utilisation limit (default: 85.0 %)
 *   - min_disk_free_mb   : Minimum free disk space (default: 500 MB)
 */
class SystemResourceCheckAction : public BT::ConditionNode
{
public:
  SystemResourceCheckAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  ~SystemResourceCheckAction() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("max_cpu_percent", 90.0, "CPU utilisation upper limit (%)"),
      BT::InputPort<double>("max_memory_percent", 85.0, "RAM utilisation upper limit (%)"),
      BT::InputPort<double>("min_disk_free_mb", 500.0, "Minimum free disk space (MB)")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;

  // Previous CPU jiffies snapshot for delta computation
  uint64_t prev_cpu_total_{0};
  uint64_t prev_cpu_idle_{0};
  bool     first_cpu_sample_{true};

  /// Reads /proc/stat and returns CPU utilisation percent (0–100).
  /// Returns -1.0 on read failure.
  double readCpuPercent();

  /// Reads /proc/meminfo and returns RAM utilisation percent (0–100).
  /// Returns -1.0 on read failure.
  double readMemoryPercent();

  /// Returns available disk space on "/" in megabytes.
  /// Returns -1.0 on query failure.
  double readDiskFreeMb();
};

}  // namespace robot_custom_behaviour_tree_nodes

#endif  // ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__SYSTEM_RESOURCE_CHECK_ACTION_HPP_
