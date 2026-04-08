#include "robot_custom_behaviour_tree_nodes/system_resource_check_action.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <sys/statvfs.h>

namespace robot_custom_behaviour_tree_nodes
{

SystemResourceCheckAction::SystemResourceCheckAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  node_ = rclcpp::Node::make_shared("system_resource_check_node");

  RCLCPP_INFO(node_->get_logger(), "SystemResourceCheckAction initialized.");
}

double SystemResourceCheckAction::readCpuPercent()
{
  std::ifstream stat_file("/proc/stat");
  if (!stat_file.is_open()) {
    return -1.0;
  }

  std::string line;
  std::getline(stat_file, line);  // first line: "cpu  user nice system idle ..."
  stat_file.close();

  std::istringstream ss(line);
  std::string label;
  uint64_t user, nice, system, idle, iowait, irq, softirq, steal;
  ss >> label >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;

  uint64_t total = user + nice + system + idle + iowait + irq + softirq + steal;
  uint64_t current_idle = idle + iowait;

  if (first_cpu_sample_) {
    prev_cpu_total_ = total;
    prev_cpu_idle_ = current_idle;
    first_cpu_sample_ = false;
    return 0.0;  // Not enough data for first sample
  }

  uint64_t total_delta = total - prev_cpu_total_;
  uint64_t idle_delta  = current_idle - prev_cpu_idle_;

  prev_cpu_total_ = total;
  prev_cpu_idle_  = current_idle;

  if (total_delta == 0) {
    return 0.0;
  }

  return 100.0 * (1.0 - static_cast<double>(idle_delta) / static_cast<double>(total_delta));
}

double SystemResourceCheckAction::readMemoryPercent()
{
  std::ifstream mem_file("/proc/meminfo");
  if (!mem_file.is_open()) {
    return -1.0;
  }

  uint64_t mem_total_kb = 0;
  uint64_t mem_available_kb = 0;
  std::string line;

  while (std::getline(mem_file, line)) {
    std::istringstream ss(line);
    std::string key;
    uint64_t value;
    ss >> key >> value;

    if (key == "MemTotal:") {
      mem_total_kb = value;
    } else if (key == "MemAvailable:") {
      mem_available_kb = value;
    }

    if (mem_total_kb > 0 && mem_available_kb > 0) {
      break;
    }
  }
  mem_file.close();

  if (mem_total_kb == 0) {
    return -1.0;
  }

  return 100.0 * (1.0 - static_cast<double>(mem_available_kb) /
                         static_cast<double>(mem_total_kb));
}

double SystemResourceCheckAction::readDiskFreeMb()
{
  struct statvfs stat {};
  if (statvfs("/", &stat) != 0) {
    return -1.0;
  }

  // f_bavail: blocks available to unprivileged users; f_frsize: fragment size in bytes
  uint64_t free_bytes = static_cast<uint64_t>(stat.f_bavail) *
                        static_cast<uint64_t>(stat.f_frsize);
  return static_cast<double>(free_bytes) / (1024.0 * 1024.0);
}

BT::NodeStatus SystemResourceCheckAction::tick()
{
  double max_cpu_percent    = 90.0;
  double max_memory_percent = 85.0;
  double min_disk_free_mb   = 500.0;

  getInput("max_cpu_percent",    max_cpu_percent);
  getInput("max_memory_percent", max_memory_percent);
  getInput("min_disk_free_mb",   min_disk_free_mb);

  bool any_exceeded = false;

  // --- CPU ---
  double cpu_pct = readCpuPercent();
  if (cpu_pct >= 0.0 && cpu_pct > max_cpu_percent) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "SystemResourceCheck: CPU usage HIGH — %.1f%% (limit: %.1f%%).",
      cpu_pct, max_cpu_percent);
    any_exceeded = true;
  } else {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
      "SystemResourceCheck: CPU %.1f%%.", cpu_pct);
  }

  // --- Memory ---
  double mem_pct = readMemoryPercent();
  if (mem_pct >= 0.0 && mem_pct > max_memory_percent) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "SystemResourceCheck: RAM usage HIGH — %.1f%% (limit: %.1f%%).",
      mem_pct, max_memory_percent);
    any_exceeded = true;
  } else {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
      "SystemResourceCheck: RAM %.1f%%.", mem_pct);
  }

  // --- Disk ---
  double disk_free_mb = readDiskFreeMb();
  if (disk_free_mb >= 0.0 && disk_free_mb < min_disk_free_mb) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "SystemResourceCheck: disk space LOW — %.0f MB free (limit: %.0f MB).",
      disk_free_mb, min_disk_free_mb);
    any_exceeded = true;
  } else {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
      "SystemResourceCheck: disk %.0f MB free.", disk_free_mb);
  }

  return any_exceeded ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

}  // namespace robot_custom_behaviour_tree_nodes

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<
    robot_custom_behaviour_tree_nodes::SystemResourceCheckAction>("SystemResourceCheck");
}
