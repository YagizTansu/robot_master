#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

// ROS 2 messages
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// LiDAR projection
#include <laser_geometry/laser_geometry.hpp>

// PCL (point cloud type only — no registration headers here)
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Core utilities
#include "factor_graph_optimization/core/geometry_2d.hpp"

// Configuration struct
#include "factor_graph_optimization/config/scan_matcher_config.hpp"

// Lidar module
#include "factor_graph_optimization/lidar/map_builder.hpp"
#include "factor_graph_optimization/lidar/scan_matcher_interface.hpp"

namespace factor_graph_optimization
{

class ScanMatcherNode : public rclcpp::Node
{
public:
  explicit ScanMatcherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── Callbacks ─────────────────────────────────────────────────────────────
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void fgoPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /** Build an Eigen initial guess from the current FGO pose. */
  Eigen::Matrix4f buildInitialGuess() const;

  // enforce2D() is a free function in factor_graph_optimization::core
  // (see core/geometry_2d.hpp).

  // ── Publishers / Subscribers ──────────────────────────────────────────────
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr   sub_scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       sub_fgo_pose_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_scan_pose_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr                         pub_fitness_score_;

  // ── TF2 (needed by LaserProjection) ──────────────────────────────────────
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── LiDAR projection ─────────────────────────────────────────────────────
  laser_geometry::LaserProjection projector_;

  // ── Lidar module objects ──────────────────────────────────────────────────
  std::unique_ptr<MapBuilder>    map_builder_;   ///< OccupancyGrid → PCL cloud
  std::unique_ptr<IScanMatcher>  matcher_;       ///< NDT or ICP, selected from cfg_

  // ── Map state ─────────────────────────────────────────────────────────────
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  bool map_received_{false};
  mutable std::mutex map_mutex_;  ///< guards map_cloud_ and map_received_

  // ── Current FGO pose (for initial guess) ─────────────────────────────────
  // THREADING: written by fgoPoseCallback, read by scanCallback + buildInitialGuess.
  // Protected by fgo_pose_mutex_ so a future switch to MultiThreadedExecutor is safe.
  geometry_msgs::msg::Pose current_fgo_pose_;
  bool has_fgo_pose_{false};
  mutable std::mutex fgo_pose_mutex_;  ///< guards current_fgo_pose_ and has_fgo_pose_

  // ── Async scan matching ───────────────────────────────────────────────────
  // Set to true while a worker thread is running match(); prevents the executor
  // thread from queuing a second match before the first one finishes.
  std::atomic<bool> matching_in_progress_{false};

  // ── Parameters ───────────────────────────────────────────────────────────
  ScanMatcherConfig cfg_;
};

}  // namespace factor_graph_optimization
