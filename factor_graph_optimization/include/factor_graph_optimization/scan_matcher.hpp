#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

// ROS 2 messages
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// LiDAR projection
#include <laser_geometry/laser_geometry.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

// Core utilities
#include "factor_graph_optimization/core/geometry_2d.hpp"

// Configuration struct
#include "factor_graph_optimization/config/scan_matcher_config.hpp"

namespace factor_graph_optimization
{

class ScanMatcherNode : public rclcpp::Node
{
public:
  explicit ScanMatcherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── Parameter helpers ─────────────────────────────────────────────────────
  // Parameters are loaded via ScanMatcherConfig::fromNode() — see config/scan_matcher_config.hpp.

  // ── Callbacks ─────────────────────────────────────────────────────────────
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void fgoPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // ── Helpers ───────────────────────────────────────────────────────────────
  /** Convert OccupancyGrid occupied cells → PCL point cloud at z=map_z_height. */
  pcl::PointCloud<pcl::PointXYZ>::Ptr occupancyGridToCloud(
    const nav_msgs::msg::OccupancyGrid & grid);

  /** Run NDT matching.  Returns fitness score; result written to result_pose. */
  double runNdt(const pcl::PointCloud<pcl::PointXYZ>::Ptr & source,
                const Eigen::Matrix4f & initial_guess,
                Eigen::Matrix4f & result_transform);

  /** Run ICP matching (fallback). */
  double runIcp(const pcl::PointCloud<pcl::PointXYZ>::Ptr & source,
                const Eigen::Matrix4f & initial_guess,
                Eigen::Matrix4f & result_transform);

  /** Build an Eigen initial guess from the current FGO pose. */
  Eigen::Matrix4f buildInitialGuess() const;

  // enforce2D() is now a free function in factor_graph_optimization::core
  // (see core/geometry_2d.hpp).

  // ── Publishers / Subscribers ──────────────────────────────────────────────
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr   sub_scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       sub_fgo_pose_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_scan_pose_;

  // ── TF2 (needed by LaserProjection) ──────────────────────────────────────
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── LiDAR projection ─────────────────────────────────────────────────────
  laser_geometry::LaserProjection projector_;

  // ── Map ───────────────────────────────────────────────────────────────────
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  bool map_received_{false};

  // ── Current FGO pose (for initial guess) ─────────────────────────────────
  geometry_msgs::msg::Pose current_fgo_pose_;
  bool has_fgo_pose_{false};

  // ── Parameters ───────────────────────────────────────────────────────────
  ScanMatcherConfig cfg_;
};

}  // namespace factor_graph_optimization
