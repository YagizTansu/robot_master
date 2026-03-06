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

namespace factor_graph_optimization
{

class ScanMatcherNode : public rclcpp::Node
{
public:
  explicit ScanMatcherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── Parameter helpers ─────────────────────────────────────────────────────
  void declareParameters();
  void loadParameters();

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

  /** Enforce 2D: zero out z, roll, pitch in a 4×4 transform. */
  static Eigen::Matrix4f enforce2D(const Eigen::Matrix4f & T);

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

  // ── Parameters ────────────────────────────────────────────────────────────
  // Topics
  std::string map_topic_;
  std::string lidar_topic_;
  std::string scan_match_pose_topic_;

  // Frames
  std::string lidar_frame_;

  // Scan matcher
  std::string scan_matcher_type_;           ///< "NDT" or "ICP"
  int         max_iterations_;
  double      max_correspondence_dist_;
  double      transformation_epsilon_;
  double      map_z_height_;
  double      ndt_resolution_;              ///< NDT internal voxel resolution (m)
  double      map_voxel_leaf_size_;         ///< VoxelGrid leaf size for map cloud (m)

  // LiDAR noise (written into published covariance)
  double noise_lidar_x_;
  double noise_lidar_y_;
  double noise_lidar_z_;
  double noise_lidar_roll_;
  double noise_lidar_pitch_;
  double noise_lidar_yaw_;
  double icp_fitness_score_threshold_;  ///< discard scan if fitness > this
  double fitness_noise_scale_;          ///< sigma *= (1 + scale * fitness_score)
};

}  // namespace factor_graph_optimization
