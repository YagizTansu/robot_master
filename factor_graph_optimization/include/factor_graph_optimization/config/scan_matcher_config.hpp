#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>

namespace factor_graph_optimization
{

/// All tunable parameters for ScanMatcherNode, collected into one POD-like struct.
struct ScanMatcherConfig
{
  // ── Topics ───────────────────────────────────────────────────────────────
  std::string lidar_topic{"/scan"};
  std::string scan_match_pose_topic{"/scan_match_pose"};
  std::string map_topic{"/map"};

  // ── Frames ───────────────────────────────────────────────────────────────
  std::string lidar_frame{"base_footprint"};

  // ── Scan matcher algorithm ───────────────────────────────────────────────
  std::string scan_matcher_type{"NDT"};     ///< "NDT" or "ICP"
  int         max_iterations{50};
  double      max_correspondence_dist{1.0};
  double      transformation_epsilon{1.0e-6};
  double      map_z_height{0.0};
  double      ndt_resolution{1.0};          ///< NDT internal voxel resolution (m)
  double      ndt_step_size{0.1};           ///< Newton step size for NDT gradient descent (m)
  double map_voxel_leaf_size{0.1};     ///< VoxelGrid leaf size for map cloud downsampling (m)
  double scan_voxel_leaf_size{0.1};    ///< VoxelGrid leaf size for source scan downsampling (m; 0 = skip)
  int    icp_ransac_iterations{5};     ///< RANSAC iterations for ICP outlier rejection (0 = off)
  double icp_ransac_threshold{0.05};   ///< RANSAC inlier distance threshold (m)
  std::string map_frame{"map"};        ///< frame_id written into the published scan-pose message

  // ── LiDAR noise written into the published covariance ────────────────────
  double noise_lidar_x{0.1};
  double noise_lidar_y{0.1};
  double noise_lidar_z{999.0};
  double noise_lidar_roll{999.0};
  double noise_lidar_pitch{999.0};
  double noise_lidar_yaw{0.1};
  double fitness_score_threshold{0.5};       ///< discard scan if score > this (applies to both NDT and ICP)
  double fitness_noise_scale{5.0};          ///< sigma *= (1 + scale * fitness)

  // ── Factory ──────────────────────────────────────────────────────────────
  /// Declare and load all parameters from a ROS 2 node.
  static ScanMatcherConfig fromNode(rclcpp::Node & node);
};

}  // namespace factor_graph_optimization
