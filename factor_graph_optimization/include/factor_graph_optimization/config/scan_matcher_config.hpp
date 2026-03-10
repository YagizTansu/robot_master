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
  double      map_voxel_leaf_size{0.1};     ///< VoxelGrid leaf size for map downsampling (m)

  // ── LiDAR noise written into the published covariance ────────────────────
  double noise_lidar_x{0.1};
  double noise_lidar_y{0.1};
  double noise_lidar_z{999.0};
  double noise_lidar_roll{999.0};
  double noise_lidar_pitch{999.0};
  double noise_lidar_yaw{0.1};
  double icp_fitness_score_threshold{0.5};  ///< discard scan if score > this
  double fitness_noise_scale{5.0};          ///< sigma *= (1 + scale * fitness)

  // ── Factory ──────────────────────────────────────────────────────────────
  /// Declare and load all parameters from a ROS 2 node.
  static ScanMatcherConfig fromNode(rclcpp::Node & node);
};

}  // namespace factor_graph_optimization
