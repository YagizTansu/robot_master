#include "factor_graph_optimization/config/scan_matcher_config.hpp"

namespace factor_graph_optimization
{

ScanMatcherConfig ScanMatcherConfig::fromNode(rclcpp::Node & node)
{
  ScanMatcherConfig cfg;

  // ── Declare ───────────────────────────────────────────────────────────────
  node.declare_parameter("topics.lidar_topic",             cfg.lidar_topic);
  node.declare_parameter("topics.scan_match_pose_topic",   cfg.scan_match_pose_topic);
  node.declare_parameter("topics.map_topic",               cfg.map_topic);

  node.declare_parameter("frames.lidar_frame", cfg.lidar_frame);

  node.declare_parameter("scan_matcher.type",                    cfg.scan_matcher_type);
  node.declare_parameter("scan_matcher.max_iterations",          cfg.max_iterations);
  node.declare_parameter("scan_matcher.max_correspondence_dist", cfg.max_correspondence_dist);
  node.declare_parameter("scan_matcher.transformation_epsilon",  cfg.transformation_epsilon);
  node.declare_parameter("scan_matcher.map_z_height",            cfg.map_z_height);
  node.declare_parameter("scan_matcher.ndt_resolution",          cfg.ndt_resolution);

  node.declare_parameter("noise.lidar.x",                          cfg.noise_lidar_x);
  node.declare_parameter("noise.lidar.y",                          cfg.noise_lidar_y);
  node.declare_parameter("noise.lidar.z",                          cfg.noise_lidar_z);
  node.declare_parameter("noise.lidar.roll",                       cfg.noise_lidar_roll);
  node.declare_parameter("noise.lidar.pitch",                      cfg.noise_lidar_pitch);
  node.declare_parameter("noise.lidar.yaw",                        cfg.noise_lidar_yaw);
  node.declare_parameter("scan_matcher.fitness_score_threshold", cfg.fitness_score_threshold);
  node.declare_parameter("scan_matcher.fitness_noise_scale",         cfg.fitness_noise_scale);
  node.declare_parameter("scan_matcher.map_voxel_leaf_size",         cfg.map_voxel_leaf_size);

  // ── Load ──────────────────────────────────────────────────────────────────
  cfg.lidar_topic            = node.get_parameter("topics.lidar_topic").as_string();
  cfg.scan_match_pose_topic  = node.get_parameter("topics.scan_match_pose_topic").as_string();
  cfg.map_topic              = node.get_parameter("topics.map_topic").as_string();

  cfg.lidar_frame = node.get_parameter("frames.lidar_frame").as_string();

  cfg.scan_matcher_type       = node.get_parameter("scan_matcher.type").as_string();
  cfg.max_iterations          = node.get_parameter("scan_matcher.max_iterations").as_int();
  cfg.max_correspondence_dist = node.get_parameter("scan_matcher.max_correspondence_dist").as_double();
  cfg.transformation_epsilon  = node.get_parameter("scan_matcher.transformation_epsilon").as_double();
  cfg.map_z_height            = node.get_parameter("scan_matcher.map_z_height").as_double();
  cfg.ndt_resolution          = node.get_parameter("scan_matcher.ndt_resolution").as_double();

  cfg.noise_lidar_x             = node.get_parameter("noise.lidar.x").as_double();
  cfg.noise_lidar_y             = node.get_parameter("noise.lidar.y").as_double();
  cfg.noise_lidar_z             = node.get_parameter("noise.lidar.z").as_double();
  cfg.noise_lidar_roll          = node.get_parameter("noise.lidar.roll").as_double();
  cfg.noise_lidar_pitch         = node.get_parameter("noise.lidar.pitch").as_double();
  cfg.noise_lidar_yaw           = node.get_parameter("noise.lidar.yaw").as_double();
  cfg.fitness_score_threshold = node.get_parameter("scan_matcher.fitness_score_threshold").as_double();
  cfg.fitness_noise_scale       = node.get_parameter("scan_matcher.fitness_noise_scale").as_double();
  cfg.map_voxel_leaf_size       = node.get_parameter("scan_matcher.map_voxel_leaf_size").as_double();

  return cfg;
}

}  // namespace factor_graph_optimization
