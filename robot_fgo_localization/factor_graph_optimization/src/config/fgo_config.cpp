#include "factor_graph_optimization/config/fgo_config.hpp"

#include <stdexcept>
#include <string>

namespace factor_graph_optimization
{

FgoConfig FgoConfig::fromNode(rclcpp::Node & node)
{
  FgoConfig cfg;

  // ── Declare (with struct defaults as fallback values) ─────────────────────
  node.declare_parameter("sensors.enable_odom",  cfg.enable_odom);
  node.declare_parameter("sensors.enable_imu",   cfg.enable_imu);
  node.declare_parameter("sensors.enable_lidar", cfg.enable_lidar);

  node.declare_parameter("topics.odom_topic",            cfg.odom_topic);
  node.declare_parameter("topics.imu_topic",             cfg.imu_topic);
  node.declare_parameter("topics.scan_match_pose_topic", cfg.scan_match_pose_topic);
  node.declare_parameter("topics.initial_pose_topic",    cfg.initial_pose_topic);

  node.declare_parameter("frames.map_frame",  cfg.map_frame);
  node.declare_parameter("frames.odom_frame", cfg.odom_frame);
  node.declare_parameter("frames.base_frame", cfg.base_frame);
  node.declare_parameter("frames.imu_frame",  cfg.imu_frame);

  node.declare_parameter("tf.publish_map_to_odom",  cfg.publish_map_to_odom);
  node.declare_parameter("tf.publish_odom_to_base", cfg.publish_odom_to_base);

  node.declare_parameter("initial_pose.x",     cfg.init_x);
  node.declare_parameter("initial_pose.y",     cfg.init_y);
  node.declare_parameter("initial_pose.z",     cfg.init_z);
  node.declare_parameter("initial_pose.roll",  cfg.init_roll);
  node.declare_parameter("initial_pose.pitch", cfg.init_pitch);
  node.declare_parameter("initial_pose.yaw",   cfg.init_yaw);

  node.declare_parameter("noise.odometry.x",     cfg.noise_odom_x);
  node.declare_parameter("noise.odometry.y",     cfg.noise_odom_y);
  node.declare_parameter("noise.odometry.z",     cfg.noise_odom_z);
  node.declare_parameter("noise.odometry.roll",  cfg.noise_odom_roll);
  node.declare_parameter("noise.odometry.pitch", cfg.noise_odom_pitch);
  node.declare_parameter("noise.odometry.yaw",   cfg.noise_odom_yaw);

  node.declare_parameter("noise.imu.accel_sigma",        cfg.noise_imu_accel_sigma);
  node.declare_parameter("noise.imu.gyro_sigma",         cfg.noise_imu_gyro_sigma);
  node.declare_parameter("noise.imu.accel_bias_sigma",   cfg.noise_imu_accel_bias_sigma);
  node.declare_parameter("noise.imu.gyro_bias_sigma",    cfg.noise_imu_gyro_bias_sigma);
  node.declare_parameter("noise.imu.integration_sigma",  cfg.noise_imu_integration_sigma);
  node.declare_parameter("noise.imu.gravity",            cfg.imu_gravity);
  node.declare_parameter("noise.imu.bias_acc_omega_int", cfg.imu_bias_acc_omega_int);

  node.declare_parameter("lidar.gating.rotation_gate_rad", cfg.lidar_rotation_gate_rad);
  node.declare_parameter("lidar.gating.max_scan_age_sec",  cfg.max_scan_age_sec);

  node.declare_parameter("isam2.relinearize_threshold", cfg.isam2_relinearize_threshold);
  node.declare_parameter("isam2.relinearize_skip",      cfg.isam2_relinearize_skip);
  node.declare_parameter("isam2.factorization",         cfg.isam2_factorization);
  node.declare_parameter("node.optimization_rate_hz",   cfg.optimization_rate_hz);
  node.declare_parameter("node.max_pending_scans",      cfg.max_pending_scans);
  node.declare_parameter("node.max_pending_imu",        cfg.max_pending_imu);
  node.declare_parameter("node.max_pending_odom",       cfg.max_pending_odom);
  node.declare_parameter("node.graph_max_size",         cfg.graph_max_size);
  node.declare_parameter("node.max_path_length",        cfg.max_path_length);
  node.declare_parameter("node.max_pending_gps",        cfg.max_pending_gps);

  node.declare_parameter("keyframe.translation_threshold", cfg.keyframe_translation_threshold);
  node.declare_parameter("keyframe.rotation_threshold",    cfg.keyframe_rotation_threshold);
  node.declare_parameter("keyframe.max_time_sec",          cfg.keyframe_max_time_sec);

  node.declare_parameter("prior.pose_pos_sigma",   cfg.prior_pose_pos_sigma);
  node.declare_parameter("prior.pose_rot_sigma",   cfg.prior_pose_rot_sigma);
  node.declare_parameter("prior.vel_sigma",        cfg.prior_vel_sigma);
  node.declare_parameter("prior.bias_accel_sigma", cfg.prior_bias_accel_sigma);
  node.declare_parameter("prior.bias_gyro_sigma",  cfg.prior_bias_gyro_sigma);

  node.declare_parameter("fallback.vel_sigma",        cfg.fallback_vel_sigma);
  node.declare_parameter("fallback.bias_accel_sigma", cfg.fallback_bias_accel_sigma);
  node.declare_parameter("fallback.bias_gyro_sigma",  cfg.fallback_bias_gyro_sigma);

  node.declare_parameter("sensors.enable_gps",          cfg.enable_gps);
  node.declare_parameter("topics.gps_topic",             cfg.gps_topic);
  node.declare_parameter("noise.gps.sigma_x",            cfg.noise_gps_sigma_x);
  node.declare_parameter("noise.gps.sigma_y",            cfg.noise_gps_sigma_y);
  node.declare_parameter("gps.hdop_reject_threshold",    cfg.gps_hdop_reject_threshold);
  node.declare_parameter("gps.outlier_reject_dist_m",    cfg.gps_outlier_reject_dist_m);
  node.declare_parameter("gps.utm_zone",                 cfg.gps_utm_zone);
  node.declare_parameter("gps.utm_hemisphere",           cfg.gps_utm_hemisphere);
  node.declare_parameter("gps.offset_x",                 cfg.gps_offset_x);
  node.declare_parameter("gps.offset_y",                 cfg.gps_offset_y);
  node.declare_parameter("gps.min_fix_type",             cfg.gps_min_fix_type);
  node.declare_parameter("gps.outlier_strike_limit",     cfg.gps_outlier_strike_limit);

  // ── Load ──────────────────────────────────────────────────────────────────
  cfg.enable_odom  = node.get_parameter("sensors.enable_odom").as_bool();
  cfg.enable_imu   = node.get_parameter("sensors.enable_imu").as_bool();
  cfg.enable_lidar = node.get_parameter("sensors.enable_lidar").as_bool();

  cfg.odom_topic            = node.get_parameter("topics.odom_topic").as_string();
  cfg.imu_topic             = node.get_parameter("topics.imu_topic").as_string();
  cfg.scan_match_pose_topic = node.get_parameter("topics.scan_match_pose_topic").as_string();
  cfg.initial_pose_topic    = node.get_parameter("topics.initial_pose_topic").as_string();

  cfg.map_frame  = node.get_parameter("frames.map_frame").as_string();
  cfg.odom_frame = node.get_parameter("frames.odom_frame").as_string();
  cfg.base_frame = node.get_parameter("frames.base_frame").as_string();
  cfg.imu_frame  = node.get_parameter("frames.imu_frame").as_string();

  cfg.publish_map_to_odom  = node.get_parameter("tf.publish_map_to_odom").as_bool();
  cfg.publish_odom_to_base = node.get_parameter("tf.publish_odom_to_base").as_bool();

  cfg.init_x     = node.get_parameter("initial_pose.x").as_double();
  cfg.init_y     = node.get_parameter("initial_pose.y").as_double();
  cfg.init_z     = node.get_parameter("initial_pose.z").as_double();
  cfg.init_roll  = node.get_parameter("initial_pose.roll").as_double();
  cfg.init_pitch = node.get_parameter("initial_pose.pitch").as_double();
  cfg.init_yaw   = node.get_parameter("initial_pose.yaw").as_double();

  cfg.noise_odom_x     = node.get_parameter("noise.odometry.x").as_double();
  cfg.noise_odom_y     = node.get_parameter("noise.odometry.y").as_double();
  cfg.noise_odom_z     = node.get_parameter("noise.odometry.z").as_double();
  cfg.noise_odom_roll  = node.get_parameter("noise.odometry.roll").as_double();
  cfg.noise_odom_pitch = node.get_parameter("noise.odometry.pitch").as_double();
  cfg.noise_odom_yaw   = node.get_parameter("noise.odometry.yaw").as_double();

  cfg.noise_imu_accel_sigma       = node.get_parameter("noise.imu.accel_sigma").as_double();
  cfg.noise_imu_gyro_sigma        = node.get_parameter("noise.imu.gyro_sigma").as_double();
  cfg.noise_imu_accel_bias_sigma  = node.get_parameter("noise.imu.accel_bias_sigma").as_double();
  cfg.noise_imu_gyro_bias_sigma   = node.get_parameter("noise.imu.gyro_bias_sigma").as_double();
  cfg.noise_imu_integration_sigma = node.get_parameter("noise.imu.integration_sigma").as_double();
  cfg.imu_gravity                 = node.get_parameter("noise.imu.gravity").as_double();
  cfg.imu_bias_acc_omega_int      = node.get_parameter("noise.imu.bias_acc_omega_int").as_double();

  cfg.lidar_rotation_gate_rad = node.get_parameter("lidar.gating.rotation_gate_rad").as_double();
  cfg.max_scan_age_sec        = node.get_parameter("lidar.gating.max_scan_age_sec").as_double();

  cfg.isam2_relinearize_threshold = node.get_parameter("isam2.relinearize_threshold").as_double();
  cfg.isam2_relinearize_skip      = node.get_parameter("isam2.relinearize_skip").as_int();
  cfg.isam2_factorization         = node.get_parameter("isam2.factorization").as_string();
  cfg.optimization_rate_hz        = node.get_parameter("node.optimization_rate_hz").as_double();
  cfg.max_pending_scans           = node.get_parameter("node.max_pending_scans").as_int();
  cfg.max_pending_imu             = node.get_parameter("node.max_pending_imu").as_int();
  cfg.max_pending_odom            = node.get_parameter("node.max_pending_odom").as_int();
  cfg.graph_max_size              = node.get_parameter("node.graph_max_size").as_int();
  cfg.max_path_length             = node.get_parameter("node.max_path_length").as_int();
  cfg.max_pending_gps             = node.get_parameter("node.max_pending_gps").as_int();

  cfg.keyframe_translation_threshold = node.get_parameter("keyframe.translation_threshold").as_double();
  cfg.keyframe_rotation_threshold    = node.get_parameter("keyframe.rotation_threshold").as_double();
  cfg.keyframe_max_time_sec          = node.get_parameter("keyframe.max_time_sec").as_double();

  cfg.prior_pose_pos_sigma   = node.get_parameter("prior.pose_pos_sigma").as_double();
  cfg.prior_pose_rot_sigma   = node.get_parameter("prior.pose_rot_sigma").as_double();
  cfg.prior_vel_sigma        = node.get_parameter("prior.vel_sigma").as_double();
  cfg.prior_bias_accel_sigma = node.get_parameter("prior.bias_accel_sigma").as_double();
  cfg.prior_bias_gyro_sigma  = node.get_parameter("prior.bias_gyro_sigma").as_double();

  cfg.fallback_vel_sigma        = node.get_parameter("fallback.vel_sigma").as_double();
  cfg.fallback_bias_accel_sigma = node.get_parameter("fallback.bias_accel_sigma").as_double();
  cfg.fallback_bias_gyro_sigma  = node.get_parameter("fallback.bias_gyro_sigma").as_double();

  cfg.enable_gps                = node.get_parameter("sensors.enable_gps").as_bool();
  cfg.gps_topic                 = node.get_parameter("topics.gps_topic").as_string();
  cfg.noise_gps_sigma_x         = node.get_parameter("noise.gps.sigma_x").as_double();
  cfg.noise_gps_sigma_y         = node.get_parameter("noise.gps.sigma_y").as_double();
  cfg.gps_hdop_reject_threshold = node.get_parameter("gps.hdop_reject_threshold").as_double();
  cfg.gps_outlier_reject_dist_m = node.get_parameter("gps.outlier_reject_dist_m").as_double();
  cfg.gps_utm_zone              = node.get_parameter("gps.utm_zone").as_int();
  cfg.gps_utm_hemisphere        = node.get_parameter("gps.utm_hemisphere").as_string();
  cfg.gps_offset_x              = node.get_parameter("gps.offset_x").as_double();
  cfg.gps_offset_y              = node.get_parameter("gps.offset_y").as_double();
  cfg.gps_min_fix_type          = node.get_parameter("gps.min_fix_type").as_int();
  cfg.gps_outlier_strike_limit  = node.get_parameter("gps.outlier_strike_limit").as_int();

  // ── Validation ────────────────────────────────────────────────────────────
  // GPS sigma must be strictly positive — zero would make the iSAM2
  // information matrix singular and crash the Cholesky solver.
  if (cfg.noise_gps_sigma_x <= 0.0 || cfg.noise_gps_sigma_y <= 0.0) {
    throw std::invalid_argument(
      "[FgoConfig] noise.gps.sigma_x and noise.gps.sigma_y must be > 0. "
      "Got sigma_x=" + std::to_string(cfg.noise_gps_sigma_x) +
      " sigma_y=" + std::to_string(cfg.noise_gps_sigma_y));
  }
  if (cfg.gps_min_fix_type < 0 || cfg.gps_min_fix_type > 2) {
    throw std::invalid_argument(
      "[FgoConfig] gps.min_fix_type must be 0, 1, or 2. "
      "Got " + std::to_string(cfg.gps_min_fix_type));
  }
  if (cfg.gps_utm_hemisphere != "N" && cfg.gps_utm_hemisphere != "S") {
    throw std::invalid_argument(
      "[FgoConfig] gps.utm_hemisphere must be \"N\" or \"S\". "
      "Got \"" + cfg.gps_utm_hemisphere + "\"");
  }
  if (cfg.max_pending_gps <= 0) {
    throw std::invalid_argument(
      "[FgoConfig] node.max_pending_gps must be > 0. "
      "Got " + std::to_string(cfg.max_pending_gps));
  }

  return cfg;
}

}  // namespace factor_graph_optimization
