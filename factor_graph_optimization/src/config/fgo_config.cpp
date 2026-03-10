#include "factor_graph_optimization/config/fgo_config.hpp"

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
  node.declare_parameter("node.optimization_rate_hz",  cfg.optimization_rate_hz);

  node.declare_parameter("keyframe.translation_threshold", cfg.keyframe_translation_threshold);
  node.declare_parameter("keyframe.rotation_threshold",    cfg.keyframe_rotation_threshold);

  node.declare_parameter("prior.pose_pos_sigma",   cfg.prior_pose_pos_sigma);
  node.declare_parameter("prior.pose_rot_sigma",   cfg.prior_pose_rot_sigma);
  node.declare_parameter("prior.vel_sigma",        cfg.prior_vel_sigma);
  node.declare_parameter("prior.bias_accel_sigma", cfg.prior_bias_accel_sigma);
  node.declare_parameter("prior.bias_gyro_sigma",  cfg.prior_bias_gyro_sigma);

  node.declare_parameter("fallback.vel_sigma",        cfg.fallback_vel_sigma);
  node.declare_parameter("fallback.bias_accel_sigma", cfg.fallback_bias_accel_sigma);
  node.declare_parameter("fallback.bias_gyro_sigma",  cfg.fallback_bias_gyro_sigma);

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
  cfg.optimization_rate_hz        = node.get_parameter("node.optimization_rate_hz").as_double();

  cfg.keyframe_translation_threshold = node.get_parameter("keyframe.translation_threshold").as_double();
  cfg.keyframe_rotation_threshold    = node.get_parameter("keyframe.rotation_threshold").as_double();

  cfg.prior_pose_pos_sigma   = node.get_parameter("prior.pose_pos_sigma").as_double();
  cfg.prior_pose_rot_sigma   = node.get_parameter("prior.pose_rot_sigma").as_double();
  cfg.prior_vel_sigma        = node.get_parameter("prior.vel_sigma").as_double();
  cfg.prior_bias_accel_sigma = node.get_parameter("prior.bias_accel_sigma").as_double();
  cfg.prior_bias_gyro_sigma  = node.get_parameter("prior.bias_gyro_sigma").as_double();

  cfg.fallback_vel_sigma        = node.get_parameter("fallback.vel_sigma").as_double();
  cfg.fallback_bias_accel_sigma = node.get_parameter("fallback.bias_accel_sigma").as_double();
  cfg.fallback_bias_gyro_sigma  = node.get_parameter("fallback.bias_gyro_sigma").as_double();

  return cfg;
}

}  // namespace factor_graph_optimization
