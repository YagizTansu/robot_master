#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>

namespace factor_graph_optimization
{

/// All tunable parameters for FgoNode, collected into one POD-like struct.
/// Never read from YAML directly inside algorithms — always access via this struct.
///
/// Default values match the historic declare_parameter() fallbacks so that
/// the system still works without a YAML file (all values are overridden by
/// the YAML in normal operation).
struct FgoConfig
{
  // ── Sensor toggles ───────────────────────────────────────────────────────
  bool enable_odom{true};
  bool enable_imu{true};
  bool enable_lidar{true};

  // ── Topics ───────────────────────────────────────────────────────────────
  std::string odom_topic{"/odom"};
  std::string imu_topic{"/imu/data"};
  std::string scan_match_pose_topic{"/scan_match_pose"};
  std::string initial_pose_topic{"/initialpose"};

  // ── Frames ───────────────────────────────────────────────────────────────
  std::string map_frame{"map"};
  std::string odom_frame{"odom"};
  std::string base_frame{"base_footprint"};
  std::string imu_frame{"imu_link"};

  // ── TF publishing ────────────────────────────────────────────────────────
  bool publish_map_to_odom{true};
  bool publish_odom_to_base{true};

  // ── Initial pose ─────────────────────────────────────────────────────────
  double init_x{0.0};
  double init_y{0.0};
  double init_z{0.0};
  double init_roll{0.0};
  double init_pitch{0.0};
  double init_yaw{0.0};

  // ── Odometry noise (σ, 6-DOF diagonal) ──────────────────────────────────
  double noise_odom_x{0.05};
  double noise_odom_y{0.05};
  double noise_odom_z{999.0};
  double noise_odom_roll{999.0};
  double noise_odom_pitch{999.0};
  double noise_odom_yaw{0.05};

  // ── IMU preintegration noise ─────────────────────────────────────────────
  double noise_imu_accel_sigma{0.01};        ///< accelerometer white noise  (m/s²/√Hz)
  double noise_imu_gyro_sigma{0.001};        ///< gyroscope white noise      (rad/s/√Hz)
  double noise_imu_accel_bias_sigma{0.0001}; ///< accelerometer bias RW      (m/s³/√Hz)
    double noise_imu_gyro_bias_sigma{1.0e-4};  ///< gyroscope bias RW          (rad/s²/√Hz) — typical MEMS: 1e-4 to 1e-3
  double noise_imu_integration_sigma{0.0001};///< numerical integration noise
  double imu_gravity{9.81};                  ///< gravity magnitude          (m/s²)
  double imu_bias_acc_omega_int{1.0e-5};     ///< initial bias-integral uncertainty (numerical stability)

  // ── LiDAR gating ────────────────────────────────────────────────────────
  double lidar_rotation_gate_rad{0.015};  ///< skip scan if keyframe |dyaw| > this (rad)
  double max_scan_age_sec{1.0};           ///< discard scans older than this (sec)

  // ── iSAM2 ───────────────────────────────────────────────
  double      isam2_relinearize_threshold{0.1};
  int         isam2_relinearize_skip{1};
  std::string isam2_factorization{"CHOLESKY"};  ///< "CHOLESKY" or "QR" (QR more stable when info matrix is ill-conditioned)

  // ── Node timing ─────────────────────────────────────────────
  double optimization_rate_hz{10.0};

  // ── Keyframe selection thresholds ────────────────────────────────────────
  double keyframe_translation_threshold{0.02};
  double keyframe_rotation_threshold{0.02};
  double keyframe_max_time_sec{2.0};  ///< force a keyframe after this interval even when stationary (prevents pending scans being dropped)
  // ── Prior noise for graph initialisation ─────────────────────────────────
  // These were previously hardcoded `constexpr` inside initGraph().
  double prior_pose_pos_sigma{0.001};    ///< 1 mm — tight prior on X(0) position
  double prior_pose_rot_sigma{0.001};    ///< ~0.057 deg — tight prior on X(0) rotation
  double prior_vel_sigma{1.0};           ///< m/s  — generous; robot starts at rest
  double prior_bias_accel_sigma{0.3};    ///< m/s² — moderate uncertainty on accel bias
  double prior_bias_gyro_sigma{0.05};    ///< rad/s — moderate uncertainty on gyro bias

  // ── Fallback V/B pinning (no IMU coverage for an interval) ───────────────
  // These were previously hardcoded inside the optimization loop.
  double fallback_vel_sigma{0.5};          ///< m/s
  double fallback_bias_accel_sigma{0.01};  ///< m/s²
  double fallback_bias_gyro_sigma{0.001};  ///< rad/s

  // ── Scan buffer ───────────────────────────────────────────────
  int max_pending_scans{10};   ///< max scan-match poses buffered before oldest is dropped
  int max_pending_imu{2000};   ///< IMU buffer cap (100 Hz × 20 s); prevents unbounded growth if optimizer stalls
  int max_pending_odom{500};   ///< odom buffer cap; generous but bounded
  int graph_max_size{2000};    ///< sliding-window limit: marginalize oldest keys when graph exceeds this size (0 = unlimited)
  int max_path_length{10000};  ///< cap on /fgo/path history (poses); oldest poses are dropped when exceeded (0 = unlimited)
  int max_pending_gps{100};    ///< GPS buffer cap; 100 samples ≈ 10 s at 10 Hz — prevents unbounded growth if optimizer stalls

  // ── GPS ──────────────────────────────────────────────────────────────────────
  // Parameters live under node.max_pending_gps (buffer), sensors.enable_gps (toggle),
  // topics.gps_topic, noise.gps.*, and gps.* groups in fgo_params.yaml.
  // They must NOT go under noise.lidar — that group belongs to scan_matcher_node.
  bool        enable_gps{false};              ///< feature flag — false → no subscription, zero overhead
  std::string gps_topic{"/gps/fix"};          ///< NavSatFix topic (standard: nmea_navsat_driver / ublox_gps)

  // noise.gps — parallel group to noise.odometry and noise.imu
  double noise_gps_sigma_x{3.0};  ///< 1σ GPS position noise in X (m). Default = consumer GPS CEP/0.8326 ≈ 3 m. Override to 0.05 for RTK.
  double noise_gps_sigma_y{3.0};  ///< 1σ GPS position noise in Y (m). Separate field allows anisotropic error modelling.

  // gps.* — quality gates and sensor geometry
  double gps_hdop_reject_threshold{2.0};  ///< reject fixes with HDOP above this (0 = disable); <1 excellent, 1-2 good, >5 poor
  double gps_outlier_reject_dist_m{10.0};  ///< reject fix if farther than this from last accepted (m); 0 = disable
  int    gps_utm_zone{0};                  ///< UTM zone 1-60; 0 = auto-detect from first fix
  std::string gps_utm_hemisphere{"N"};     ///< "N" or "S"; used only when gps_utm_zone != 0
  double gps_offset_x{0.0};               ///< GPS antenna offset from base_link, forward axis (m)
  double gps_offset_y{0.0};               ///< GPS antenna offset from base_link, left axis (m)
  int    gps_min_fix_type{1};             ///< min accepted quality: 0=any, 1=fix, 2=DGPS/SBAS
  int    gps_outlier_strike_limit{10};    ///< reset outlier baseline after this many consecutive rejects
  // ── Factory ──────────────────────────────────────────────────────────────
  /// Declare and load all parameters from a ROS 2 node.
  /// Replaces the former declareParameters() + loadParameters() pair.
  static FgoConfig fromNode(rclcpp::Node & node);
};

}  // namespace factor_graph_optimization
