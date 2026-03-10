#include "factor_graph_optimization/fgo_node.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// GTSAM
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/ISAM2Params.h>

namespace factor_graph_optimization
{

// ═════════════════════════════════════════════════════════════════════════════
// Construction
// ═════════════════════════════════════════════════════════════════════════════

FgoNode::FgoNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("fgo_node", options)
{
  cfg_ = FgoConfig::fromNode(*this);
  initIsam2();

  // ── TF broadcaster + listener ────────────────────────────────────────────
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_      = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // One-time lookup of static IMU→base rotation; falls back gracefully if unavailable.
  if (cfg_.imu_frame != cfg_.base_frame) {
    try {
      auto tf_imu = tf_buffer_->lookupTransform(
        cfg_.base_frame, cfg_.imu_frame, tf2::TimePointZero,
        tf2::durationFromSec(2.0));
      const auto & q = tf_imu.transform.rotation;
      R_imu_to_base_ = gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z);
      imu_tf_ready_  = true;
      RCLCPP_INFO(get_logger(),
        "[FgoNode] IMU frame '%s' -> '%s' rotation cached.",
        cfg_.imu_frame.c_str(), cfg_.base_frame.c_str());
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(),
        "[FgoNode] IMU->base TF not available (%s). "
        "Assuming IMU is already expressed in base frame.", ex.what());
    }
  }

  // ── Initialize pose tracking from initial_pose parameters ────────────────
  auto make_init_pose = [this]() -> geometry_msgs::msg::Pose {
    geometry_msgs::msg::Pose p;
    p.position.x = cfg_.init_x;
    p.position.y = cfg_.init_y;
    p.position.z = cfg_.init_z;
    tf2::Quaternion q;
    q.setRPY(cfg_.init_roll, cfg_.init_pitch, cfg_.init_yaw);
    q.normalize();
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
    return p;
  };
  keyframe_sel_ = std::make_unique<KeyframeSelector>(
    cfg_.keyframe_translation_threshold,
    cfg_.keyframe_rotation_threshold,
    make_init_pose());
  last_consumed_odom_pose_  = make_init_pose();
  last_consumed_odom_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // ── Build IMU preintegration params ──────────────────────────────────────
  if (cfg_.enable_imu) {
    initImuPreintegration();
  }

  // ── Initialise graph: add X(0)/V(0)/B(0) prior ────────────────────────
  initGraph();

  // ── Path message header ───────────────────────────────────────────────────
  path_msg_.header.frame_id = cfg_.map_frame;

  // ── Subscribers ──────────────────────────────────────────────────────────
  if (cfg_.enable_odom) {
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      cfg_.odom_topic, rclcpp::QoS(10),
      std::bind(&FgoNode::odomCallback, this, std::placeholders::_1));
  }
  if (cfg_.enable_imu) {
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      cfg_.imu_topic, rclcpp::QoS(100),
      std::bind(&FgoNode::imuCallback, this, std::placeholders::_1));
  }
  if (cfg_.enable_lidar) {
    sub_scan_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      cfg_.scan_match_pose_topic, rclcpp::QoS(10),
      std::bind(&FgoNode::scanPoseCallback, this, std::placeholders::_1));
  }
  sub_init_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    cfg_.initial_pose_topic, rclcpp::QoS(5),
    std::bind(&FgoNode::initialPoseCallback, this, std::placeholders::_1));

  // ── Publishers ────────────────────────────────────────────────────────────
  pub_odometry_ = create_publisher<nav_msgs::msg::Odometry>("/fgo/odometry", rclcpp::QoS(10));
  pub_path_     = create_publisher<nav_msgs::msg::Path>("/fgo/path", rclcpp::QoS(10));

  // ── Optimization timer ────────────────────────────────────────────────────
  const auto period_ms = std::chrono::milliseconds(
    static_cast<int>(1000.0 / cfg_.optimization_rate_hz));
  optimization_timer_ = create_wall_timer(
    period_ms, std::bind(&FgoNode::optimizationStep, this));

  RCLCPP_INFO(get_logger(), "[FgoNode] Started. map=%s odom=%s base=%s",
    cfg_.map_frame.c_str(), cfg_.odom_frame.c_str(), cfg_.base_frame.c_str());
}

// ═════════════════════════════════════════════════════════════════════════════
// Parameter loading
// ═════════════════════════════════════════════════════════════════════════════

// Parameter loading — moved to FgoConfig::fromNode() in src/config/fgo_config.cpp

// ═════════════════════════════════════════════════════════════════════════════
// iSAM2 initialisation
// ═════════════════════════════════════════════════════════════════════════════

void FgoNode::initIsam2()
{
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = cfg_.isam2_relinearize_threshold;
  params.relinearizeSkip      = cfg_.isam2_relinearize_skip;
  params.evaluateNonlinearError = false;
  isam2_ = std::make_unique<gtsam::ISAM2>(params);
}

// ════════════════════════════════════════════════════════════════════════════════
// IMU preintegration setup
// ════════════════════════════════════════════════════════════════════════════════

void FgoNode::initImuPreintegration()
{
  // MakeSharedU: ENU (Z-up) world frame convention.
  // Gravity acts along -Z in world frame → magnitude passed as positive scalar.
  auto p = gtsam::PreintegrationCombinedParams::MakeSharedU(cfg_.imu_gravity);

  // White-noise spectral densities (continuous time)
  p->accelerometerCovariance =
    gtsam::I_3x3 * (cfg_.noise_imu_accel_sigma * cfg_.noise_imu_accel_sigma);
  p->gyroscopeCovariance =
    gtsam::I_3x3 * (cfg_.noise_imu_gyro_sigma * cfg_.noise_imu_gyro_sigma);

  // Numerical integration uncertainty
  p->integrationCovariance =
    gtsam::I_3x3 * (cfg_.noise_imu_integration_sigma * cfg_.noise_imu_integration_sigma);

  // Bias random-walk spectral densities
  p->biasAccCovariance =
    gtsam::I_3x3 * (cfg_.noise_imu_accel_bias_sigma * cfg_.noise_imu_accel_bias_sigma);
  p->biasOmegaCovariance =
    gtsam::I_3x3 * (cfg_.noise_imu_gyro_bias_sigma * cfg_.noise_imu_gyro_bias_sigma);

  // Small uncertainty on initial bias integral (numerical stability)
  p->biasAccOmegaInt = gtsam::I_6x6 * cfg_.imu_bias_acc_omega_int;

  imu_preint_params_ = p;

  RCLCPP_INFO(get_logger(),
    "[FgoNode] IMU preintegration params set. "
    "accel_sigma=%.4f gyro_sigma=%.4f gravity=%.2f",
    cfg_.noise_imu_accel_sigma, cfg_.noise_imu_gyro_sigma, cfg_.imu_gravity);
}

void FgoNode::initGraph()
{
  key_ = 0;
  new_factors_.resize(0);
  new_values_.clear();

  // ── X(0): tight prior on initial pose ───────────────────────────────────────────
  gtsam::Pose3 init_pose(
    gtsam::Rot3::RzRyRx(cfg_.init_roll, cfg_.init_pitch, cfg_.init_yaw),
    gtsam::Point3(cfg_.init_x, cfg_.init_y, cfg_.init_z));

  auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector6() << cfg_.prior_pose_rot_sigma, cfg_.prior_pose_rot_sigma, cfg_.prior_pose_rot_sigma,
                         cfg_.prior_pose_pos_sigma, cfg_.prior_pose_pos_sigma, cfg_.prior_pose_pos_sigma).finished());

  new_factors_.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), init_pose, prior_noise));
  new_values_.insert(X(0), init_pose);

  // ── V(0) and B(0): velocity and bias priors (only when IMU is enabled) ────
  if (cfg_.enable_imu && imu_preint_params_) {
    optimized_velocity_ = gtsam::Vector3::Zero();
    optimized_bias_     = gtsam::imuBias::ConstantBias();

    // Robot starts from rest — prior_vel_sigma uncertainty (generous)
    auto vel_noise = gtsam::noiseModel::Isotropic::Sigma(3, cfg_.prior_vel_sigma);
    new_factors_.add(
      gtsam::PriorFactor<gtsam::Vector3>(V(0), optimized_velocity_, vel_noise));
    new_values_.insert(V(0), optimized_velocity_);

    // Bias assumed near-zero at startup; moderate uncertainty lets GTSAM estimate it
    auto bias_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector6() << cfg_.prior_bias_accel_sigma, cfg_.prior_bias_accel_sigma, cfg_.prior_bias_accel_sigma,
                           cfg_.prior_bias_gyro_sigma,  cfg_.prior_bias_gyro_sigma,  cfg_.prior_bias_gyro_sigma).finished());
    new_factors_.add(
      gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), optimized_bias_, bias_noise));
    new_values_.insert(B(0), optimized_bias_);
  }

  isam2_->update(new_factors_, new_values_);
  new_factors_.resize(0);
  new_values_.clear();

  optimized_pose_ = init_pose;

  RCLCPP_INFO(get_logger(), "[FgoNode] Graph initialised with X(0) at (%.2f, %.2f, yaw=%.2f)",
    cfg_.init_x, cfg_.init_y, cfg_.init_yaw);
}

// ═════════════════════════════════════════════════════════════════════════════
// Odometry callback
// ═════════════════════════════════════════════════════════════════════════════

void FgoNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & raw_pose = msg->pose.pose;

  // 1. Always publish odom → base_footprint using raw odometry
  if (cfg_.publish_odom_to_base) {
    publishOdomToBase(msg->header.stamp, raw_pose);
  }

  if (keyframe_sel_->checkAndUpdate(raw_pose)) {
    OdomSample s;
    s.pose      = raw_pose;
    s.timestamp = rclcpp::Time(msg->header.stamp);
    odom_buf_.push(s);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// IMU callback
// ═════════════════════════════════════════════════════════════════════════════

void FgoNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  ImuSample sample;
  sample.timestamp = rclcpp::Time(msg->header.stamp);
  sample.accel = gtsam::Vector3(
    msg->linear_acceleration.x,
    msg->linear_acceleration.y,
    msg->linear_acceleration.z);
  sample.gyro = gtsam::Vector3(
    msg->angular_velocity.x,
    msg->angular_velocity.y,
    msg->angular_velocity.z);

  // Rotate raw measurements into the robot base frame when the IMU is not
  // co-located / co-oriented with base_frame (e.g. tilted mount, X-forward vs
  // Y-forward IMU convention).  R_imu_to_base_ is the pure rotation component
  // of the static TF, cached once at startup.
  if (imu_tf_ready_) {
    sample.accel = R_imu_to_base_.rotate(sample.accel);
    sample.gyro  = R_imu_to_base_.rotate(sample.gyro);
  }

  imu_buf_.push(sample);
}

// ═════════════════════════════════════════════════════════════════════════════
// Scan pose callback (from scan_matcher_node)
// ═════════════════════════════════════════════════════════════════════════════

void FgoNode::scanPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // Gate 1: reject stale scan matches that built up while robot was stationary.
  // Without this, restarting motion floods the graph with old scans attributed
  // to wrong keyframes.
  const double age_s = (now() - rclcpp::Time(msg->header.stamp)).seconds();
  if (age_s > cfg_.max_scan_age_sec) {
    RCLCPP_DEBUG(get_logger(),
      "[FgoNode] Scan match discarded: age=%.3fs > max=%.3fs", age_s, cfg_.max_scan_age_sec);
    return;
  }

  // Gate 2: cap buffer size — prevents unbounded growth during long stationary periods
  scan_buf_.push(*msg, 10);
}

// ═════════════════════════════════════════════════════════════════════════════
// Initial pose callback — full reset
// ═════════════════════════════════════════════════════════════════════════════

void FgoNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_WARN(get_logger(), "[FgoNode] Received /initialpose — resetting graph.");

  const auto & new_pose = msg->pose.pose;

  // Extract new yaw for re-initialisation
  tf2::Quaternion tq(
    new_pose.orientation.x, new_pose.orientation.y,
    new_pose.orientation.z, new_pose.orientation.w);
  double new_roll, new_pitch, new_yaw;
  tf2::Matrix3x3(tq).getRPY(new_roll, new_pitch, new_yaw);

  // Update init parameters so initGraph() uses new pose
  cfg_.init_x     = new_pose.position.x;
  cfg_.init_y     = new_pose.position.y;
  cfg_.init_z     = new_pose.position.z;
  cfg_.init_roll  = new_roll;
  cfg_.init_pitch = new_pitch;
  cfg_.init_yaw   = new_yaw;

  // Clear sensor buffers and reset keyframe selector.
  keyframe_sel_->reset(new_pose);
  odom_buf_.clear();
  imu_buf_.clear();
  scan_buf_.clear();

  // Reset all graph state behind graph_mutex_.
  std::lock_guard<std::mutex> graph_lock(graph_mutex_);
  last_consumed_odom_pose_  = new_pose;
  last_consumed_odom_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  optimized_velocity_       = gtsam::Vector3::Zero();
  optimized_bias_           = gtsam::imuBias::ConstantBias();
  path_msg_.poses.clear();
  path_msg_.header.stamp = now();
  has_map_to_odom_cache_ = false;

  initIsam2();
  initGraph();
  updateMapToOdomCache();

  RCLCPP_INFO(get_logger(), "[FgoNode] Graph reset to (%.2f, %.2f, yaw=%.2f)",
    cfg_.init_x, cfg_.init_y, cfg_.init_yaw);
}

// ═════════════════════════════════════════════════════════════════════════════
// Optimization timer callback
// ═════════════════════════════════════════════════════════════════════════════

void FgoNode::optimizationStep()
{
  // ── Drain sensor buffers ───────────────────────────────────────────────
  std::vector<OdomSample> local_odom;
  odom_buf_.drain(local_odom);

  std::vector<ImuSample> local_imu;
  imu_buf_.drain(local_imu);

  std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> local_scan;
  scan_buf_.drain(local_scan);

  // All graph operations are serialised behind graph_mutex_.
  std::lock_guard<std::mutex> graph_lock(graph_mutex_);

  // No new keyframes — republish cached map→odom to prevent Nav2 TF timeout.
  if (local_odom.empty()) {
    if (has_map_to_odom_cache_ && cfg_.publish_map_to_odom) {
      publishMapToOdom(now());
    }
    return;
  }

  // ── Noise models ──────────────────────────────────────────────────
  auto odom_noise = makeDiagonalNoise(
    cfg_.noise_odom_x, cfg_.noise_odom_y, cfg_.noise_odom_z,
    cfg_.noise_odom_roll, cfg_.noise_odom_pitch, cfg_.noise_odom_yaw);

  // ── Odometry BetweenFactors ────────────────────────────────────────────────
  geometry_msgs::msg::Pose prev_pose = last_consumed_odom_pose_;
  const geometry_msgs::msg::Pose pre_batch_odom_pose = last_consumed_odom_pose_;
  const int batch_start_key = key_;

  // Seed running estimate from latest iSAM2 result; falls back at startup.
  gtsam::Pose3 current_estimate = optimized_pose_;
  try {
    current_estimate = isam2_->calculateEstimate<gtsam::Pose3>(X(key_));
  } catch (...) {}

  for (const auto & sample : local_odom) {
    const gtsam::Pose3 gtsam_prev    = msgToGtsam(prev_pose);
    const gtsam::Pose3 gtsam_current = msgToGtsam(sample.pose);
    const gtsam::Pose3 delta         = gtsam_prev.between(gtsam_current);

    const gtsam::Pose3 estimate = current_estimate.compose(delta);

    key_++;
    new_factors_.add(
      gtsam::BetweenFactor<gtsam::Pose3>(X(key_ - 1), X(key_), delta, odom_noise));
    new_values_.insert(X(key_), estimate);

    current_estimate = estimate;  // propagate anchor to the next keyframe
    prev_pose = sample.pose;
  }
  last_consumed_odom_pose_  = local_odom.back().pose;
  const rclcpp::Time prev_consumed_stamp = last_consumed_odom_stamp_;
  last_consumed_odom_stamp_ = local_odom.back().timestamp;

  // ── CombinedImuFactor: preintegrate IMU per keyframe interval ─────────────
  // V and B are inserted for every new key. If no IMU data covers an interval,
  // prior factors pin V and B to prevent unconstrained nodes in the graph.
  if (cfg_.enable_imu && imu_preint_params_) {
    std::sort(local_imu.begin(), local_imu.end(),
      [](const ImuSample & a, const ImuSample & b) {
        return a.timestamp < b.timestamp;
      });

    for (int i = 0; i < static_cast<int>(local_odom.size()); ++i) {
      const int from_key = batch_start_key + i;
      const int to_key   = from_key + 1;

      if (!new_values_.exists(V(to_key)))
        new_values_.insert(V(to_key), optimized_velocity_);
      if (!new_values_.exists(B(to_key)))
        new_values_.insert(B(to_key), optimized_bias_);

      const rclcpp::Time t_from =
        (i == 0) ? prev_consumed_stamp : local_odom[i - 1].timestamp;
      const rclcpp::Time t_to = local_odom[i].timestamp;

      bool imu_integrated = false;

      if (!local_imu.empty() && t_from.nanoseconds() != 0 && t_to > t_from) {
        gtsam::PreintegratedCombinedMeasurements preint(imu_preint_params_, optimized_bias_);
        int n_integrated = 0;

        for (std::size_t j = 1; j < local_imu.size(); ++j) {
          const rclcpp::Time & t_a = local_imu[j - 1].timestamp;
          const rclcpp::Time & t_b = local_imu[j].timestamp;
          if (t_b <= t_from || t_a >= t_to) continue;

          // Clamp interval to [t_from, t_to] to avoid double-counting across batches.
          const double t_a_eff = std::max(t_a.seconds(), t_from.seconds());
          const double t_b_eff = std::min(t_b.seconds(), t_to.seconds());
          const double dt = t_b_eff - t_a_eff;
          if (dt <= 0.0 || dt > 0.5) continue;

          preint.integrateMeasurement(local_imu[j - 1].accel,
                                      local_imu[j - 1].gyro, dt);
          ++n_integrated;
        }

        if (n_integrated > 0) {
          new_factors_.add(gtsam::CombinedImuFactor(
            X(from_key), V(from_key),
            X(to_key),   V(to_key),
            B(from_key), B(to_key),
            preint));
          imu_integrated = true;
        }
      }

      if (!imu_integrated) {
        // No IMU coverage for this interval — pin V and B to prevent unconstrained nodes.
        new_factors_.add(gtsam::PriorFactor<gtsam::Vector3>(
          V(to_key), optimized_velocity_,
          gtsam::noiseModel::Isotropic::Sigma(3, cfg_.fallback_vel_sigma)));
        new_factors_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
          B(to_key), optimized_bias_,
          gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector6() << cfg_.fallback_bias_accel_sigma, cfg_.fallback_bias_accel_sigma, cfg_.fallback_bias_accel_sigma,
                                 cfg_.fallback_bias_gyro_sigma,  cfg_.fallback_bias_gyro_sigma,  cfg_.fallback_bias_gyro_sigma).finished())));
      }
    }
  }

  // ── Scan match PriorFactors ───────────────────────────────────────────────
  // Each scan is gated independently against the instantaneous |dyaw| of its
  // nearest keyframe interval. NDT is unreliable during in-place rotation.
  if (!local_scan.empty()) {
    // Pre-compute |dyaw| per keyframe interval; index 0 uses pre-batch pose.
    std::vector<double> keyframe_dyaw(local_odom.size());
    {
      double yaw_prev = extractYaw(pre_batch_odom_pose.orientation);
      for (std::size_t i = 0; i < local_odom.size(); ++i) {
        double dy = std::fmod(std::fabs(extractYaw(local_odom[i].pose.orientation) - yaw_prev), 2.0 * M_PI);
        if (dy > M_PI) dy = 2.0 * M_PI - dy;
        keyframe_dyaw[i] = dy;
        yaw_prev = extractYaw(local_odom[i].pose.orientation);
      }
    }

    for (const auto & scan_msg : local_scan) {
      const rclcpp::Time scan_t(scan_msg.header.stamp);
      const gtsam::Pose3 scan_pose = msgToGtsam(scan_msg.pose.pose);

      // Find nearest keyframe by timestamp
      int best_i = static_cast<int>(local_odom.size()) - 1;
      double best_dt = std::numeric_limits<double>::max();
      for (int i = 0; i < static_cast<int>(local_odom.size()); ++i) {
        const double dt = std::fabs((local_odom[i].timestamp - scan_t).seconds());
        if (dt < best_dt) { best_dt = dt; best_i = i; }
      }

      // Per-scan gate: skip only THIS scan if its keyframe interval had rotation
      const double local_dyaw = keyframe_dyaw[best_i];
      if (local_dyaw > cfg_.lidar_rotation_gate_rad) {
        RCLCPP_DEBUG(get_logger(),
          "[FgoNode] Scan prior skipped: keyframe[%d] |dyaw|=%.4f rad > gate=%.4f rad",
          best_i, local_dyaw, cfg_.lidar_rotation_gate_rad);
        continue;
      }

      // GTSAM Pose3 sigma order: [roll, pitch, yaw, x, y, z]
      const auto & cov = scan_msg.pose.covariance;
      auto lidar_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector6() <<
          std::sqrt(cov[21]),   // roll  σ
          std::sqrt(cov[28]),   // pitch σ
          std::sqrt(cov[35]),   // yaw   σ
          std::sqrt(cov[0]),    // x     σ
          std::sqrt(cov[7]),    // y     σ
          std::sqrt(cov[14])    // z     σ
        ).finished());

      const int target_key = batch_start_key + best_i + 1;

      new_factors_.add(
        gtsam::PriorFactor<gtsam::Pose3>(X(target_key), scan_pose, lidar_noise));

      RCLCPP_DEBUG(get_logger(),
        "[FgoNode] Scan prior -> X(%d), dt=%.3fs, dyaw=%.4f rad",
        target_key, best_dt, local_dyaw);
    }
  }

  // ── iSAM2 update ─────────────────────────────────────────────────────────
  try {
    isam2_->update(new_factors_, new_values_);
    isam2_->update();  // extra pass for convergence
    new_factors_.resize(0);
    new_values_.clear();

    // Retrieve optimised pose at current key
    optimized_pose_ = isam2_->calculateEstimate<gtsam::Pose3>(X(key_));

    // Retrieve velocity and bias if IMU is active
    if (cfg_.enable_imu && imu_preint_params_) {
      try {
        optimized_velocity_ =
          isam2_->calculateEstimate<gtsam::Vector3>(V(key_));
        optimized_bias_ =
          isam2_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(key_));
      } catch (...) {
        // V/B keys may not exist if no IMU factors were added this step
      }
    }

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "[FgoNode] iSAM2 update failed: %s", e.what());
    new_factors_.resize(0);
    new_values_.clear();
    return;
  }

  // ── Publish map→odom TF ──────────────────────────────────────────────────
  if (cfg_.publish_map_to_odom) {
    updateMapToOdomCache();
    publishMapToOdom(now());
  }

  // ── Publish /fgo/odometry ─────────────────────────────────────────────────
  {
    nav_msgs::msg::Odometry odom_msg;
    // Use the sensor timestamp of the last consumed odom sample, NOT now().
    // now() is called after optimization completes (potentially 10-50ms later),
    // which breaks ApproximateTimeSynchronizer in downstream nodes (benchmark, etc.).
    odom_msg.header.stamp    = last_consumed_odom_stamp_;
    odom_msg.header.frame_id = cfg_.map_frame;
    odom_msg.child_frame_id  = cfg_.base_frame;
    odom_msg.pose.pose       = gtsamToMsg(optimized_pose_);
    pub_odometry_->publish(odom_msg);
  }

  // ── Publish /fgo/path ─────────────────────────────────────────────────────
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp    = now();
    ps.header.frame_id = cfg_.map_frame;
    ps.pose            = gtsamToMsg(optimized_pose_);
    path_msg_.poses.push_back(ps);
    path_msg_.header.stamp = now();
    pub_path_->publish(path_msg_);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// TF helpers
// ═════════════════════════════════════════════════════════════════════════════

void FgoNode::publishOdomToBase(const rclcpp::Time & stamp,
                                const geometry_msgs::msg::Pose & raw_pose)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.stamp    = stamp;
  ts.header.frame_id = cfg_.odom_frame;
  ts.child_frame_id  = cfg_.base_frame;

  ts.transform.translation.x = raw_pose.position.x;
  ts.transform.translation.y = raw_pose.position.y;
  ts.transform.translation.z = raw_pose.position.z;
  ts.transform.rotation      = raw_pose.orientation;

  tf_broadcaster_->sendTransform(ts);
}

void FgoNode::updateMapToOdomCache()
{
  // T(map→odom) = T(map→base)_opt × T(odom→base)_keyframe⁻¹
  // Keyframe anchor ensures odom dead-reckons correctly between optimisation steps.
  const gtsam::Pose3 odom_T_base = msgToGtsam(last_consumed_odom_pose_);
  const gtsam::Pose3 map_T_odom  = optimized_pose_.compose(odom_T_base.inverse());

  const gtsam::Point3 & t = map_T_odom.translation();
  const gtsam::Quaternion q = map_T_odom.rotation().toQuaternion();

  cached_map_to_odom_tf_.header.frame_id = cfg_.map_frame;
  cached_map_to_odom_tf_.child_frame_id  = cfg_.odom_frame;
  cached_map_to_odom_tf_.transform.translation.x = t.x();
  cached_map_to_odom_tf_.transform.translation.y = t.y();
  cached_map_to_odom_tf_.transform.translation.z = t.z();
  cached_map_to_odom_tf_.transform.rotation.x    = q.x();
  cached_map_to_odom_tf_.transform.rotation.y    = q.y();
  cached_map_to_odom_tf_.transform.rotation.z    = q.z();
  cached_map_to_odom_tf_.transform.rotation.w    = q.w();
  has_map_to_odom_cache_ = true;
}

void FgoNode::publishMapToOdom(const rclcpp::Time & stamp)
{
  if (!has_map_to_odom_cache_) return;
  cached_map_to_odom_tf_.header.stamp = stamp;
  tf_broadcaster_->sendTransform(cached_map_to_odom_tf_);
}

}  // namespace factor_graph_optimization
