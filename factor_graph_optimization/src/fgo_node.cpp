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
  declareParameters();
  loadParameters();
  initIsam2();

  // ── TF broadcaster + listener ────────────────────────────────────────────
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_      = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // One-time lookup of static IMU→base_frame rotation.
  // Static TFs are available immediately; 2-second timeout covers slow startup.
  // If the transform is unavailable, imu_tf_ready_ stays false and raw IMU
  // data is used (safe when imu_frame == base_frame).
  if (imu_frame_ != base_frame_) {
    try {
      auto tf_imu = tf_buffer_->lookupTransform(
        base_frame_, imu_frame_, tf2::TimePointZero,
        tf2::durationFromSec(2.0));
      const auto & q = tf_imu.transform.rotation;
      R_imu_to_base_ = gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z);
      imu_tf_ready_  = true;
      RCLCPP_INFO(get_logger(),
        "[FgoNode] IMU frame '%s' -> '%s' rotation cached.",
        imu_frame_.c_str(), base_frame_.c_str());
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(),
        "[FgoNode] IMU->base TF not available (%s). "
        "Assuming IMU is already expressed in base frame.", ex.what());
    }
  }

  // ── Initialize pose tracking from initial_pose parameters ────────────────
  auto make_init_pose = [this]() -> geometry_msgs::msg::Pose {
    geometry_msgs::msg::Pose p;
    p.position.x = init_x_;
    p.position.y = init_y_;
    p.position.z = init_z_;
    tf2::Quaternion q;
    q.setRPY(init_roll_, init_pitch_, init_yaw_);
    q.normalize();
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
    return p;
  };
  last_keyframe_odom_pose_  = make_init_pose();
  last_consumed_odom_pose_  = make_init_pose();
  last_consumed_odom_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // ── Build IMU preintegration params ──────────────────────────────────────
  if (enable_imu_) {
    initImuPreintegration();
  }

  // ── Initialise graph: add X(0)/V(0)/B(0) prior ────────────────────────
  initGraph();

  // ── Path message header ───────────────────────────────────────────────────
  path_msg_.header.frame_id = map_frame_;

  // ── Subscribers ──────────────────────────────────────────────────────────
  if (enable_odom_) {
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      std::bind(&FgoNode::odomCallback, this, std::placeholders::_1));
  }
  if (enable_imu_) {
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::QoS(100),
      std::bind(&FgoNode::imuCallback, this, std::placeholders::_1));
  }
  if (enable_lidar_) {
    sub_scan_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      scan_match_pose_topic_, rclcpp::QoS(10),
      std::bind(&FgoNode::scanPoseCallback, this, std::placeholders::_1));
  }
  sub_init_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    initial_pose_topic_, rclcpp::QoS(5),
    std::bind(&FgoNode::initialPoseCallback, this, std::placeholders::_1));

  // ── Publishers ────────────────────────────────────────────────────────────
  pub_odometry_ = create_publisher<nav_msgs::msg::Odometry>("/fgo/odometry", rclcpp::QoS(10));
  pub_path_     = create_publisher<nav_msgs::msg::Path>("/fgo/path", rclcpp::QoS(10));

  // ── Optimization timer ────────────────────────────────────────────────────
  const auto period_ms = std::chrono::milliseconds(
    static_cast<int>(1000.0 / optimization_rate_hz_));
  optimization_timer_ = create_wall_timer(
    period_ms, std::bind(&FgoNode::optimizationStep, this));

  RCLCPP_INFO(get_logger(), "[FgoNode] Started. map=%s odom=%s base=%s",
    map_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
}

// ═════════════════════════════════════════════════════════════════════════════
// Parameter loading
// ═════════════════════════════════════════════════════════════════════════════

void FgoNode::declareParameters()
{
  // Sensor toggles
  declare_parameter("sensors.enable_odom",  true);
  declare_parameter("sensors.enable_imu",   true);
  declare_parameter("sensors.enable_lidar", true);

  // Topics
  declare_parameter("topics.odom_topic",            "/odom");
  declare_parameter("topics.imu_topic",             "/imu/data");
  declare_parameter("topics.scan_match_pose_topic", "/scan_match_pose");
  declare_parameter("topics.initial_pose_topic",    "/initialpose");

  // Frames
  declare_parameter("frames.map_frame",  "map");
  declare_parameter("frames.odom_frame", "odom");
  declare_parameter("frames.base_frame", "base_footprint");
  declare_parameter("frames.imu_frame",  "imu_link");

  // TF
  declare_parameter("tf.publish_map_to_odom",  true);
  declare_parameter("tf.publish_odom_to_base", true);

  // Initial pose
  declare_parameter("initial_pose.x",     0.0);
  declare_parameter("initial_pose.y",     0.0);
  declare_parameter("initial_pose.z",     0.0);
  declare_parameter("initial_pose.roll",  0.0);
  declare_parameter("initial_pose.pitch", 0.0);
  declare_parameter("initial_pose.yaw",   0.0);

  // Odometry noise
  declare_parameter("noise.odometry.x",     0.05);
  declare_parameter("noise.odometry.y",     0.05);
  declare_parameter("noise.odometry.z",     999.0);
  declare_parameter("noise.odometry.roll",  999.0);
  declare_parameter("noise.odometry.pitch", 999.0);
  declare_parameter("noise.odometry.yaw",   0.05);

  // IMU preintegration noise
  declare_parameter("noise.imu.accel_sigma",       0.01);
  declare_parameter("noise.imu.gyro_sigma",        0.001);
  declare_parameter("noise.imu.accel_bias_sigma",  0.0001);
  declare_parameter("noise.imu.gyro_bias_sigma",   0.000001);
  declare_parameter("noise.imu.integration_sigma", 0.0001);
  declare_parameter("noise.imu.gravity",           9.81);

  // LiDAR noise + gating
  declare_parameter("noise.lidar.x",     0.1);
  declare_parameter("noise.lidar.y",     0.1);
  declare_parameter("noise.lidar.z",     999.0);
  declare_parameter("noise.lidar.roll",  999.0);
  declare_parameter("noise.lidar.pitch", 999.0);
  declare_parameter("noise.lidar.yaw",   0.1);
  declare_parameter("noise.lidar.icp_fitness_score_threshold", 0.5);
  // Skip scan prior if total batch |dyaw| exceeds this value (rad).
  // NDT becomes unreliable during fast in-place rotation.
  declare_parameter("noise.lidar.rotation_gate_rad",   0.02);
  // Sigma scaling: sigma *= (1 + fitness_noise_scale * fitness_score).
  // Poor matches (high fitness) contribute a much weaker constraint.
  declare_parameter("noise.lidar.fitness_noise_scale", 5.0);
  // Discard scan poses older than this (sec). Prevents stale scans after a pause.
  declare_parameter("noise.lidar.max_scan_age_sec", 1.0);

  // iSAM2
  declare_parameter("isam2.relinearize_threshold", 0.1);
  declare_parameter("isam2.relinearize_skip",       1);
  declare_parameter("isam2.optimization_rate_hz",  10.0);

  // Keyframe
  declare_parameter("keyframe.translation_threshold", 0.02);
  declare_parameter("keyframe.rotation_threshold",    0.02);
}

void FgoNode::loadParameters()
{
  enable_odom_  = get_parameter("sensors.enable_odom").as_bool();
  enable_imu_   = get_parameter("sensors.enable_imu").as_bool();
  enable_lidar_ = get_parameter("sensors.enable_lidar").as_bool();

  odom_topic_            = get_parameter("topics.odom_topic").as_string();
  imu_topic_             = get_parameter("topics.imu_topic").as_string();
  scan_match_pose_topic_ = get_parameter("topics.scan_match_pose_topic").as_string();
  initial_pose_topic_    = get_parameter("topics.initial_pose_topic").as_string();

  map_frame_  = get_parameter("frames.map_frame").as_string();
  odom_frame_ = get_parameter("frames.odom_frame").as_string();
  base_frame_ = get_parameter("frames.base_frame").as_string();
  imu_frame_  = get_parameter("frames.imu_frame").as_string();

  publish_map_to_odom_  = get_parameter("tf.publish_map_to_odom").as_bool();
  publish_odom_to_base_ = get_parameter("tf.publish_odom_to_base").as_bool();

  init_x_     = get_parameter("initial_pose.x").as_double();
  init_y_     = get_parameter("initial_pose.y").as_double();
  init_z_     = get_parameter("initial_pose.z").as_double();
  init_roll_  = get_parameter("initial_pose.roll").as_double();
  init_pitch_ = get_parameter("initial_pose.pitch").as_double();
  init_yaw_   = get_parameter("initial_pose.yaw").as_double();

  noise_odom_x_     = get_parameter("noise.odometry.x").as_double();
  noise_odom_y_     = get_parameter("noise.odometry.y").as_double();
  noise_odom_z_     = get_parameter("noise.odometry.z").as_double();
  noise_odom_roll_  = get_parameter("noise.odometry.roll").as_double();
  noise_odom_pitch_ = get_parameter("noise.odometry.pitch").as_double();
  noise_odom_yaw_   = get_parameter("noise.odometry.yaw").as_double();

  noise_imu_accel_sigma_       = get_parameter("noise.imu.accel_sigma").as_double();
  noise_imu_gyro_sigma_        = get_parameter("noise.imu.gyro_sigma").as_double();
  noise_imu_accel_bias_sigma_  = get_parameter("noise.imu.accel_bias_sigma").as_double();
  noise_imu_gyro_bias_sigma_   = get_parameter("noise.imu.gyro_bias_sigma").as_double();
  noise_imu_integration_sigma_ = get_parameter("noise.imu.integration_sigma").as_double();
  imu_gravity_                 = get_parameter("noise.imu.gravity").as_double();

  noise_lidar_x_     = get_parameter("noise.lidar.x").as_double();
  noise_lidar_y_     = get_parameter("noise.lidar.y").as_double();
  noise_lidar_z_     = get_parameter("noise.lidar.z").as_double();
  noise_lidar_roll_  = get_parameter("noise.lidar.roll").as_double();
  noise_lidar_pitch_ = get_parameter("noise.lidar.pitch").as_double();
  noise_lidar_yaw_   = get_parameter("noise.lidar.yaw").as_double();

  icp_fitness_score_threshold_ = get_parameter("noise.lidar.icp_fitness_score_threshold").as_double();
  lidar_rotation_gate_rad_     = get_parameter("noise.lidar.rotation_gate_rad").as_double();
  lidar_fitness_noise_scale_   = get_parameter("noise.lidar.fitness_noise_scale").as_double();
  max_scan_age_sec_            = get_parameter("noise.lidar.max_scan_age_sec").as_double();

  isam2_relinearize_threshold_ = get_parameter("isam2.relinearize_threshold").as_double();
  isam2_relinearize_skip_      = get_parameter("isam2.relinearize_skip").as_int();
  optimization_rate_hz_        = get_parameter("isam2.optimization_rate_hz").as_double();

  keyframe_translation_threshold_ = get_parameter("keyframe.translation_threshold").as_double();
  keyframe_rotation_threshold_    = get_parameter("keyframe.rotation_threshold").as_double();
}

// ═════════════════════════════════════════════════════════════════════════════
// iSAM2 initialisation
// ═════════════════════════════════════════════════════════════════════════════

void FgoNode::initIsam2()
{
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = isam2_relinearize_threshold_;
  params.relinearizeSkip      = isam2_relinearize_skip_;
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
  auto p = gtsam::PreintegrationCombinedParams::MakeSharedU(imu_gravity_);

  // White-noise spectral densities (continuous time)
  p->accelerometerCovariance =
    gtsam::I_3x3 * (noise_imu_accel_sigma_ * noise_imu_accel_sigma_);
  p->gyroscopeCovariance =
    gtsam::I_3x3 * (noise_imu_gyro_sigma_ * noise_imu_gyro_sigma_);

  // Numerical integration uncertainty
  p->integrationCovariance =
    gtsam::I_3x3 * (noise_imu_integration_sigma_ * noise_imu_integration_sigma_);

  // Bias random-walk spectral densities
  p->biasAccCovariance =
    gtsam::I_3x3 * (noise_imu_accel_bias_sigma_ * noise_imu_accel_bias_sigma_);
  p->biasOmegaCovariance =
    gtsam::I_3x3 * (noise_imu_gyro_bias_sigma_ * noise_imu_gyro_bias_sigma_);

  // Small uncertainty on initial bias integral (numerical stability)
  p->biasAccOmegaInt = gtsam::I_6x6 * 1e-5;

  imu_preint_params_ = p;

  RCLCPP_INFO(get_logger(),
    "[FgoNode] IMU preintegration params set. "
    "accel_sigma=%.4f gyro_sigma=%.4f gravity=%.2f",
    noise_imu_accel_sigma_, noise_imu_gyro_sigma_, imu_gravity_);
}

void FgoNode::initGraph()
{
  key_ = 0;
  new_factors_.resize(0);
  new_values_.clear();

  // ── X(0): tight prior on initial pose ───────────────────────────────────────────
  gtsam::Pose3 init_pose(
    gtsam::Rot3::RzRyRx(init_roll_, init_pitch_, init_yaw_),
    gtsam::Point3(init_x_, init_y_, init_z_));

  constexpr double kPosSigma = 0.001;  // 1 mm
  constexpr double kRotSigma = 0.001;  // ~0.057 deg
  auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector6() << kRotSigma, kRotSigma, kRotSigma,
                         kPosSigma, kPosSigma, kPosSigma).finished());

  new_factors_.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), init_pose, prior_noise));
  new_values_.insert(X(0), init_pose);

  // ── V(0) and B(0): velocity and bias priors (only when IMU is enabled) ────
  if (enable_imu_ && imu_preint_params_) {
    optimized_velocity_ = gtsam::Vector3::Zero();
    optimized_bias_     = gtsam::imuBias::ConstantBias();

    // Robot starts from rest — 1 m/s uncertainty (generous)
    auto vel_noise = gtsam::noiseModel::Isotropic::Sigma(3, 1.0);
    new_factors_.add(
      gtsam::PriorFactor<gtsam::Vector3>(V(0), optimized_velocity_, vel_noise));
    new_values_.insert(V(0), optimized_velocity_);

    // Bias assumed near-zero at startup; moderate uncertainty lets GTSAM estimate it
    auto bias_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector6() << 0.3, 0.3, 0.3, 0.05, 0.05, 0.05).finished());
    new_factors_.add(
      gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), optimized_bias_, bias_noise));
    new_values_.insert(B(0), optimized_bias_);
  }

  isam2_->update(new_factors_, new_values_);
  new_factors_.resize(0);
  new_values_.clear();

  optimized_pose_ = init_pose;

  RCLCPP_INFO(get_logger(), "[FgoNode] Graph initialised with X(0) at (%.2f, %.2f, yaw=%.2f)",
    init_x_, init_y_, init_yaw_);
}

// ═════════════════════════════════════════════════════════════════════════════
// Odometry callback
// ═════════════════════════════════════════════════════════════════════════════

void FgoNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & raw_pose = msg->pose.pose;

  // 1. Always publish odom → base_footprint using raw odometry
  if (publish_odom_to_base_) {
    publishOdomToBase(msg->header.stamp, raw_pose);
  }

  // 2. Evaluate keyframe condition
  // Compute delta position
  const double dx = raw_pose.position.x - last_keyframe_odom_pose_.position.x;
  const double dy = raw_pose.position.y - last_keyframe_odom_pose_.position.y;
  const double dz = raw_pose.position.z - last_keyframe_odom_pose_.position.z;
  const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

  // Compute delta yaw
  auto get_yaw = [](const geometry_msgs::msg::Quaternion & q) -> double {
    tf2::Quaternion tq(q.x, q.y, q.z, q.w);
    double r, p, y;
    tf2::Matrix3x3(tq).getRPY(r, p, y);
    return y;
  };
  const double yaw_curr = get_yaw(raw_pose.orientation);
  const double yaw_last = get_yaw(last_keyframe_odom_pose_.orientation);
  // Wrap delta yaw to [0, π]
  double dyaw = std::fmod(
    std::fabs(yaw_curr - yaw_last), 2.0 * M_PI);
  if (dyaw > M_PI) dyaw = 2.0 * M_PI - dyaw;

  const bool threshold_met = (dist > keyframe_translation_threshold_) ||
                             (dyaw > keyframe_rotation_threshold_);

  if (threshold_met) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    OdomSample s;
    s.pose      = raw_pose;
    s.timestamp = rclcpp::Time(msg->header.stamp);
    odom_buffer_.push_back(s);
    last_keyframe_odom_pose_ = raw_pose;
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

  std::lock_guard<std::mutex> lock(imu_mutex_);
  imu_buffer_.push_back(sample);
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
  if (age_s > max_scan_age_sec_) {
    RCLCPP_DEBUG(get_logger(),
      "[FgoNode] Scan match discarded: age=%.3fs > max=%.3fs", age_s, max_scan_age_sec_);
    return;
  }

  // Gate 2: fitness score (stored in non-standard covariance[34] slot)
  const double fitness_score = msg->pose.covariance[34];
  if (fitness_score > icp_fitness_score_threshold_) {
    RCLCPP_DEBUG(get_logger(),
      "[FgoNode] Scan match discarded: fitness=%.3f > threshold=%.3f",
      fitness_score, icp_fitness_score_threshold_);
    return;
  }

  // Gate 3: cap buffer size — prevents unbounded growth during long stationary periods
  std::lock_guard<std::mutex> lock(scan_mutex_);
  if (scan_pose_buffer_.size() >= 10) {
    scan_pose_buffer_.erase(scan_pose_buffer_.begin());
  }
  scan_pose_buffer_.push_back(*msg);
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
  init_x_     = new_pose.position.x;
  init_y_     = new_pose.position.y;
  init_z_     = new_pose.position.z;
  init_roll_  = new_roll;
  init_pitch_ = new_pitch;
  init_yaw_   = new_yaw;

  // Clear all buffers
  {
    std::lock_guard<std::mutex> l(odom_mutex_);
    odom_buffer_.clear();
    last_keyframe_odom_pose_ = new_pose;
    last_consumed_odom_pose_ = new_pose;
  }
  {
    std::lock_guard<std::mutex> l(imu_mutex_);
    imu_buffer_.clear();
  }
  {
    std::lock_guard<std::mutex> l(scan_mutex_);
    scan_pose_buffer_.clear();
  }

  // Reset IMU state
  optimized_velocity_       = gtsam::Vector3::Zero();
  optimized_bias_           = gtsam::imuBias::ConstantBias();
  last_consumed_odom_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // Reset path
  path_msg_.poses.clear();
  path_msg_.header.stamp = now();

  // Reset cached map→odom so stale transform is not republished
  has_map_to_odom_cache_ = false;

  // Reset iSAM2 and re-add X(0)
  initIsam2();
  initGraph();

  // After initGraph(), update the map→odom cache to the new initial pose
  updateMapToOdomCache();

  RCLCPP_INFO(get_logger(), "[FgoNode] Graph reset to (%.2f, %.2f, yaw=%.2f)",
    init_x_, init_y_, init_yaw_);
}

// ═════════════════════════════════════════════════════════════════════════════
// Optimization timer callback
// ═════════════════════════════════════════════════════════════════════════════

void FgoNode::optimizationStep()
{
  // ── Drain odom buffer ─────────────────────────────────────────────────────
  std::vector<OdomSample> local_odom;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    local_odom.swap(odom_buffer_);
  }

  // ── Drain IMU buffer ──────────────────────────────────────────────────────
  std::vector<ImuSample> local_imu;
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    local_imu.swap(imu_buffer_);
  }

  // ── Drain scan pose buffer ────────────────────────────────────────────────
  std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> local_scan;
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    local_scan.swap(scan_pose_buffer_);
  }

  // If no new keyframes, re-publish the CACHED map→odom transform to prevent
  // Nav2 TF timeout.  We must NOT recompute here: recomputing with the
  // current (live) last_raw_odom_pose_ would cancel out the odom→base TF and
  // freeze the robot's apparent position in the map frame.
  if (local_odom.empty()) {
    if (has_map_to_odom_cache_ && publish_map_to_odom_) {
      publishMapToOdom(now());
    }
    return;
  }

  // ── Noise models ──────────────────────────────────────────────────────────
  auto odom_noise = makeDiagonalNoise(
    noise_odom_x_, noise_odom_y_, noise_odom_z_,
    noise_odom_roll_, noise_odom_pitch_, noise_odom_yaw_);

  // ── Build odometry BetweenFactors ─────────────────────────────────────────
  // The first delta is computed from last_consumed_odom_pose_ (the keyframe
  // that corresponds to the last optimised state), so the graph is consistent.
  geometry_msgs::msg::Pose prev_pose = last_consumed_odom_pose_;
  // Snapshot of the pose BEFORE this batch: used for rotation gate calculation.
  // Must be captured before the loop overwrites last_consumed_odom_pose_.
  const geometry_msgs::msg::Pose pre_batch_odom_pose = last_consumed_odom_pose_;
  const int batch_start_key = key_;  // key index BEFORE this batch

  for (const auto & sample : local_odom) {
    const gtsam::Pose3 gtsam_prev    = msgToGtsam(prev_pose);
    const gtsam::Pose3 gtsam_current = msgToGtsam(sample.pose);
    const gtsam::Pose3 delta         = gtsam_prev.between(gtsam_current);

    // Initial value guess for new key: propagate from last optimised pose
    gtsam::Pose3 estimate;
    try {
      estimate = isam2_->calculateEstimate<gtsam::Pose3>(X(key_)).compose(delta);
    } catch (...) {
      estimate = optimized_pose_.compose(delta);
    }

    key_++;
    new_factors_.add(
      gtsam::BetweenFactor<gtsam::Pose3>(X(key_ - 1), X(key_), delta, odom_noise));
    new_values_.insert(X(key_), estimate);

    prev_pose = sample.pose;
  }
  last_consumed_odom_pose_  = local_odom.back().pose;
  // Save OLD stamp before overwriting: needed as t_from for the first IMU interval
  const rclcpp::Time prev_consumed_stamp = last_consumed_odom_stamp_;
  last_consumed_odom_stamp_ = local_odom.back().timestamp;

  // ── CombinedImuFactor: preintegrate IMU per keyframe interval ────────────
  // V and B states must exist for every new key regardless of whether IMU
  // data is available this batch.  If insertion is skipped when local_imu is
  // empty, the next batch that has IMU data will reference V(k)/B(k) that
  // were never committed to iSAM2 -> exception or silent graph corruption.
  if (enable_imu_ && imu_preint_params_) {
    for (int i = 0; i < static_cast<int>(local_odom.size()); ++i) {
      const int to_key = batch_start_key + i + 1;
      if (!new_values_.exists(V(to_key))) {
        new_values_.insert(V(to_key), optimized_velocity_);
      }
      if (!new_values_.exists(B(to_key))) {
        new_values_.insert(B(to_key), optimized_bias_);
      }
    }
  }

  if (enable_imu_ && imu_preint_params_ && !local_imu.empty()) {
    // Sort IMU samples by timestamp (defensive guard)
    std::sort(local_imu.begin(), local_imu.end(),
      [](const ImuSample & a, const ImuSample & b) {
        return a.timestamp < b.timestamp;
      });

    for (int i = 0; i < static_cast<int>(local_odom.size()); ++i) {
      const int from_key = batch_start_key + i;
      const int to_key   = from_key + 1;

      // Time window for this interval
      const rclcpp::Time t_from =
        (i == 0) ? prev_consumed_stamp          // OLD last-consumed stamp
                 : local_odom[i - 1].timestamp;
      const rclcpp::Time t_to = local_odom[i].timestamp;

      // Skip if t_from not yet initialised (first ever batch after startup)
      if (t_from.nanoseconds() == 0 || t_to <= t_from) continue;

      gtsam::PreintegratedCombinedMeasurements preint(imu_preint_params_, optimized_bias_);
      int n_integrated = 0;

      for (std::size_t j = 1; j < local_imu.size(); ++j) {
        const rclcpp::Time & t_a = local_imu[j - 1].timestamp;
        const rclcpp::Time & t_b = local_imu[j].timestamp;

        // Reject samples fully outside the window
        if (t_b <= t_from || t_a >= t_to) continue;

        // Clamp to [t_from, t_to] so boundary-straddling samples are NOT
        // double-counted across consecutive keyframe intervals.
        const double t_a_eff = std::max(t_a.seconds(), t_from.seconds());
        const double t_b_eff = std::min(t_b.seconds(), t_to.seconds());
        const double dt = t_b_eff - t_a_eff;
        if (dt <= 0.0 || dt > 0.5) continue;  // sanity guard

        preint.integrateMeasurement(local_imu[j - 1].accel,
                                    local_imu[j - 1].gyro, dt);
        ++n_integrated;
      }

      // V/B values already pre-inserted in the unconditional block above.

      if (n_integrated == 0) continue;

      // CombinedImuFactor constrains X, V, B at both ends of the interval
      new_factors_.add(gtsam::CombinedImuFactor(
        X(from_key), V(from_key),
        X(to_key),   V(to_key),
        B(from_key), B(to_key),
        preint));
    }
  }

  // ── Scan match PriorFactors ───────────────────────────────────────────────
  // Rotation gate: skip ALL scan priors if the robot rotated more than
  // lidar_rotation_gate_rad_ this batch.  NDT is unreliable during in-place
  // rotation and converges to a wrong local minimum, permanently shifting the graph.
  if (!local_scan.empty()) {
    auto get_yaw_pose = [](const geometry_msgs::msg::Pose & p) -> double {
      tf2::Quaternion tq(p.orientation.x, p.orientation.y,
                         p.orientation.z, p.orientation.w);
      double r, pi, y;
      tf2::Matrix3x3(tq).getRPY(r, pi, y);
      return y;
    };
    double batch_dyaw = 0.0;
    double yaw_prev = get_yaw_pose(pre_batch_odom_pose);
    for (const auto & s : local_odom) {
      double dy = std::fmod(std::fabs(get_yaw_pose(s.pose) - yaw_prev), 2.0 * M_PI);
      if (dy > M_PI) dy = 2.0 * M_PI - dy;
      batch_dyaw += dy;
      yaw_prev = get_yaw_pose(s.pose);
    }

    if (batch_dyaw > lidar_rotation_gate_rad_) {
      RCLCPP_DEBUG(get_logger(),
        "[FgoNode] Scan prior skipped: batch |dyaw|=%.4f rad > gate=%.4f rad",
        batch_dyaw, lidar_rotation_gate_rad_);
    } else {
      for (const auto & scan_msg : local_scan) {
        const rclcpp::Time scan_t(scan_msg.header.stamp);
        const gtsam::Pose3 scan_pose = msgToGtsam(scan_msg.pose.pose);
        const double fitness = scan_msg.pose.covariance[34];

        // Adaptive noise: sigma *= (1 + fitness_noise_scale * fitness_score)
        const double scale = 1.0 + lidar_fitness_noise_scale_ * fitness;
        auto lidar_noise = makeDiagonalNoise(
          noise_lidar_x_ * scale, noise_lidar_y_ * scale, noise_lidar_z_,
          noise_lidar_roll_,       noise_lidar_pitch_,
          noise_lidar_yaw_ * scale);

        // Find nearest keyframe by timestamp
        int best_i = static_cast<int>(local_odom.size()) - 1;
        double best_dt = std::numeric_limits<double>::max();
        for (int i = 0; i < static_cast<int>(local_odom.size()); ++i) {
          const double dt = std::fabs((local_odom[i].timestamp - scan_t).seconds());
          if (dt < best_dt) { best_dt = dt; best_i = i; }
        }
        const int target_key = batch_start_key + best_i + 1;

        new_factors_.add(
          gtsam::PriorFactor<gtsam::Pose3>(X(target_key), scan_pose, lidar_noise));

        RCLCPP_DEBUG(get_logger(),
          "[FgoNode] Scan prior -> X(%d), dt=%.3fs, fitness=%.3f, scale=%.2f",
          target_key, best_dt, fitness, scale);
      }
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
    if (enable_imu_ && imu_preint_params_) {
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

  // ── Update and publish map → odom TF ─────────────────────────────────────
  // updateMapToOdomCache() anchors map→odom using last_consumed_odom_pose_
  // (the keyframe pose that CORRESPONDS to optimized_pose_).  This ensures
  // that between optimisation steps the raw odom dead-reckons from the
  // keyframe correctly, instead of freezing the robot in the map frame.
  if (publish_map_to_odom_) {
    updateMapToOdomCache();
    publishMapToOdom(now());
  }

  // ── Publish /fgo/odometry ─────────────────────────────────────────────────
  {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp    = now();
    odom_msg.header.frame_id = map_frame_;
    odom_msg.child_frame_id  = base_frame_;
    odom_msg.pose.pose       = gtsamToMsg(optimized_pose_);
    pub_odometry_->publish(odom_msg);
  }

  // ── Publish /fgo/path ─────────────────────────────────────────────────────
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp    = now();
    ps.header.frame_id = map_frame_;
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
  ts.header.frame_id = odom_frame_;
  ts.child_frame_id  = base_frame_;

  ts.transform.translation.x = raw_pose.position.x;
  ts.transform.translation.y = raw_pose.position.y;
  ts.transform.translation.z = raw_pose.position.z;
  ts.transform.rotation      = raw_pose.orientation;

  tf_broadcaster_->sendTransform(ts);
}

void FgoNode::updateMapToOdomCache()
{
  // T(map→odom) = T(map→base)_opt × T(odom→base)_keyframe⁻¹
  //
  // IMPORTANT: we use last_consumed_odom_pose_ (the KEYFRAME odom pose that
  // CORRESPONDS to optimized_pose_), NOT last_raw_odom_pose_ (the live pose).
  //
  // Using the live raw pose here would cause the map→odom and odom→base
  // transforms to cancel each other out whenever no new keyframes exist,
  // freezing the robot's apparent position in the map frame during rotation.
  //
  // With the keyframe anchor the TF chain works correctly:
  //   map→base = (map→odom_cached) × (odom→base_live)
  //            = (optimized × keyframe_odom⁻¹) × live_odom
  //            = optimized × Δodom(keyframe → live)    ← dead-reckon ✓
  const gtsam::Pose3 odom_T_base = msgToGtsam(last_consumed_odom_pose_);
  const gtsam::Pose3 map_T_odom  = optimized_pose_.compose(odom_T_base.inverse());

  const gtsam::Point3 & t = map_T_odom.translation();
  const gtsam::Quaternion q = map_T_odom.rotation().toQuaternion();

  cached_map_to_odom_tf_.header.frame_id = map_frame_;
  cached_map_to_odom_tf_.child_frame_id  = odom_frame_;
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

// ═════════════════════════════════════════════════════════════════════════════
// Conversion helpers
// ═════════════════════════════════════════════════════════════════════════════

gtsam::Pose3 FgoNode::msgToGtsam(const geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion tq(
    pose.orientation.x, pose.orientation.y,
    pose.orientation.z, pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);

  return gtsam::Pose3(
    gtsam::Rot3::RzRyRx(roll, pitch, yaw),
    gtsam::Point3(pose.position.x, pose.position.y, pose.position.z));
}

geometry_msgs::msg::Pose FgoNode::gtsamToMsg(const gtsam::Pose3 & pose)
{
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.translation().x();
  msg.position.y = pose.translation().y();
  msg.position.z = pose.translation().z();

  const gtsam::Quaternion q = pose.rotation().toQuaternion();
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();
  return msg;
}

gtsam::noiseModel::Diagonal::shared_ptr FgoNode::makeDiagonalNoise(
  double x, double y, double z,
  double roll, double pitch, double yaw)
{
  // GTSAM Pose3 sigma order: [roll, pitch, yaw, x, y, z]
  return gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector6() << roll, pitch, yaw, x, y, z).finished());
}

}  // namespace factor_graph_optimization
