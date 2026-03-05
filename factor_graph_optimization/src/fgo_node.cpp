#include "factor_graph_optimization/fgo_node.hpp"

#include <cmath>
#include <chrono>

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

  // ── TF broadcaster ───────────────────────────────────────────────────────
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

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
  last_raw_odom_pose_       = make_init_pose();

  // ── Initialise graph: add X(0) prior ─────────────────────────────────────
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

  // IMU noise
  declare_parameter("noise.imu.gyroscope_z_sigma", 0.001);
  declare_parameter("noise.imu.x",     999.0);
  declare_parameter("noise.imu.y",     999.0);
  declare_parameter("noise.imu.z",     999.0);
  declare_parameter("noise.imu.roll",  999.0);
  declare_parameter("noise.imu.pitch", 999.0);

  // LiDAR noise + gating
  declare_parameter("noise.lidar.x",     0.1);
  declare_parameter("noise.lidar.y",     0.1);
  declare_parameter("noise.lidar.z",     999.0);
  declare_parameter("noise.lidar.roll",  999.0);
  declare_parameter("noise.lidar.pitch", 999.0);
  declare_parameter("noise.lidar.yaw",   0.1);
  declare_parameter("noise.lidar.icp_fitness_score_threshold", 0.5);
  declare_parameter("noise.lidar.icp_covariance_threshold",    0.3);

  // iSAM2
  declare_parameter("isam2.relinearize_threshold", 0.1);
  declare_parameter("isam2.relinearize_skip",       1);
  declare_parameter("isam2.optimization_rate_hz",  10.0);

  // Keyframe
  declare_parameter("keyframe.translation_threshold", 0.2);
  declare_parameter("keyframe.rotation_threshold",    0.1);
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

  noise_imu_gyro_z_sigma_ = get_parameter("noise.imu.gyroscope_z_sigma").as_double();
  noise_imu_x_     = get_parameter("noise.imu.x").as_double();
  noise_imu_y_     = get_parameter("noise.imu.y").as_double();
  noise_imu_z_     = get_parameter("noise.imu.z").as_double();
  noise_imu_roll_  = get_parameter("noise.imu.roll").as_double();
  noise_imu_pitch_ = get_parameter("noise.imu.pitch").as_double();

  noise_lidar_x_     = get_parameter("noise.lidar.x").as_double();
  noise_lidar_y_     = get_parameter("noise.lidar.y").as_double();
  noise_lidar_z_     = get_parameter("noise.lidar.z").as_double();
  noise_lidar_roll_  = get_parameter("noise.lidar.roll").as_double();
  noise_lidar_pitch_ = get_parameter("noise.lidar.pitch").as_double();
  noise_lidar_yaw_   = get_parameter("noise.lidar.yaw").as_double();

  icp_fitness_score_threshold_ = get_parameter("noise.lidar.icp_fitness_score_threshold").as_double();
  icp_covariance_threshold_    = get_parameter("noise.lidar.icp_covariance_threshold").as_double();

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

void FgoNode::initGraph()
{
  key_ = 0;
  new_factors_.resize(0);
  new_values_.clear();

  // Add X(0) with prior from initial_pose parameters
  gtsam::Pose3 init_pose(
    gtsam::Rot3::RzRyRx(init_roll_, init_pitch_, init_yaw_),
    gtsam::Point3(init_x_, init_y_, init_z_));

  // Use a fixed tight prior on starting pose (1 mm / 0.001 rad).
  // A constant sigma is simpler and avoids Eigen operator-precedence pitfalls.
  constexpr double kPosSigma = 0.001;  // 1 mm
  constexpr double kRotSigma = 0.001;  // ~0.057 deg
  auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector6() << kRotSigma, kRotSigma, kRotSigma,
                         kPosSigma, kPosSigma, kPosSigma).finished());

  new_factors_.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), init_pose, prior_noise));
  new_values_.insert(X(0), init_pose);

  isam2_->update(new_factors_, new_values_);
  new_factors_.resize(0);
  new_values_.clear();

  optimized_pose_ = init_pose;
  has_optimized_pose_ = true;

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

  last_raw_odom_pose_ = raw_pose;

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
  double dyaw = std::fabs(yaw_curr - yaw_last);
  // Wrap to [-π, π]
  while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
  dyaw = std::fabs(dyaw);

  const bool threshold_met = (dist > keyframe_translation_threshold_) ||
                             (dyaw > keyframe_rotation_threshold_);

  if (threshold_met) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_buffer_.push_back(raw_pose);
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
  sample.gyro_z    = msg->angular_velocity.z;

  std::lock_guard<std::mutex> lock(imu_mutex_);
  imu_buffer_.push_back(sample);
}

// ═════════════════════════════════════════════════════════════════════════════
// Scan pose callback (from scan_matcher_node)
// ═════════════════════════════════════════════════════════════════════════════

void FgoNode::scanPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // Extract fitness score from covariance[34] (non-standard slot)
  const double fitness_score = msg->pose.covariance[34];

  if (fitness_score > icp_fitness_score_threshold_) {
    RCLCPP_DEBUG(get_logger(),
      "[FgoNode] Scan match discarded: fitness=%.3f > threshold=%.3f",
      fitness_score, icp_fitness_score_threshold_);
    return;
  }

  // Check meaningful covariance diagonal elements: index 0 (x), 7 (y), 35 (yaw)
  const double cov_x   = msg->pose.covariance[0];
  const double cov_y   = msg->pose.covariance[7];
  const double cov_yaw = msg->pose.covariance[35];

  if (cov_x > icp_covariance_threshold_ ||
      cov_y > icp_covariance_threshold_ ||
      cov_yaw > icp_covariance_threshold_)
  {
    RCLCPP_DEBUG(get_logger(),
      "[FgoNode] Scan match discarded: covariance too large (%.3f, %.3f, %.3f)",
      cov_x, cov_y, cov_yaw);
    return;
  }

  std::lock_guard<std::mutex> lock(scan_mutex_);
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

  // Reset path
  path_msg_.poses.clear();
  path_msg_.header.stamp = now();

  // Reset iSAM2 and re-add X(0)
  initIsam2();
  initGraph();

  RCLCPP_INFO(get_logger(), "[FgoNode] Graph reset to (%.2f, %.2f, yaw=%.2f)",
    init_x_, init_y_, init_yaw_);
}

// ═════════════════════════════════════════════════════════════════════════════
// Optimization timer callback
// ═════════════════════════════════════════════════════════════════════════════

void FgoNode::optimizationStep()
{
  // ── Drain odom buffer ─────────────────────────────────────────────────────
  std::vector<geometry_msgs::msg::Pose> local_odom;
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

  // If no new keyframes, still publish TF to prevent Nav2 TF timeout.
  // Nav2 requires map->odom to be published continuously even when stationary.
  if (local_odom.empty()) {
    if (has_optimized_pose_ && publish_map_to_odom_) {
      publishMapToOdom(now(), optimized_pose_);
    }
    return;
  }

  // ── Noise models ──────────────────────────────────────────────────────────
  auto odom_noise = makeDiagonalNoise(
    noise_odom_x_, noise_odom_y_, noise_odom_z_,
    noise_odom_roll_, noise_odom_pitch_, noise_odom_yaw_);

  // ── Build odometry BetweenFactors ─────────────────────────────────────────
  // The reference for the FIRST delta is last_consumed_odom_pose_
  // (which itself is initialized from initial_pose at startup or /initialpose reset).
  geometry_msgs::msg::Pose prev_pose = last_consumed_odom_pose_;

  for (const auto & current_pose : local_odom) {
    const gtsam::Pose3 gtsam_prev    = msgToGtsam(prev_pose);
    const gtsam::Pose3 gtsam_current = msgToGtsam(current_pose);
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

    prev_pose = current_pose;
  }
  // Update last consumed pose to the last element processed
  last_consumed_odom_pose_ = local_odom.back();

  // ── IMU yaw BetweenFactor (trapezoidal integration) ───────────────────────
  if (!local_imu.empty()) {
    double delta_yaw = 0.0;
    for (std::size_t i = 1; i < local_imu.size(); ++i) {
      const double dt =
        (local_imu[i].timestamp - local_imu[i - 1].timestamp).seconds();
      if (dt <= 0.0 || dt > 1.0) continue;  // sanity guard
      // Trapezoidal rule
      delta_yaw += 0.5 * (local_imu[i - 1].gyro_z + local_imu[i].gyro_z) * dt;
    }

    // Noise: driven entirely from YAML — typically only yaw is constrained
    auto imu_noise = makeDiagonalNoise(
      noise_imu_x_, noise_imu_y_, noise_imu_z_,
      noise_imu_roll_, noise_imu_pitch_, noise_imu_gyro_z_sigma_);

    const gtsam::Pose3 imu_delta(
      gtsam::Rot3::Rz(delta_yaw),
      gtsam::Point3(0.0, 0.0, 0.0));

    // Apply between the FIRST new key in this batch and the PREVIOUS key
    // (i.e. spanning the same interval as the odom batch)
    const int first_new_key = key_ - static_cast<int>(local_odom.size()) + 1;
    const int prev_key      = first_new_key - 1;
    if (prev_key >= 0) {
      new_factors_.add(
        gtsam::BetweenFactor<gtsam::Pose3>(X(prev_key), X(key_), imu_delta, imu_noise));
    }
  }

  // ── Scan match PriorFactors ───────────────────────────────────────────────
  if (!local_scan.empty()) {
    auto lidar_noise = makeDiagonalNoise(
      noise_lidar_x_, noise_lidar_y_, noise_lidar_z_,
      noise_lidar_roll_, noise_lidar_pitch_, noise_lidar_yaw_);

    // Use latest scan match pose as a prior on the current key
    const auto & latest_scan = local_scan.back();
    const gtsam::Pose3 scan_pose = msgToGtsam(latest_scan.pose.pose);
    new_factors_.add(
      gtsam::PriorFactor<gtsam::Pose3>(X(key_), scan_pose, lidar_noise));
  }

  // ── iSAM2 update ─────────────────────────────────────────────────────────
  try {
    isam2_->update(new_factors_, new_values_);
    isam2_->update();  // extra pass for convergence
    new_factors_.resize(0);
    new_values_.clear();

    // Retrieve optimised pose at current key
    optimized_pose_ = isam2_->calculateEstimate<gtsam::Pose3>(X(key_));
    has_optimized_pose_ = true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "[FgoNode] iSAM2 update failed: %s", e.what());
    new_factors_.resize(0);
    new_values_.clear();
    return;
  }

  // ── Publish map → odom TF ────────────────────────────────────────────────
  if (publish_map_to_odom_) {
    publishMapToOdom(now(), optimized_pose_);
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

void FgoNode::publishMapToOdom(const rclcpp::Time & stamp,
                               const gtsam::Pose3 & map_T_base)
{
  // T(map→odom) = T(map→base)_opt × T(odom→base)_raw⁻¹
  // T(odom→base)_raw comes from last raw odom pose
  const gtsam::Pose3 odom_T_base = msgToGtsam(last_raw_odom_pose_);
  const gtsam::Pose3 map_T_odom  = map_T_base.compose(odom_T_base.inverse());

  const gtsam::Point3 & t = map_T_odom.translation();
  const gtsam::Rot3   & R = map_T_odom.rotation();

  // Convert GTSAM rotation to quaternion
  const gtsam::Quaternion q = R.toQuaternion();

  geometry_msgs::msg::TransformStamped ts;
  ts.header.stamp    = stamp;
  ts.header.frame_id = map_frame_;
  ts.child_frame_id  = odom_frame_;

  ts.transform.translation.x = t.x();
  ts.transform.translation.y = t.y();
  ts.transform.translation.z = t.z();
  ts.transform.rotation.x    = q.x();
  ts.transform.rotation.y    = q.y();
  ts.transform.rotation.z    = q.z();
  ts.transform.rotation.w    = q.w();

  tf_broadcaster_->sendTransform(ts);
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
