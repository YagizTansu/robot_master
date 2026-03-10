#include "factor_graph_optimization/fgo_node.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// GTSAM

namespace factor_graph_optimization
{

// ═════════════════════════════════════════════════════════════════════════════
// Construction
// ═════════════════════════════════════════════════════════════════════════════

FgoNode::FgoNode(const rclcpp::NodeOptions & options) : rclcpp::Node("fgo_node", options)
{
  cfg_ = FgoConfig::fromNode(*this);

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

  // ── Graph manager (owns iSAM2, factor graph, IMU preintegrator) ───────────
  std::unique_ptr<ImuPreintegrator> imu_preint;
  if (cfg_.enable_imu) {
    imu_preint = std::make_unique<ImuPreintegrator>(cfg_);
    RCLCPP_INFO(get_logger(),
      "[FgoNode] IMU preintegration enabled. "
      "accel_sigma=%.4f gyro_sigma=%.4f gravity=%.2f",
      cfg_.noise_imu_accel_sigma, cfg_.noise_imu_gyro_sigma, cfg_.imu_gravity);
  }
  graph_mgr_ = std::make_unique<GraphManager>(cfg_, std::move(imu_preint));

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

void FgoNode::scanPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
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
  scan_buf_.push(*msg, cfg_.max_pending_scans);
}

// ═════════════════════════════════════════════════════════════════════════════
// Initial pose callback — full reset
// ═════════════════════════════════════════════════════════════════════════════

void FgoNode::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
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
  path_msg_.poses.clear();
  path_msg_.header.stamp = now();
  has_map_to_odom_cache_ = false;

  graph_mgr_->reinit(cfg_);
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

  if (!graph_mgr_->step(local_odom, std::move(local_imu), local_scan, get_logger())) {
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
    odom_msg.header.stamp    = graph_mgr_->lastConsumedOdomStamp();
    odom_msg.header.frame_id = cfg_.map_frame;
    odom_msg.child_frame_id  = cfg_.base_frame;
    odom_msg.pose.pose       = gtsamToMsg(graph_mgr_->optimizedPose());
    pub_odometry_->publish(odom_msg);
  }

  // ── Publish /fgo/path ─────────────────────────────────────────────────────
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp    = now();
    ps.header.frame_id = cfg_.map_frame;
    ps.pose            = gtsamToMsg(graph_mgr_->optimizedPose());
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
  const gtsam::Pose3 odom_T_base = msgToGtsam(graph_mgr_->lastConsumedOdomPose());
  const gtsam::Pose3 map_T_odom  = graph_mgr_->optimizedPose().compose(odom_T_base.inverse());

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
