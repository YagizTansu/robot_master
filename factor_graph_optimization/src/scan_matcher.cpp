#include "factor_graph_optimization/scan_matcher.hpp"

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "factor_graph_optimization/lidar/ndt_matcher.hpp"
#include "factor_graph_optimization/lidar/icp_matcher.hpp"

namespace factor_graph_optimization
{

// ═════════════════════════════════════════════════════════════════════════════
// Construction
// ═════════════════════════════════════════════════════════════════════════════

ScanMatcherNode::ScanMatcherNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("scan_matcher_node", options)
{
  cfg_ = ScanMatcherConfig::fromNode(*this);

  // ── Build lidar module objects ─────────────────────────────────────────────
  map_builder_ = std::make_unique<MapBuilder>(
    cfg_.map_z_height, cfg_.map_voxel_leaf_size);

  if (cfg_.scan_matcher_type == "NDT") {
    matcher_ = std::make_unique<NdtMatcher>(
      cfg_.max_iterations,
      cfg_.max_correspondence_dist,
      cfg_.transformation_epsilon,
      cfg_.ndt_resolution,
      cfg_.ndt_step_size);
  } else {
    matcher_ = std::make_unique<IcpMatcher>(
      cfg_.max_iterations,
      cfg_.max_correspondence_dist,
      cfg_.transformation_epsilon);
  }

  // ── TF2 buffer + listener (needed by LaserProjection) ─────────────────────
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ── Map subscription — transient_local QoS (to receive latched map) ───────
  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.transient_local().reliable();

  sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    cfg_.map_topic, map_qos,
    std::bind(&ScanMatcherNode::mapCallback, this, std::placeholders::_1));

  // ── Scan subscription ─────────────────────────────────────────────────────
  sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
    cfg_.lidar_topic, rclcpp::QoS(10),
    std::bind(&ScanMatcherNode::scanCallback, this, std::placeholders::_1));

  // ── FGO pose subscription (for NDT initial guess) ─────────────────────────
  sub_fgo_pose_ = create_subscription<nav_msgs::msg::Odometry>(
    "/fgo/odometry", rclcpp::QoS(10),
    std::bind(&ScanMatcherNode::fgoPoseCallback, this, std::placeholders::_1));

  // ── Publisher ─────────────────────────────────────────────────────────────
  pub_scan_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    cfg_.scan_match_pose_topic, rclcpp::QoS(10));

  RCLCPP_INFO(get_logger(),
    "[ScanMatcherNode] Started. type=%s lidar_frame=%s",
    cfg_.scan_matcher_type.c_str(), cfg_.lidar_frame.c_str());
}

// ═════════════════════════════════════════════════════════════════════════════
// Map callback
// ═════════════════════════════════════════════════════════════════════════════

void ScanMatcherNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "[ScanMatcherNode] Map received (%zu × %zu). Building PCL cloud...",
    static_cast<std::size_t>(msg->info.width),
    static_cast<std::size_t>(msg->info.height));

  map_cloud_    = map_builder_->build(*msg);
  map_received_ = true;

  RCLCPP_INFO(get_logger(),
    "[ScanMatcherNode] Map cloud ready: %zu points (leaf=%.2fm).",
    map_cloud_->size(), cfg_.map_voxel_leaf_size);
}

// ═════════════════════════════════════════════════════════════════════════════
// Scan callback
// ═════════════════════════════════════════════════════════════════════════════

void ScanMatcherNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!map_received_) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
      "[ScanMatcherNode] Waiting for map...");
    return;
  }

  // ── Convert LaserScan → sensor_msgs::PointCloud2 via LaserProjection ──────
  sensor_msgs::msg::PointCloud2 pc2_msg;
  try {
    projector_.transformLaserScanToPointCloud(
      cfg_.lidar_frame, *msg, pc2_msg, *tf_buffer_);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "[ScanMatcherNode] TF error in laser projection: %s", ex.what());
    return;
  }

  // ── Convert to PCL ────────────────────────────────────────────────────────
  auto source_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(pc2_msg, *source_cloud);

  if (source_cloud->empty()) {
    RCLCPP_WARN(get_logger(), "[ScanMatcherNode] Empty scan cloud — skipping.");
    return;
  }

  // ── Run matching ──────────────────────────────────────────────────────────
  const Eigen::Matrix4f initial_guess = buildInitialGuess();
  Eigen::Matrix4f result_transform    = Eigen::Matrix4f::Identity();

  const double fitness_score =
    matcher_->match(source_cloud, map_cloud_, initial_guess, result_transform);

  // ── Fitness gate ──────────────────────────────────────────────────────────
  if (fitness_score > cfg_.fitness_score_threshold) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
      "[ScanMatcherNode] Scan match discarded: fitness=%.3f > threshold=%.3f",
      fitness_score, cfg_.fitness_score_threshold);
    return;
  }

  // Adaptive noise: sigma *= (1 + fitness_noise_scale * fitness_score)
  const double scan_noise_scale = 1.0 + cfg_.fitness_noise_scale * fitness_score;

  // ── Enforce 2D constraints ────────────────────────────────────────────────
  result_transform = enforce2D(result_transform);

  // ── Extract pose from transform ───────────────────────────────────────────
  const float tx  = result_transform(0, 3);
  const float ty  = result_transform(1, 3);

  // Extract yaw (roll and pitch are 0 after enforce2D)
  Eigen::Matrix3f rot = result_transform.block<3, 3>(0, 0);
  const float yaw = std::atan2(rot(1, 0), rot(0, 0));

  // Convert to quaternion
  Eigen::Quaternionf q =
    Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(yaw,  Eigen::Vector3f::UnitZ());
  q.normalize();

  // ── Build output message ──────────────────────────────────────────────────
  geometry_msgs::msg::PoseWithCovarianceStamped out;
  out.header.stamp    = msg->header.stamp;
  out.header.frame_id = "map";

  out.pose.pose.position.x    = static_cast<double>(tx);
  out.pose.pose.position.y    = static_cast<double>(ty);
  out.pose.pose.position.z    = 0.0;
  out.pose.pose.orientation.x = static_cast<double>(q.x());
  out.pose.pose.orientation.y = static_cast<double>(q.y());
  out.pose.pose.orientation.z = static_cast<double>(q.z());
  out.pose.pose.orientation.w = static_cast<double>(q.w());

  // Diagonal covariance [x,y,z,roll,pitch,yaw] — adaptive scaling on planar DOF
  out.pose.covariance.fill(0.0);
  out.pose.covariance[0]  = std::pow(cfg_.noise_lidar_x   * scan_noise_scale, 2.0);
  out.pose.covariance[7]  = std::pow(cfg_.noise_lidar_y   * scan_noise_scale, 2.0);
  out.pose.covariance[14] = std::pow(cfg_.noise_lidar_z,                      2.0);
  out.pose.covariance[21] = std::pow(cfg_.noise_lidar_roll,                   2.0);
  out.pose.covariance[28] = std::pow(cfg_.noise_lidar_pitch,                  2.0);
  out.pose.covariance[35] = std::pow(cfg_.noise_lidar_yaw * scan_noise_scale, 2.0);

  pub_scan_pose_->publish(out);
}

// ═════════════════════════════════════════════════════════════════════════════
// FGO pose callback (for initial guess)
// ═════════════════════════════════════════════════════════════════════════════

void ScanMatcherNode::fgoPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_fgo_pose_ = msg->pose.pose;
  has_fgo_pose_     = true;
}

// ═════════════════════════════════════════════════════════════════════════════
// Helpers
// ═════════════════════════════════════════════════════════════════════════════

Eigen::Matrix4f ScanMatcherNode::buildInitialGuess() const
{
  if (!has_fgo_pose_) {
    return Eigen::Matrix4f::Identity();
  }

  const double yaw = extractYaw(current_fgo_pose_.orientation);

  Eigen::Affine3f affine = Eigen::Affine3f::Identity();
  affine.translation() << static_cast<float>(current_fgo_pose_.position.x),
                          static_cast<float>(current_fgo_pose_.position.y),
                          0.0f;
  affine.rotate(Eigen::AngleAxisf(static_cast<float>(yaw), Eigen::Vector3f::UnitZ()));
  return affine.matrix();
}

}  // namespace factor_graph_optimization
