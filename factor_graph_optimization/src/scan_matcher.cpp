#include "factor_graph_optimization/scan_matcher.hpp"

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

// PCL registration
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>

#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace factor_graph_optimization
{

// ═════════════════════════════════════════════════════════════════════════════
// Construction
// ═════════════════════════════════════════════════════════════════════════════

ScanMatcherNode::ScanMatcherNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("scan_matcher_node", options)
{
  declareParameters();
  loadParameters();

  // ── TF2 buffer + listener (needed by LaserProjection) ─────────────────────
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ── Map subscription — transient_local QoS (to receive latched map) ───────
  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.transient_local().reliable();

  sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, map_qos,
    std::bind(&ScanMatcherNode::mapCallback, this, std::placeholders::_1));

  // ── Scan subscription ─────────────────────────────────────────────────────
  sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
    lidar_topic_, rclcpp::QoS(10),
    std::bind(&ScanMatcherNode::scanCallback, this, std::placeholders::_1));

  // ── FGO pose subscription (for NDT initial guess) ─────────────────────────
  sub_fgo_pose_ = create_subscription<nav_msgs::msg::Odometry>(
    "/fgo/odometry", rclcpp::QoS(10),
    std::bind(&ScanMatcherNode::fgoPoseCallback, this, std::placeholders::_1));

  // ── Publisher ─────────────────────────────────────────────────────────────
  pub_scan_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    scan_match_pose_topic_, rclcpp::QoS(10));

  RCLCPP_INFO(get_logger(),
    "[ScanMatcherNode] Started. type=%s lidar_frame=%s",
    scan_matcher_type_.c_str(), lidar_frame_.c_str());
}

// ═════════════════════════════════════════════════════════════════════════════
// Parameter helpers
// ═════════════════════════════════════════════════════════════════════════════

void ScanMatcherNode::declareParameters()
{
  declare_parameter("topics.lidar_topic",             "/scan");
  declare_parameter("topics.scan_match_pose_topic",   "/scan_match_pose");
  declare_parameter("topics.map_topic",               "/map");

  declare_parameter("frames.lidar_frame", "base_footprint");

  declare_parameter("scan_matcher.type",                    "NDT");
  declare_parameter("scan_matcher.max_iterations",          50);
  declare_parameter("scan_matcher.max_correspondence_dist", 1.0);
  declare_parameter("scan_matcher.transformation_epsilon",  1.0e-6);
  declare_parameter("scan_matcher.map_z_height",            0.0);
  declare_parameter("scan_matcher.ndt_resolution",          1.0);

  declare_parameter("noise.lidar.x",     0.1);
  declare_parameter("noise.lidar.y",     0.1);
  declare_parameter("noise.lidar.z",     999.0);
  declare_parameter("noise.lidar.roll",  999.0);
  declare_parameter("noise.lidar.pitch", 999.0);
  declare_parameter("noise.lidar.yaw",   0.1);
  declare_parameter("noise.lidar.icp_fitness_score_threshold", 0.5);
  declare_parameter("noise.lidar.fitness_noise_scale",         5.0);
  declare_parameter("noise.lidar.map_voxel_leaf_size",         0.1);
}

void ScanMatcherNode::loadParameters()
{
  lidar_topic_            = get_parameter("topics.lidar_topic").as_string();
  scan_match_pose_topic_  = get_parameter("topics.scan_match_pose_topic").as_string();
  map_topic_              = get_parameter("topics.map_topic").as_string();

  lidar_frame_ = get_parameter("frames.lidar_frame").as_string();

  scan_matcher_type_        = get_parameter("scan_matcher.type").as_string();
  max_iterations_           = get_parameter("scan_matcher.max_iterations").as_int();
  max_correspondence_dist_  = get_parameter("scan_matcher.max_correspondence_dist").as_double();
  transformation_epsilon_   = get_parameter("scan_matcher.transformation_epsilon").as_double();
  map_z_height_             = get_parameter("scan_matcher.map_z_height").as_double();
  ndt_resolution_           = get_parameter("scan_matcher.ndt_resolution").as_double();

  noise_lidar_x_     = get_parameter("noise.lidar.x").as_double();
  noise_lidar_y_     = get_parameter("noise.lidar.y").as_double();
  noise_lidar_z_     = get_parameter("noise.lidar.z").as_double();
  noise_lidar_roll_  = get_parameter("noise.lidar.roll").as_double();
  noise_lidar_pitch_ = get_parameter("noise.lidar.pitch").as_double();
  noise_lidar_yaw_   = get_parameter("noise.lidar.yaw").as_double();

  icp_fitness_score_threshold_ =
    get_parameter("noise.lidar.icp_fitness_score_threshold").as_double();
  fitness_noise_scale_ =
    get_parameter("noise.lidar.fitness_noise_scale").as_double();
  map_voxel_leaf_size_ =
    get_parameter("noise.lidar.map_voxel_leaf_size").as_double();
}

// ═════════════════════════════════════════════════════════════════════════════
// Map callback
// ═════════════════════════════════════════════════════════════════════════════

void ScanMatcherNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "[ScanMatcherNode] Map received (%zu × %zu). Converting to PCL...",
    msg->info.width, msg->info.height);

  map_cloud_ = occupancyGridToCloud(*msg);

  // Downsample map cloud to reduce NDT/ICP computation cost.
  {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(map_cloud_);
    vg.setLeafSize(
      static_cast<float>(map_voxel_leaf_size_),
      static_cast<float>(map_voxel_leaf_size_),
      static_cast<float>(map_voxel_leaf_size_));
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(
      new pcl::PointCloud<pcl::PointXYZ>);
    vg.filter(*filtered);
    map_cloud_ = filtered;
  }

  map_received_ = true;

  RCLCPP_INFO(get_logger(),
    "[ScanMatcherNode] Map cloud built: %zu points (after voxel filter, leaf=%.2fm).",
    map_cloud_->size(), map_voxel_leaf_size_);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
ScanMatcherNode::occupancyGridToCloud(const nav_msgs::msg::OccupancyGrid & grid)
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->header.frame_id = grid.header.frame_id;

  const double res    = grid.info.resolution;
  const double ox     = grid.info.origin.position.x;
  const double oy     = grid.info.origin.position.y;
  const unsigned int W = grid.info.width;
  const unsigned int H = grid.info.height;

  cloud->reserve(W * H / 4);  // rough upper bound

  for (unsigned int row = 0; row < H; ++row) {
    for (unsigned int col = 0; col < W; ++col) {
      const int cell_val = grid.data[row * W + col];
      if (cell_val == 100) {  // occupied
        pcl::PointXYZ pt;
        pt.x = ox + (static_cast<double>(col) + 0.5) * res;
        pt.y = oy + (static_cast<double>(row) + 0.5) * res;
        pt.z = static_cast<float>(map_z_height_);
        cloud->push_back(pt);
      }
    }
  }
  cloud->width    = static_cast<uint32_t>(cloud->size());
  cloud->height   = 1;
  cloud->is_dense = true;
  return cloud;
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
      lidar_frame_,   // target frame
      *msg,
      pc2_msg,
      *tf_buffer_);
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

  // ── Build initial guess from latest FGO pose ──────────────────────────────
  Eigen::Matrix4f initial_guess = buildInitialGuess();

  // ── Run matching ──────────────────────────────────────────────────────────
  Eigen::Matrix4f result_transform = Eigen::Matrix4f::Identity();
  double fitness_score = 0.0;

  if (scan_matcher_type_ == "NDT") {
    fitness_score = runNdt(source_cloud, initial_guess, result_transform);
  } else {
    fitness_score = runIcp(source_cloud, initial_guess, result_transform);
  }

  // ── Fitness gate: discard poor matches before publishing ──────────────────
  if (fitness_score > icp_fitness_score_threshold_) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
      "[ScanMatcherNode] Scan match discarded: fitness=%.3f > threshold=%.3f",
      fitness_score, icp_fitness_score_threshold_);
    return;
  }

  // Adaptive noise: sigma *= (1 + fitness_noise_scale * fitness_score)
  const double scan_noise_scale = 1.0 + fitness_noise_scale_ * fitness_score;

  // ── Enforce 2D constraints ────────────────────────────────────────────────
  result_transform = enforce2D(result_transform);

  // ── Extract pose from transform ───────────────────────────────────────────
  Eigen::Affine3f affine(result_transform);
  const float tx  = result_transform(0, 3);
  const float ty  = result_transform(1, 3);
  const float tz  = 0.0f;  // enforced to 0

  // Extract yaw only (roll, pitch are 0 after enforce2D)
  Eigen::Matrix3f rot = result_transform.block<3, 3>(0, 0);
  const float yaw = std::atan2(rot(1, 0), rot(0, 0));
  const float roll  = 0.0f;
  const float pitch = 0.0f;

  // Convert to quaternion
  Eigen::Quaternionf q =
    Eigen::AngleAxisf(roll,  Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(yaw,   Eigen::Vector3f::UnitZ());
  q.normalize();

  // ── Build output message ────────────────────────────────────────────────
  geometry_msgs::msg::PoseWithCovarianceStamped out;
  out.header.stamp    = msg->header.stamp;
  out.header.frame_id = "map";

  out.pose.pose.position.x = static_cast<double>(tx);
  out.pose.pose.position.y = static_cast<double>(ty);
  out.pose.pose.position.z = static_cast<double>(tz);
  out.pose.pose.orientation.x = static_cast<double>(q.x());
  out.pose.pose.orientation.y = static_cast<double>(q.y());
  out.pose.pose.orientation.z = static_cast<double>(q.z());
  out.pose.pose.orientation.w = static_cast<double>(q.w());

  // Proper diagonal covariance (σ² terms). Adaptive scaling already applied.
  // Order (row-major, Pose6DoF): [x, y, z, roll, pitch, yaw]
  // Diagonal indices: [0]=x·x, [7]=y·y, [14]=z·z, [21]=roll·roll, [28]=pitch·pitch, [35]=yaw·yaw
  out.pose.covariance.fill(0.0);
  out.pose.covariance[0]  = std::pow(noise_lidar_x_     * scan_noise_scale, 2.0);
  out.pose.covariance[7]  = std::pow(noise_lidar_y_     * scan_noise_scale, 2.0);
  out.pose.covariance[14] = std::pow(noise_lidar_z_,                        2.0);
  out.pose.covariance[21] = std::pow(noise_lidar_roll_,                     2.0);
  out.pose.covariance[28] = std::pow(noise_lidar_pitch_,                    2.0);
  out.pose.covariance[35] = std::pow(noise_lidar_yaw_   * scan_noise_scale, 2.0);

  pub_scan_pose_->publish(out);
}

// ═════════════════════════════════════════════════════════════════════════════
// FGO pose callback (for initial guess)
// ═════════════════════════════════════════════════════════════════════════════

void ScanMatcherNode::fgoPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_fgo_pose_ = msg->pose.pose;
  has_fgo_pose_ = true;
}

// ═════════════════════════════════════════════════════════════════════════════
// NDT matching
// ═════════════════════════════════════════════════════════════════════════════

double ScanMatcherNode::runNdt(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & source,
  const Eigen::Matrix4f & initial_guess,
  Eigen::Matrix4f & result_transform)
{
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setMaximumIterations(max_iterations_);
  ndt.setMaxCorrespondenceDistance(max_correspondence_dist_);
  ndt.setTransformationEpsilon(transformation_epsilon_);
  ndt.setResolution(static_cast<float>(ndt_resolution_));

  ndt.setInputTarget(map_cloud_);
  ndt.setInputSource(source);

  pcl::PointCloud<pcl::PointXYZ> aligned;
  ndt.align(aligned, initial_guess);

  if (ndt.hasConverged()) {
    result_transform = ndt.getFinalTransformation();
    return static_cast<double>(ndt.getFitnessScore());
  } else {
    RCLCPP_WARN(get_logger(), "[ScanMatcherNode] NDT did not converge.");
    result_transform = initial_guess;
    return 1e9;  // large fitness score → will be gated out
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// ICP matching (fallback)
// ═════════════════════════════════════════════════════════════════════════════

double ScanMatcherNode::runIcp(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & source,
  const Eigen::Matrix4f & initial_guess,
  Eigen::Matrix4f & result_transform)
{
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaximumIterations(max_iterations_);
  icp.setMaxCorrespondenceDistance(max_correspondence_dist_);
  icp.setTransformationEpsilon(transformation_epsilon_);

  icp.setInputTarget(map_cloud_);

  // Pre-transform source to initial guess position; ICP refines from there
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_init(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*source, *source_init, initial_guess);
  icp.setInputSource(source_init);

  pcl::PointCloud<pcl::PointXYZ> aligned;
  icp.align(aligned);

  if (icp.hasConverged()) {
    result_transform = icp.getFinalTransformation() * initial_guess;
    return static_cast<double>(icp.getFitnessScore());
  } else {
    RCLCPP_WARN(get_logger(), "[ScanMatcherNode] ICP did not converge.");
    result_transform = initial_guess;
    return 1e9;
  }
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
