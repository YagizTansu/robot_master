#include "robot_fgo_localization/fgo_localization_node.hpp"

#include <cmath>

namespace robot_fgo_localization
{

// ─── Constructor ──────────────────────────────────────────────────────────────
FGOLocalizationNode::FGOLocalizationNode(const rclcpp::NodeOptions & options)
: Node("fgo_localization_node", options)
{
  declareParameters();
  initISAM2();

  scan_matcher_ = std::make_shared<ScanMatcher>(get_logger());

  // ── TF ──────────────────────────────────────────────────────────────────────
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_      = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ── Subscribers ─────────────────────────────────────────────────────────────
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odometry", rclcpp::QoS(10),
    std::bind(&FGOLocalizationNode::odometryCallback, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu", rclcpp::QoS(50),
    std::bind(&FGOLocalizationNode::imuCallback, this, std::placeholders::_1));

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&FGOLocalizationNode::scanCallback, this, std::placeholders::_1));

  // Map is latched (transient_local) from map_server
  rclcpp::QoS map_qos(10);
  map_qos.transient_local();
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", map_qos,
    std::bind(&FGOLocalizationNode::mapCallback, this, std::placeholders::_1));

  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", rclcpp::QoS(10),
    std::bind(&FGOLocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  // ── Publishers ──────────────────────────────────────────────────────────────
  fgo_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/fgo_pose", rclcpp::QoS(10));

  RCLCPP_INFO(get_logger(), "[FGO] Node initialized. Waiting for map and initial pose...");

  // Auto-init: 5 saniye içinde ne map ne initialpose gelmezse (0,0,0)'dan başla.
  // Bu timer mapCallback veya initialPoseCallback tetiklenince iptal edilir.
  auto_init_timer_ = create_wall_timer(
    std::chrono::seconds(5),
    [this]() {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!graph_initialized_) {
        RCLCPP_WARN(get_logger(),
          "[FGO] No map / initial pose received in 5s — auto-starting at origin (0,0,0)");
        addPriorFactor(gtsam::Pose2(0.0, 0.0, 0.0));
        publishTFs(gtsam::Pose2(0.0, 0.0, 0.0), now());
      }
      auto_init_timer_->cancel();
    });
}

// ─── Parameter Declaration ────────────────────────────────────────────────────
void FGOLocalizationNode::declareParameters()
{
  map_frame_   = declare_parameter<std::string>("map_frame",  "map");
  odom_frame_  = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_  = declare_parameter<std::string>("base_frame", "base_footprint");

  odom_noise_x_   = declare_parameter<double>("odom_noise_x",   0.05);
  odom_noise_y_   = declare_parameter<double>("odom_noise_y",   0.05);
  odom_noise_yaw_ = declare_parameter<double>("odom_noise_yaw", 0.02);

  imu_noise_yaw_  = declare_parameter<double>("imu_noise_yaw",  0.01);

  scan_noise_x_       = declare_parameter<double>("scan_noise_x",       0.10);
  scan_noise_y_       = declare_parameter<double>("scan_noise_y",       0.10);
  scan_noise_yaw_     = declare_parameter<double>("scan_noise_yaw",     0.05);
  scan_min_fitness_   = declare_parameter<double>("scan_min_fitness",   0.30);  // ICP: lower = better

  isam2_relinearize_threshold_ = declare_parameter<double>("isam2_relinearize_threshold", 0.1);
  isam2_relinearize_skip_      = declare_parameter<int>("isam2_relinearize_skip", 10);

  initial_cov_x_   = declare_parameter<double>("initial_cov_x",   0.1);
  initial_cov_y_   = declare_parameter<double>("initial_cov_y",   0.1);
  initial_cov_yaw_ = declare_parameter<double>("initial_cov_yaw", 0.05);
}

// ─── ISAM2 Setup ─────────────────────────────────────────────────────────────
void FGOLocalizationNode::initISAM2()
{
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = isam2_relinearize_threshold_;
  params.relinearizeSkip      = isam2_relinearize_skip_;
  params.enableRelinearization = true;
  params.evaluateNonlinearError = false;
  params.factorization = gtsam::ISAM2Params::CHOLESKY;
  isam_ = gtsam::ISAM2(params);

  RCLCPP_INFO(get_logger(), "[FGO] iSAM2 initialized (threshold=%.2f, skip=%d)",
    isam2_relinearize_threshold_, isam2_relinearize_skip_);
}

// ─── Prior Factor ─────────────────────────────────────────────────────────────
void FGOLocalizationNode::addPriorFactor(const gtsam::Pose2 & initial_pose)
{
  // Reset graph state
  new_factors_.resize(0);
  new_values_.clear();
  pose_key_ = 0;
  graph_initialized_ = false;
  last_odom_pose_.reset();
  imu_yaw_pending_ = false;
  scan_factor_pending_ = false;

  // Re-initialize iSAM2 with fresh graph
  initISAM2();

  auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
    gtsam::Vector3(initial_cov_x_, initial_cov_y_, initial_cov_yaw_));

  new_factors_.add(gtsam::PriorFactor<gtsam::Pose2>(X(0), initial_pose, prior_noise));
  new_values_.insert(X(0), initial_pose);

  isam_.update(new_factors_, new_values_);
  current_estimate_ = isam_.calculateBestEstimate();
  new_factors_.resize(0);
  new_values_.clear();

  pose_key_ = 0;
  graph_initialized_ = true;

  RCLCPP_INFO(get_logger(),
    "[FGO] Graph initialized. Prior at x=%.3f y=%.3f yaw=%.3f",
    initial_pose.x(), initial_pose.y(), initial_pose.theta());
}

// ─── Callbacks ────────────────────────────────────────────────────────────────

void FGOLocalizationNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "[FGO] Map received (%ux%u @ %.3fm/cell)",
    msg->info.width, msg->info.height, msg->info.resolution);
  scan_matcher_->setMap(*msg);

  // Harita geldi — auto-init timer'a gerek yok, iptal et
  if (auto_init_timer_) {
    auto_init_timer_->cancel();
  }

  // initialpose gelmediıyse direkt origin'den başla
  if (!graph_initialized_) {
    RCLCPP_INFO(get_logger(), "[FGO] No initial pose yet — starting at map origin (0,0,0)");
    addPriorFactor(gtsam::Pose2(0.0, 0.0, 0.0));
    publishTFs(gtsam::Pose2(0.0, 0.0, 0.0), now());
  }
}

void FGOLocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  // initialpose geldi — auto-init timer'a gerek yok
  if (auto_init_timer_) {
    auto_init_timer_->cancel();
  }

  const double yaw = quaternionToYaw(msg->pose.pose.orientation);
  const gtsam::Pose2 new_pose(
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    yaw);

  RCLCPP_INFO(get_logger(), "[FGO] Initial pose received: x=%.3f y=%.3f yaw=%.3f",
    new_pose.x(), new_pose.y(), new_pose.theta());

  addPriorFactor(new_pose);

  // Publish immediately so map→odom TF is available
  publishTFs(new_pose, now());
  publishFGOPose(new_pose, now());
}

void FGOLocalizationNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_imu_yaw_ = quaternionToYaw(msg->orientation);
  imu_yaw_pending_ = true;
}

void FGOLocalizationNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!graph_initialized_ || !scan_matcher_->hasMap()) {
    return;
  }

  // Get current pose estimate as initial guess for ICP
  gtsam::Pose2 guess(0, 0, 0);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (current_estimate_.exists(X(pose_key_))) {
      guess = current_estimate_.at<gtsam::Pose2>(X(pose_key_));
    }
  }

  const auto result = scan_matcher_->match(
    *msg, guess.x(), guess.y(), guess.theta(), scan_min_fitness_);

  if (result.success) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    pending_scan_pose_ = gtsam::Pose2(result.x, result.y, result.yaw);
    scan_factor_pending_ = true;
  }
}

void FGOLocalizationNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!graph_initialized_) {
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);

  const gtsam::Pose2 current_odom = odomToPose2(*msg);

  // ── First odometry: just store, nothing to diff against ──────────────────
  if (!last_odom_pose_.has_value()) {
    last_odom_pose_ = current_odom;
    return;
  }

  // ── Compute pose delta in body frame ─────────────────────────────────────
  // Δpose = last_odom⁻¹ · current_odom  (relative motion since last step)
  const gtsam::Pose2 delta = last_odom_pose_->between(current_odom);
  last_odom_pose_ = current_odom;

  // Skip near-zero motion (reduces drift during standstill)
  if (std::abs(delta.x())     < 1e-4 &&
      std::abs(delta.y())     < 1e-4 &&
      std::abs(delta.theta()) < 1e-4)
  {
    return;
  }

  // ── Advance to next pose key ───────────────────────────────────────────────
  const uint64_t prev_key    = pose_key_;
  const uint64_t current_key = pose_key_ + 1;

  // Odometry between factor
  auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(
    gtsam::Vector3(odom_noise_x_, odom_noise_y_, odom_noise_yaw_));
  new_factors_.add(
    gtsam::BetweenFactor<gtsam::Pose2>(X(prev_key), X(current_key), delta, odom_noise));

  // Initial value for new key: dead-reckoning prediction
  const gtsam::Pose2 predicted = current_estimate_.at<gtsam::Pose2>(X(prev_key)).compose(delta);
  new_values_.insert(X(current_key), predicted);

  // ── IMU yaw factor ────────────────────────────────────────────────────────
  if (imu_yaw_pending_ && latest_imu_yaw_.has_value()) {
    // We model the IMU yaw as a unary factor on the current key.
    // Use a custom PriorFactor on orientation (yaw only, x/y left unconstrained
    // via large sigma). GTSAM's PriorFactor<Pose2> constrains all 3 DOF —
    // we set large sigmas on x/y and tight sigma on yaw.
    auto imu_noise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector3(1000.0, 1000.0, imu_noise_yaw_));  // x/y unconstrained
    const gtsam::Pose2 imu_pose(predicted.x(), predicted.y(), *latest_imu_yaw_);
    new_factors_.add(
      gtsam::PriorFactor<gtsam::Pose2>(X(current_key), imu_pose, imu_noise));
    imu_yaw_pending_ = false;
  }

  // ── Scan match factor ─────────────────────────────────────────────────────
  if (scan_factor_pending_) {
    auto scan_noise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector3(scan_noise_x_, scan_noise_y_, scan_noise_yaw_));
    new_factors_.add(
      gtsam::PriorFactor<gtsam::Pose2>(X(current_key), pending_scan_pose_, scan_noise));
    scan_factor_pending_ = false;
  }

  pose_key_ = current_key;

  // ── Update iSAM2 ──────────────────────────────────────────────────────────
  try {
    isam_.update(new_factors_, new_values_);
    // Extra update pass to improve linearization
    isam_.update();
    current_estimate_ = isam_.calculateBestEstimate();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "[FGO] iSAM2 update failed: %s", e.what());
    new_factors_.resize(0);
    new_values_.clear();
    return;
  }

  new_factors_.resize(0);
  new_values_.clear();

  // ── Publish ───────────────────────────────────────────────────────────────
  const gtsam::Pose2 map_pose = current_estimate_.at<gtsam::Pose2>(X(pose_key_));
  const rclcpp::Time stamp    = msg->header.stamp;
  publishTFs(map_pose, stamp);
  publishFGOPose(map_pose, stamp);
}

// ─── TF Publishing ────────────────────────────────────────────────────────────
void FGOLocalizationNode::publishTFs(const gtsam::Pose2 & map_pose, const rclcpp::Time & stamp)
{
  // FGO gives us the robot pose in the map frame: map_T_base
  // We need to publish:
  //   map → odom  (map_T_odom)
  //   odom → base_footprint  (odom_T_base)   kept at identity — FGO is our odom
  //
  // Since FGO directly computes map_T_base, we set odom_T_base = identity
  // and map_T_odom = map_T_base.

  std::vector<geometry_msgs::msg::TransformStamped> transforms;

  // map → odom  =  map_T_base pose
  transforms.push_back(makeTransform(
    map_frame_, odom_frame_,
    map_pose.x(), map_pose.y(), map_pose.theta(), stamp));

  // odom → base_footprint  =  identity (FGO subsumes odom)
  transforms.push_back(makeTransform(
    odom_frame_, base_frame_,
    0.0, 0.0, 0.0, stamp));

  tf_broadcaster_->sendTransform(transforms);
}

void FGOLocalizationNode::publishFGOPose(
  const gtsam::Pose2 & pose, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = map_frame_;
  msg.header.stamp    = stamp;

  msg.pose.pose.position.x = pose.x();
  msg.pose.pose.position.y = pose.y();
  msg.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose.theta());
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();

  // Covariance from iSAM2 marginals (diagonal approximation)
  // Full marginals are expensive — we use a fixed diagonal based on noise params
  msg.pose.covariance[0]  = scan_noise_x_ * scan_noise_x_;   // xx
  msg.pose.covariance[7]  = scan_noise_y_ * scan_noise_y_;   // yy
  msg.pose.covariance[35] = scan_noise_yaw_ * scan_noise_yaw_; // yawyaw

  fgo_pose_pub_->publish(msg);
}

// ─── Helpers ──────────────────────────────────────────────────────────────────
gtsam::Pose2 FGOLocalizationNode::odomToPose2(const nav_msgs::msg::Odometry & odom)
{
  const double yaw = quaternionToYaw(odom.pose.pose.orientation);
  return gtsam::Pose2(
    odom.pose.pose.position.x,
    odom.pose.pose.position.y,
    yaw);
}

double FGOLocalizationNode::quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
{
  // yaw = atan2(2*(w*z + x*y), 1 - 2*(y² + z²))
  const double sinr = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosr = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(sinr, cosr);
}

geometry_msgs::msg::TransformStamped FGOLocalizationNode::makeTransform(
  const std::string & parent, const std::string & child,
  double x, double y, double yaw, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp    = stamp;
  tf.header.frame_id = parent;
  tf.child_frame_id  = child;

  tf.transform.translation.x = x;
  tf.transform.translation.y = y;
  tf.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();

  return tf;
}

}  // namespace robot_fgo_localization

// ─── Component Registration ───────────────────────────────────────────────────
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(robot_fgo_localization::FGOLocalizationNode)
