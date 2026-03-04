#include "robot_fgo_localization/fgo_localization_node.hpp"

#include <cmath>
#include <gtsam/nonlinear/Marginals.h>

namespace robot_fgo_localization
{

// ─── Constructor ──────────────────────────────────────────────────────────────
FGOLocalizationNode::FGOLocalizationNode(const rclcpp::NodeOptions & options)
: Node("fgo_localization_node", options)
{
  declareParameters();
  initISAM2();

  scan_matcher_ = std::make_shared<ScanMatcher>(get_logger());
  scan_matcher_->configure(
    lidar_cfg_.icp_max_correspondence_dist,
    lidar_cfg_.icp_max_iterations,
    lidar_cfg_.icp_voxel_leaf_size);

  // ── TF ──────────────────────────────────────────────────────────────────────
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_      = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ── Odometry (zorunlu — graph backbone) ─────────────────────────────────────
  if (!odom_cfg_.enabled) {
    RCLCPP_WARN(get_logger(),
      "[FGO] sensors.odometry.enabled=false! "
      "Odometry graphın backbone'udur; diğer sensörler çalışmaz.");
  }
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_cfg_.topic, rclcpp::QoS(10),
    std::bind(&FGOLocalizationNode::odometryCallback, this, std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "[FGO] Odometry → %s  (enabled=%s)",
    odom_cfg_.topic.c_str(), odom_cfg_.enabled ? "true" : "false");

  // ── IMU (opsiyonel — yaw kısıtı) ────────────────────────────────────────────
  if (imu_cfg_.enabled) {
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_cfg_.topic, rclcpp::QoS(50),
      std::bind(&FGOLocalizationNode::imuCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "[FGO] IMU     → %s  (enabled=true, noise_yaw=%.4f)",
      imu_cfg_.topic.c_str(), imu_cfg_.noise_yaw);
  } else {
    RCLCPP_INFO(get_logger(), "[FGO] IMU     → DEVRE DIŞI (sensors.imu.enabled=false)");
  }

  // ── Lidar / Scan matcher (opsiyonel — ICP kısıtı) ───────────────────────────
  if (lidar_cfg_.enabled) {
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      lidar_cfg_.topic, rclcpp::SensorDataQoS(),
      std::bind(&FGOLocalizationNode::scanCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "[FGO] Lidar   → %s  (enabled=true, min_fitness=%.3f)",
      lidar_cfg_.topic.c_str(), lidar_cfg_.min_fitness);
  } else {
    RCLCPP_INFO(get_logger(), "[FGO] Lidar   → DEVRE DIŞI (sensors.lidar.enabled=false)");
  }

  // ── GPS (opsiyonel — NavSatFix → yerel XY) ──────────────────────────────
  if (gps_cfg_.enabled) {
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_cfg_.topic, rclcpp::SensorDataQoS(),
      std::bind(&FGOLocalizationNode::gpsCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(),
      "[FGO] GPS     → %s  (enabled=true, datum=%.6f,%.6f, noise=%.2fm)",
      gps_cfg_.topic.c_str(), gps_cfg_.datum_lat, gps_cfg_.datum_lon, gps_cfg_.noise_x);
  } else {
    RCLCPP_INFO(get_logger(), "[FGO] GPS     → DEVRE DISI (sensors.gps.enabled=false)");
  }

  // ── Map (latched) ────────────────────────────────────────────────────────────
  rclcpp::QoS map_qos(10);
  map_qos.transient_local();
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", map_qos,
    std::bind(&FGOLocalizationNode::mapCallback, this, std::placeholders::_1));

  // ── Initial Pose ─────────────────────────────────────────────────────────────
  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", rclcpp::QoS(10),
    std::bind(&FGOLocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  // ── Publishers ──────────────────────────────────────────────────────────────
  fgo_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/fgo_pose", rclcpp::QoS(10));

  RCLCPP_INFO(get_logger(), "[FGO] Node initialized. Waiting for map and initial pose...");

  // ── Auto-init timer (5 sn) ───────────────────────────────────────────────────
  auto_init_timer_ = create_wall_timer(
    std::chrono::seconds(5),
    [this]() {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!graph_initialized_) {
        RCLCPP_WARN(get_logger(),
          "[FGO] No map / initial pose received in 5s — auto-starting at origin (0,0,0)");
        addPriorFactor(gtsam::Pose2(0.0, 0.0, 0.0));
        publishTFs(gtsam::Pose2(0.0, 0.0, 0.0), last_raw_odom_pose_, now());
      }
      auto_init_timer_->cancel();
    });

  // ── TF yayın timer (50 Hz) ───────────────────────────────────────────────────
  tf_publish_timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    [this]() {
      if (!graph_initialized_) return;
      std::lock_guard<std::mutex> lock(state_mutex_);
      publishTFs(last_map_pose_, last_raw_odom_pose_, now());
    });
}

// ─── Parameter Declaration ────────────────────────────────────────────────────
void FGOLocalizationNode::declareParameters()
{
  // Frame IDs
  map_frame_  = declare_parameter<std::string>("map_frame",  "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");

  // iSAM2
  isam2_relinearize_threshold_ = declare_parameter<double>("isam2_relinearize_threshold", 0.1);
  isam2_relinearize_skip_      = declare_parameter<int>("isam2_relinearize_skip", 10);

  // Initial covariance
  initial_cov_x_   = declare_parameter<double>("initial_cov_x",   0.5);
  initial_cov_y_   = declare_parameter<double>("initial_cov_y",   0.5);
  initial_cov_yaw_ = declare_parameter<double>("initial_cov_yaw", 0.2);

  // ── Odometry sensor ──────────────────────────────────────────────────────────
  odom_cfg_.enabled   = declare_parameter<bool>       ("sensors.odometry.enabled",   true);
  odom_cfg_.topic     = declare_parameter<std::string>("sensors.odometry.topic",     "/odometry");
  odom_cfg_.noise_x   = declare_parameter<double>     ("sensors.odometry.noise_x",   0.05);
  odom_cfg_.noise_y   = declare_parameter<double>     ("sensors.odometry.noise_y",   0.05);
  odom_cfg_.noise_yaw = declare_parameter<double>     ("sensors.odometry.noise_yaw", 0.02);

  // ── IMU sensor ───────────────────────────────────────────────────────────────
  imu_cfg_.enabled   = declare_parameter<bool>       ("sensors.imu.enabled",   true);
  imu_cfg_.topic     = declare_parameter<std::string>("sensors.imu.topic",     "/imu");
  imu_cfg_.noise_yaw = declare_parameter<double>     ("sensors.imu.noise_yaw", 0.01);

  // ── Lidar sensor ─────────────────────────────────────────────────────────────
  lidar_cfg_.enabled      = declare_parameter<bool>       ("sensors.lidar.enabled",      true);
  lidar_cfg_.topic        = declare_parameter<std::string>("sensors.lidar.topic",        "/scan");
  lidar_cfg_.noise_x      = declare_parameter<double>     ("sensors.lidar.noise_x",      0.10);
  lidar_cfg_.noise_y      = declare_parameter<double>     ("sensors.lidar.noise_y",      0.10);
  lidar_cfg_.noise_yaw    = declare_parameter<double>     ("sensors.lidar.noise_yaw",    0.05);
  lidar_cfg_.min_fitness  = declare_parameter<double>     ("sensors.lidar.min_fitness",  0.30);
  lidar_cfg_.icp_max_correspondence_dist =
    declare_parameter<double>("sensors.lidar.icp_max_correspondence_dist", 0.5);
  lidar_cfg_.icp_max_iterations =
    declare_parameter<int>   ("sensors.lidar.icp_max_iterations",          50);
  lidar_cfg_.icp_voxel_leaf_size =
    declare_parameter<double>("sensors.lidar.icp_voxel_leaf_size",         0.05);

  // ── GPS sensor ────────────────────────────────────────────────────
  gps_cfg_.enabled    = declare_parameter<bool>       ("sensors.gps.enabled",    false);
  gps_cfg_.topic      = declare_parameter<std::string>("sensors.gps.topic",      "/gps/fix");
  gps_cfg_.datum_lat  = declare_parameter<double>     ("sensors.gps.datum_lat",  0.0);
  gps_cfg_.datum_lon  = declare_parameter<double>     ("sensors.gps.datum_lon",  0.0);
  gps_cfg_.noise_x    = declare_parameter<double>     ("sensors.gps.noise_x",    2.0);
  gps_cfg_.noise_y    = declare_parameter<double>     ("sensors.gps.noise_y",    2.0);
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
  new_factors_.resize(0);
  new_values_.clear();
  pose_key_ = 0;
  graph_initialized_ = false;
  last_odom_pose_.reset();
  imu_yaw_pending_ = false;
  scan_factor_pending_ = false;
  gps_factor_pending_ = false;

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

  // FIX 3: state_mutex_ ile korunuyoruz — scan callback aynı anda
  // scan_matcher_->match() çağırabilir; setMap() sırasında map_cloud_
  // yeniden inşa edildiğinden data race oluşurdu.
  std::lock_guard<std::mutex> lock(state_mutex_);

  scan_matcher_->setMap(*msg);

  if (auto_init_timer_) { auto_init_timer_->cancel(); }

  if (!graph_initialized_) {
    RCLCPP_INFO(get_logger(), "[FGO] No initial pose yet — starting at map origin (0,0,0)");
    addPriorFactor(gtsam::Pose2(0.0, 0.0, 0.0));
    publishTFs(gtsam::Pose2(0.0, 0.0, 0.0), last_raw_odom_pose_, now());
  }
}

void FGOLocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (auto_init_timer_) { auto_init_timer_->cancel(); }

  const double yaw = quaternionToYaw(msg->pose.pose.orientation);
  const gtsam::Pose2 new_pose(
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    yaw);

  RCLCPP_INFO(get_logger(), "[FGO] Initial pose received: x=%.3f y=%.3f yaw=%.3f",
    new_pose.x(), new_pose.y(), new_pose.theta());

  addPriorFactor(new_pose);
  last_map_pose_ = new_pose;

  publishTFs(new_pose, last_raw_odom_pose_, now());
  publishFGOPose(new_pose, now());
}

void FGOLocalizationNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // Bu callback sadece imu_cfg_.enabled == true ise oluşturuldu
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_imu_yaw_ = quaternionToYaw(msg->orientation);
  imu_yaw_pending_ = true;
}

void FGOLocalizationNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // Bu callback sadece lidar_cfg_.enabled == true ise oluşturuldu
  if (!graph_initialized_ || !scan_matcher_->hasMap()) {
    return;
  }

  // ── Lidar frame → base_footprint dönüşümü (otomatik) ────────────────────────
  // Scan zaten base_footprint'te yayınlanıyorsa (örn. dual lidar merger) identity kullanılır.
  // Tek lidar senaryosunda frame_id farklıysa TF'ten otomatik alınır.
  Eigen::Matrix4f scan_to_base = Eigen::Matrix4f::Identity();
  if (msg->header.frame_id != base_frame_) {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        base_frame_, msg->header.frame_id,
        msg->header.stamp,
        rclcpp::Duration::from_seconds(0.05));

      const auto & t = tf.transform.translation;
      const auto & q = tf.transform.rotation;
      // Quaternion → rotation matrix (sadece Z dönüşü için)
      const double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      const float cy = static_cast<float>(std::cos(yaw));
      const float sy = static_cast<float>(std::sin(yaw));

      scan_to_base(0, 0) = cy;  scan_to_base(0, 1) = -sy;
      scan_to_base(1, 0) = sy;  scan_to_base(1, 1) =  cy;
      scan_to_base(0, 3) = static_cast<float>(t.x);
      scan_to_base(1, 3) = static_cast<float>(t.y);

      RCLCPP_DEBUG(get_logger(),
        "[FGO] Scan frame '%s' → '%s': tx=%.3f ty=%.3f yaw=%.3f",
        msg->header.frame_id.c_str(), base_frame_.c_str(), t.x, t.y, yaw);

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "[FGO] TF lookup '%s'→'%s' failed: %s — skipping scan",
        msg->header.frame_id.c_str(), base_frame_.c_str(), ex.what());
      return;
    }
  }

  // FIX 4: pose_key_ snapshot — scan ile odom arasındaki race condition'ı önler.
  // ICP hesabı mutex dışında (blocking) yapılır; bu süre içinde odometryCallback
  // pose_key_'i artırabilir. Snapshot alınarak scan sonucu doğru node'a bağlanır.
  gtsam::Pose2 guess(0, 0, 0);
  uint64_t scan_target_key = 0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    scan_target_key = pose_key_;  // snapshot
    if (current_estimate_.exists(X(scan_target_key))) {
      guess = current_estimate_.at<gtsam::Pose2>(X(scan_target_key));
    }
  }

  // ICP hesabı — mutex dışında, blocking olabilir
  const auto result = scan_matcher_->match(
    *msg, guess.x(), guess.y(), guess.theta(), lidar_cfg_.min_fitness, scan_to_base);

  if (result.success) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    pending_scan_pose_    = gtsam::Pose2(result.x, result.y, result.yaw);
    pending_scan_key_     = scan_target_key;  // hangi node'a bağlanacağı kayıt altına alındı
    scan_factor_pending_  = true;
  }
}


void FGOLocalizationNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  // Bu callback sadece gps_cfg_.enabled == true ise oluşturuldu

  // Fix kalitesini kontrol et (STATUS_NO_FIX = -1, STATUS_FIX = 0, SBAS/GBAS daha iyi)
  if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "[FGO] GPS: No fix (status=%d) — skipping", msg->status.status);
    return;
  }

  // ── Equirectangular projection: lat/lon → yerel metre (X, Y) ───────────────
  // Küçük alanlar için yeterli hassasiyet. Büyük alanlar için UTM kullanılabilir.
  //
  //   x = (lon - datum_lon) * cos(datum_lat) * R_earth
  //   y = (lat - datum_lat) * R_earth
  //
  constexpr double R_EARTH = 6378137.0;  // metre (WGS-84 yarıçapı)
  constexpr double DEG2RAD = M_PI / 180.0;

  const double dlat = (msg->latitude  - gps_cfg_.datum_lat) * DEG2RAD;
  const double dlon = (msg->longitude - gps_cfg_.datum_lon) * DEG2RAD;
  const double cos_lat = std::cos(gps_cfg_.datum_lat * DEG2RAD);

  const double local_x = dlon * cos_lat * R_EARTH;
  const double local_y = dlat * R_EARTH;

  RCLCPP_DEBUG(get_logger(),
    "[FGO] GPS fix: lat=%.7f lon=%.7f → x=%.2f y=%.2f (cov=%.2f)",
    msg->latitude, msg->longitude, local_x, local_y,
    msg->position_covariance[0]);

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // Pose2'nin yaw'ı graph tarafından belirlenir; GPS sadece x/y kısıtı sağlar
    pending_gps_pose_ = gtsam::Pose2(local_x, local_y, 0.0);
    gps_factor_pending_ = true;
  }
}

void FGOLocalizationNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const gtsam::Pose2 current_odom = odomToPose2(*msg);
  const rclcpp::Time stamp        = msg->header.stamp;

  std::lock_guard<std::mutex> lock(state_mutex_);

  last_raw_odom_pose_ = current_odom;

  if (!graph_initialized_) {
    last_odom_pose_ = current_odom;
    return;
  }

  if (!last_odom_pose_.has_value()) {
    last_odom_pose_ = current_odom;
    return;
  }

  const gtsam::Pose2 delta = last_odom_pose_->between(current_odom);
  last_odom_pose_ = current_odom;

  if (std::abs(delta.x())     < 1e-4 &&
      std::abs(delta.y())     < 1e-4 &&
      std::abs(delta.theta()) < 1e-4)
  {
    return;
  }

  const uint64_t prev_key    = pose_key_;
  const uint64_t current_key = pose_key_ + 1;

  // ── Odometry BetweenFactor ─────────────────────────────────────────────────
  auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(
    gtsam::Vector3(odom_cfg_.noise_x, odom_cfg_.noise_y, odom_cfg_.noise_yaw));
  new_factors_.add(
    gtsam::BetweenFactor<gtsam::Pose2>(X(prev_key), X(current_key), delta, odom_noise));

  const gtsam::Pose2 predicted = current_estimate_.at<gtsam::Pose2>(X(prev_key)).compose(delta);
  new_values_.insert(X(current_key), predicted);

  // ── IMU yaw factor (sadece imu_cfg_.enabled ise callback var) ───────────────
  // FIX 2: IMU sadece yaw ölçer — x/y bilgisi yoktur. x/y sigma'sı için odom
  // noise'unun %100 büyüğünü kullanıyoruz (yani IMU x/y'ye hiç katkı yapmaz
  // ama 1000.0 gibi uç değerler sayısal instabiliteye yol açabilir).
  // Robust yöntem: x/y sigması = odom sigması * büyük_çarpan (örn. 50x).
  if (imu_yaw_pending_ && latest_imu_yaw_.has_value()) {
    const double imu_xy_sigma = std::max(odom_cfg_.noise_x, odom_cfg_.noise_y) * 50.0;
    auto imu_noise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector3(imu_xy_sigma, imu_xy_sigma, imu_cfg_.noise_yaw));
    const gtsam::Pose2 imu_pose(predicted.x(), predicted.y(), *latest_imu_yaw_);
    new_factors_.add(
      gtsam::PriorFactor<gtsam::Pose2>(X(current_key), imu_pose, imu_noise));
    imu_yaw_pending_ = false;
  }

  // ── Scan factor (sadece lidar_cfg_.enabled ise callback var) ────────────────
  // FIX 4: Scan faktörü snapshot'taki key'e bağlanır (current_key değil).
  // Eğer scan arrival sırasında current_key > pending_scan_key_ ise
  // (yani odom callback araya girdi), scan faktörü kaçırılır — bir sonrak
  // scan zaten doğru key'i taşıyacak. Bu sayede yanlış node'a bağlama olmaz.
  if (scan_factor_pending_) {
    if (pending_scan_key_ == current_key ||
        (pending_scan_key_ < current_key && current_estimate_.exists(X(pending_scan_key_))))
    {
      const uint64_t target_key = (pending_scan_key_ <= current_key) ? pending_scan_key_ : current_key;
      auto scan_noise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(lidar_cfg_.noise_x, lidar_cfg_.noise_y, lidar_cfg_.noise_yaw));
      new_factors_.add(
        gtsam::PriorFactor<gtsam::Pose2>(X(target_key), pending_scan_pose_, scan_noise));
    } else {
      RCLCPP_DEBUG(get_logger(),
        "[FGO] Scan factor dropped: snapshot key=%lu, current_key=%lu (stale)",
        pending_scan_key_, current_key);
    }
    scan_factor_pending_ = false;
  }

  // ── GPS factor (sadece gps_cfg_.enabled ise callback var) ────────────────────
  if (gps_factor_pending_) {
    // Sadece x/y kısıtı: yaw'a 1000.0 sigma vererek "önemseme" diyoruz
    auto gps_noise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector3(gps_cfg_.noise_x, gps_cfg_.noise_y, 1000.0));
    new_factors_.add(
      gtsam::PriorFactor<gtsam::Pose2>(X(current_key), pending_gps_pose_, gps_noise));
    gps_factor_pending_ = false;
  }

  pose_key_ = current_key;

  // ── iSAM2 güncelleme ────────────────────────────────────────────────────────
  try {
    isam_.update(new_factors_, new_values_);
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

  last_map_pose_ = current_estimate_.at<gtsam::Pose2>(X(pose_key_));

  publishTFs(last_map_pose_, last_raw_odom_pose_, stamp);
  publishFGOPose(last_map_pose_, stamp);
}

// ─── TF Publishing ────────────────────────────────────────────────────────────
void FGOLocalizationNode::publishTFs(
  const gtsam::Pose2 & map_pose,
  const gtsam::Pose2 & odom_pose,
  const rclcpp::Time & stamp)
{
  const gtsam::Pose2 map_T_odom = map_pose.compose(odom_pose.inverse());

  std::vector<geometry_msgs::msg::TransformStamped> transforms;

  transforms.push_back(makeTransform(
    map_frame_, odom_frame_,
    map_T_odom.x(), map_T_odom.y(), map_T_odom.theta(), stamp));

  transforms.push_back(makeTransform(
    odom_frame_, base_frame_,
    odom_pose.x(), odom_pose.y(), odom_pose.theta(), stamp));

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

  // FIX 1: iSAM2'den gerçek marginal covariance — Nav2 için doğru belirsizlik bilgisi.
  // Hardcoded lidar noise değerleri yerine graph'ın hesapladığı covariance kullanılır.
  // Marginals hesabı pahalıdır — sadece publish anında (odometryCallback hızında) yapılır.
  bool cov_ok = false;
  try {
    const gtsam::Marginals marginals(
      isam_.getFactorsUnsafe(), current_estimate_,
      gtsam::Marginals::Factorization::CHOLESKY);
    const gtsam::Matrix33 cov = marginals.marginalCovariance(X(pose_key_));
    // Pose2 covariance sırası: [x, y, theta]
    // nav_msgs covariance: 6×6 [x, y, z, roll, pitch, yaw] — düz sıra
    msg.pose.covariance[0]  = cov(0, 0);  // x-x
    msg.pose.covariance[1]  = cov(0, 1);  // x-y
    msg.pose.covariance[5]  = cov(0, 2);  // x-yaw
    msg.pose.covariance[6]  = cov(1, 0);  // y-x
    msg.pose.covariance[7]  = cov(1, 1);  // y-y
    msg.pose.covariance[11] = cov(1, 2);  // y-yaw
    msg.pose.covariance[30] = cov(2, 0);  // yaw-x
    msg.pose.covariance[31] = cov(2, 1);  // yaw-y
    msg.pose.covariance[35] = cov(2, 2);  // yaw-yaw
    cov_ok = true;
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "[FGO] Marginals failed (%s) — using noise model as fallback", e.what());
  }

  if (!cov_ok) {
    // Fallback: lidar noise sigma^2 — graph yeni başladıysa veya degenerate ise
    msg.pose.covariance[0]  = lidar_cfg_.noise_x   * lidar_cfg_.noise_x;
    msg.pose.covariance[7]  = lidar_cfg_.noise_y   * lidar_cfg_.noise_y;
    msg.pose.covariance[35] = lidar_cfg_.noise_yaw * lidar_cfg_.noise_yaw;
  }

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
