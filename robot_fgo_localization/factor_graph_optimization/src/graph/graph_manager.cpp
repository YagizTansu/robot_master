#include "factor_graph_optimization/graph/graph_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

#include <tf2/LinearMath/Quaternion.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "factor_graph_optimization/core/pose_conversion.hpp"
#include "factor_graph_optimization/core/noise_model.hpp"
#include "factor_graph_optimization/core/geometry_2d.hpp"

namespace factor_graph_optimization
{

// ─────────────────────────────────────────────────────────────────────────────
// Construction / re-initialisation
// ─────────────────────────────────────────────────────────────────────────────

GraphManager::GraphManager(const FgoConfig & cfg,
                           std::unique_ptr<ImuPreintegrator> imu_preint)
: cfg_(cfg), imu_preint_(std::move(imu_preint))
{
  last_consumed_odom_pose_  = makePoseFromCfg(cfg_);
  last_consumed_odom_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  initIsam2();
  initGraph();
}

void GraphManager::reinit(const FgoConfig & cfg)
{
  cfg_ = cfg;
  last_consumed_odom_pose_  = makePoseFromCfg(cfg_);
  last_consumed_odom_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  optimized_velocity_       = gtsam::Vector3::Zero();
  optimized_bias_           = gtsam::imuBias::ConstantBias();
  initIsam2();
  initGraph();
}

// ─────────────────────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────────────────────

void GraphManager::initIsam2()
{
  gtsam::ISAM2Params params;
  params.relinearizeThreshold   = cfg_.isam2_relinearize_threshold;
  params.relinearizeSkip        = cfg_.isam2_relinearize_skip;
  params.factorization          = (cfg_.isam2_factorization == "QR")
                                  ? gtsam::ISAM2Params::QR
                                  : gtsam::ISAM2Params::CHOLESKY;
  params.evaluateNonlinearError = false;
  isam2_ = std::make_unique<gtsam::ISAM2>(params);
}

void GraphManager::initGraph()
{
  key_ = 0;
  oldest_kept_key_ = 0;
  new_factors_.resize(0);
  new_values_.clear();

  // ── X(0): tight prior on initial pose ────────────────────────────────────
  gtsam::Pose3 init_pose(
    gtsam::Rot3::RzRyRx(cfg_.init_roll, cfg_.init_pitch, cfg_.init_yaw),
    gtsam::Point3(cfg_.init_x, cfg_.init_y, cfg_.init_z));

  auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector6() << cfg_.prior_pose_rot_sigma, cfg_.prior_pose_rot_sigma, cfg_.prior_pose_rot_sigma,
                         cfg_.prior_pose_pos_sigma, cfg_.prior_pose_pos_sigma, cfg_.prior_pose_pos_sigma
    ).finished());

  new_factors_.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), init_pose, prior_noise));
  new_values_.insert(X(0), init_pose);

  // ── V(0) and B(0): velocity and bias priors (IMU only) ───────────────────
  if (imu_preint_) {
    optimized_velocity_ = gtsam::Vector3::Zero();
    optimized_bias_     = gtsam::imuBias::ConstantBias();

    auto vel_noise = gtsam::noiseModel::Isotropic::Sigma(3, cfg_.prior_vel_sigma);
    new_factors_.add(
      gtsam::PriorFactor<gtsam::Vector3>(V(0), optimized_velocity_, vel_noise));
    new_values_.insert(V(0), optimized_velocity_);

    auto bias_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector6() << cfg_.prior_bias_accel_sigma, cfg_.prior_bias_accel_sigma, cfg_.prior_bias_accel_sigma,
                           cfg_.prior_bias_gyro_sigma,  cfg_.prior_bias_gyro_sigma,  cfg_.prior_bias_gyro_sigma
      ).finished());
    new_factors_.add(
      gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), optimized_bias_, bias_noise));
    new_values_.insert(B(0), optimized_bias_);
  }

  isam2_->update(new_factors_, new_values_);
  new_factors_.resize(0);
  new_values_.clear();

  optimized_pose_ = init_pose;
}

geometry_msgs::msg::Pose GraphManager::makePoseFromCfg(const FgoConfig & cfg)
{
  geometry_msgs::msg::Pose p;
  p.position.x = cfg.init_x;
  p.position.y = cfg.init_y;
  p.position.z = cfg.init_z;
  tf2::Quaternion q;
  q.setRPY(cfg.init_roll, cfg.init_pitch, cfg.init_yaw);
  q.normalize();
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

// ─────────────────────────────────────────────────────────────────────────────
// Main optimisation step
// ─────────────────────────────────────────────────────────────────────────────

bool GraphManager::step(
  const std::vector<OdomSample> & local_odom,
  std::vector<ImuSample>          local_imu,
  const std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> & local_scan,
  const std::vector<GpsSample> & local_gps,
  const rclcpp::Logger & logger)
{
  // ── Noise model for odometry ──────────────────────────────────────────────
  auto odom_noise = makeDiagonalNoise(
    cfg_.noise_odom_x, cfg_.noise_odom_y, cfg_.noise_odom_z,
    cfg_.noise_odom_roll, cfg_.noise_odom_pitch, cfg_.noise_odom_yaw);

  // ── Odometry BetweenFactors ───────────────────────────────────────────────
  geometry_msgs::msg::Pose prev_pose             = last_consumed_odom_pose_;
  const geometry_msgs::msg::Pose pre_batch_pose  = last_consumed_odom_pose_;
  const int batch_start_key                      = key_;
  const int key_before_batch                     = key_;  // saved for rollback if iSAM2 throws

  // Seed running estimate from the latest iSAM2 result; falls back at startup.
  gtsam::Pose3 current_estimate = optimized_pose_;
  try {
    current_estimate = isam2_->calculateEstimate<gtsam::Pose3>(X(key_));
  } catch (...) {}

  for (const auto & sample : local_odom) {
    const gtsam::Pose3 gtsam_prev    = msgToGtsam(prev_pose);
    const gtsam::Pose3 gtsam_current = msgToGtsam(sample.pose);
    const gtsam::Pose3 delta         = gtsam_prev.between(gtsam_current);
    const gtsam::Pose3 estimate      = current_estimate.compose(delta);

    key_++;
    new_factors_.add(
      gtsam::BetweenFactor<gtsam::Pose3>(X(key_ - 1), X(key_), delta, odom_noise));
    new_values_.insert(X(key_), estimate);

    current_estimate = estimate;
    prev_pose        = sample.pose;
  }

  last_consumed_odom_pose_              = local_odom.back().pose;
  const rclcpp::Time prev_consumed_stamp = last_consumed_odom_stamp_;
  last_consumed_odom_stamp_             = local_odom.back().timestamp;

  // ── IMU preintegration ────────────────────────────────────────────────────
  if (imu_preint_) {
    imu_preint_->addFactors(
      new_factors_, new_values_,
      std::move(local_imu), local_odom,
      batch_start_key, prev_consumed_stamp,
      optimized_bias_, optimized_velocity_,
      logger);
  }

  // ── Scan-match PriorFactors ───────────────────────────────────────────────
  // Each scan is gated against the |dyaw| of its nearest keyframe interval.
  // NDT is unreliable during in-place rotation.
  if (!local_scan.empty()) {
    // Pre-compute |dyaw| per keyframe interval; index 0 uses the pre-batch pose.
    std::vector<double> keyframe_dyaw(local_odom.size());
    {
      double yaw_prev = extractYaw(pre_batch_pose.orientation);
      for (std::size_t i = 0; i < local_odom.size(); ++i) {
        double dy = std::fmod(
          std::fabs(extractYaw(local_odom[i].pose.orientation) - yaw_prev),
          2.0 * M_PI);
        if (dy > M_PI) dy = 2.0 * M_PI - dy;
        keyframe_dyaw[i] = dy;
        yaw_prev = extractYaw(local_odom[i].pose.orientation);
      }
    }

    // One scan prior per keyframe per batch — prevents silent noise halving
    // when multiple scan messages happen to map to the same nearest keyframe.
    std::unordered_set<int> assigned_scan_keys;

    for (const auto & scan_msg : local_scan) {
      const rclcpp::Time scan_t(scan_msg.header.stamp);
      const gtsam::Pose3 scan_pose = msgToGtsam(scan_msg.pose.pose);

      // Find nearest keyframe by timestamp.
      int    best_i  = static_cast<int>(local_odom.size()) - 1;
      double best_dt = std::numeric_limits<double>::max();
      for (int i = 0; i < static_cast<int>(local_odom.size()); ++i) {
        const double dt = std::fabs((local_odom[i].timestamp - scan_t).seconds());
        if (dt < best_dt) { best_dt = dt; best_i = i; }
      }

      // Skip this scan if its keyframe interval had significant rotation.
      const double local_dyaw = keyframe_dyaw[best_i];
      if (local_dyaw > cfg_.lidar_rotation_gate_rad) {
        RCLCPP_DEBUG(logger,
          "[GraphManager] Scan prior skipped: keyframe[%d] |dyaw|=%.4f rad > gate=%.4f rad",
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

      // Guard: allow at most one scan prior per keyframe per batch.
      // A burst of scans all mapping to the same X(k) would be fused as
      // independent measurements, silently halving the noise sigma each time.
      if (!assigned_scan_keys.insert(target_key).second) {
        RCLCPP_DEBUG(logger,
          "[GraphManager] Scan prior skipped: X(%d) already has a prior this batch (dt=%.3fs)",
          target_key, best_dt);
        continue;
      }

      new_factors_.add(
        gtsam::PriorFactor<gtsam::Pose3>(X(target_key), scan_pose, lidar_noise));

      RCLCPP_DEBUG(logger,
        "[GraphManager] Scan prior -> X(%d), dt=%.3fs, dyaw=%.4f rad",
        target_key, best_dt, local_dyaw);
    }
  }

  // ── GPS factors ─────────────────────────────────────────────────────────────
  // Called after the odometry BetweenFactor loop so all X(k) keys known to
  // this batch are already present in new_values_.
  if (!local_gps.empty()) {
    addGpsBatch(local_gps, local_odom, batch_start_key, logger);
  }

  // ── iSAM2 update ─────────────────────────────────────────────────────────
  try {
    isam2_->update(new_factors_, new_values_);
    isam2_->update();  // extra pass for convergence
    new_factors_.resize(0);
    new_values_.clear();

    optimized_pose_ = isam2_->calculateEstimate<gtsam::Pose3>(X(key_));

    if (imu_preint_) {
      try {
        optimized_velocity_ =
          isam2_->calculateEstimate<gtsam::Vector3>(V(key_));
        optimized_bias_ =
          isam2_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(key_));
      } catch (...) {
        // V/B keys may not exist if no IMU factors were added this step
      }
    }
    // ── Sliding-window marginalization ──────────────────────────────────────
    // Margins the oldest key cluster so the Bayes tree does not grow without
    // bound.  Must be called after calculateEstimate() — marginalization
    // removes a key from iSAM2 but the bayes-factor chain is still intact.
    trimOldestKeys(logger);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "[GraphManager] iSAM2 update failed: %s", e.what());
    key_ = key_before_batch;   // roll back: next step must rebuild from a consistent state
    new_factors_.resize(0);
    new_values_.clear();
    return false;
  }

  return true;
}

void GraphManager::trimOldestKeys(const rclcpp::Logger & logger)
{
  if (cfg_.graph_max_size <= 0) return;  // feature disabled

  while ((key_ - oldest_kept_key_) > cfg_.graph_max_size) {
    gtsam::FastList<gtsam::Key> to_marg;
    to_marg.push_back(X(oldest_kept_key_));
    if (imu_preint_) {
      to_marg.push_back(V(oldest_kept_key_));
      to_marg.push_back(B(oldest_kept_key_));
    }
    try {
      isam2_->marginalizeLeaves(to_marg);
    } catch (const std::exception & e) {
      // marginalizeLeaves() requires the variable to be a leaf in the current
      // Bayes tree.  A scan prior bridging an old key to a recent one can
      // temporarily prevent this.  Stop trimming; the graph will shrink
      // naturally as newer factors push the old key towards leaf status.
      RCLCPP_WARN_ONCE(logger,
        "[GraphManager] Marginalization of key %d failed: %s. "
        "Trim halted until next opportunity.",
        oldest_kept_key_, e.what());
      break;
    }
    ++oldest_kept_key_;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// GPS helpers
// ─────────────────────────────────────────────────────────────────────────────

void GraphManager::addGpsFactor(gtsam::Key key,
                                const gtsam::Point3 & utm_position,
                                const gtsam::SharedNoiseModel & noise,
                                gtsam::Values & /*initial_values*/)
{
  // GPS does NOT add a new graph variable — it only constrains the translation
  // component of an existing Pose3 key.  The initial_values parameter is
  // intentionally unused and exists only for API consistency with other factor
  // addition helpers.

  // Guard: X(key) must already be in new_values_ (written by the odom loop
  // earlier in the same step() call).  If the key is absent, something is
  // badly wrong with call ordering; skip and log rather than insert an
  // orphaned factor which would crash iSAM2.
  if (!new_values_.exists(key)) {
    RCLCPP_WARN(
      rclcpp::get_logger("GraphManager"),
      "[GraphManager] addGpsFactor: key %lu not in current Values — "
      "skipping GPS factor. "
      "(Check that addGpsBatch is called after the odom BetweenFactor loop.)",
      static_cast<unsigned long>(key));
    return;
  }

  new_factors_.add(gtsam::GPSFactor(key, utm_position, noise));
}

void GraphManager::addGpsBatch(
  const std::vector<GpsSample> & local_gps,
  const std::vector<OdomSample> & local_odom,
  int batch_start_key,
  const rclcpp::Logger & logger)
{
  for (const auto & sample : local_gps) {

    // ── Find nearest keyframe by timestamp ───────────────────────────────────
    int    best_i  = static_cast<int>(local_odom.size()) - 1;
    double best_dt = std::numeric_limits<double>::max();
    for (int i = 0; i < static_cast<int>(local_odom.size()); ++i) {
      const double dt = std::fabs(
        (local_odom[i].timestamp - sample.timestamp).seconds());
      if (dt < best_dt) { best_dt = dt; best_i = i; }
    }

    const int target_key = batch_start_key + best_i + 1;

    // ── Antenna offset correction ────────────────────────────────────────────
    // Apply the GPS antenna → base_link rigid-body offset using the current
    // keyframe's estimated yaw.  x/y offsets are expressed in the body frame:
    //   corrected = R_map_body * offset_body + raw_gps_local
    // where R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)].
    // This is done here (not in GpsHandler) because the keyframe yaw is
    // available from new_values_, which lives inside GraphManager.
    double corrected_x = sample.x;
    double corrected_y = sample.y;

    if (std::fabs(cfg_.gps_offset_x) > 1e-9 ||
        std::fabs(cfg_.gps_offset_y) > 1e-9)
    {
      if (new_values_.exists(X(target_key))) {
        const gtsam::Pose3 kf_pose =
          new_values_.at<gtsam::Pose3>(X(target_key));
        const double yaw = kf_pose.rotation().yaw();
        // Rotate offset from body frame to map frame and subtract from raw GPS
        // (the antenna sits forward/left of base_link, so we subtract to get base_link pos).
        corrected_x = sample.x
                      - (cfg_.gps_offset_x * std::cos(yaw)
                         - cfg_.gps_offset_y * std::sin(yaw));
        corrected_y = sample.y
                      - (cfg_.gps_offset_x * std::sin(yaw)
                         + cfg_.gps_offset_y * std::cos(yaw));
      }
      // If key does not exist, addGpsFactor() will catch it — fall through.
    }

    const gtsam::Point3 measurement(corrected_x, corrected_y, 0.0);

    // ── Build HDOP-scaled noise model ────────────────────────────────────────
    // hdop in GpsSample is already clamped to ≥ 1.0 by GpsHandler.
    const double sx     = cfg_.noise_gps_sigma_x * sample.hdop;
    const double sy     = cfg_.noise_gps_sigma_y * sample.hdop;
    constexpr double sz = 999.0;  // unconstrained Z — 2D robot convention
    auto noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(sx, sy, sz));

    // ── Add factor (guarded) ─────────────────────────────────────────────────
    // Re-use the public addGpsFactor() for the key-existence guard and logging.
    gtsam::Values dummy;  // unused — GPS adds no new variable
    addGpsFactor(X(target_key), measurement, noise, dummy);

    RCLCPP_DEBUG(logger,
      "[GraphManager] GPS factor -> X(%d)  x=%.3f y=%.3f  "
      "hdop=%.2f sx=%.3f sy=%.3f  dt=%.3fs  offset=(%.3f, %.3f)",
      target_key, corrected_x, corrected_y,
      sample.hdop, sx, sy, best_dt,
      cfg_.gps_offset_x, cfg_.gps_offset_y);
  }
}

}  // namespace factor_graph_optimization
