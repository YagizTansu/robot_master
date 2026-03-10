#include "factor_graph_optimization/graph/graph_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <tf2/LinearMath/Quaternion.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
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
  params.evaluateNonlinearError = false;
  isam2_ = std::make_unique<gtsam::ISAM2>(params);
}

void GraphManager::initGraph()
{
  key_ = 0;
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
      optimized_bias_, optimized_velocity_);
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
      new_factors_.add(
        gtsam::PriorFactor<gtsam::Pose3>(X(target_key), scan_pose, lidar_noise));

      RCLCPP_DEBUG(logger,
        "[GraphManager] Scan prior -> X(%d), dt=%.3fs, dyaw=%.4f rad",
        target_key, best_dt, local_dyaw);
    }
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
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "[GraphManager] iSAM2 update failed: %s", e.what());
    new_factors_.resize(0);
    new_values_.clear();
    return false;
  }

  return true;
}

}  // namespace factor_graph_optimization
