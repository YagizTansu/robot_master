#pragma once

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "factor_graph_optimization/config/fgo_config.hpp"
#include "factor_graph_optimization/core/trust_weights.hpp"
#include "factor_graph_optimization/odometry/sensor_buffer.hpp"  // OdomSample, ImuSample
#include "factor_graph_optimization/imu/imu_preintegrator.hpp"
#include "factor_graph_optimization/gps/gps_handler.hpp"         // GpsSample

namespace factor_graph_optimization
{

using gtsam::symbol_shorthand::X;  // Pose3 keys
using gtsam::symbol_shorthand::V;  // Velocity keys  (gtsam::Vector3)
using gtsam::symbol_shorthand::B;  // IMU-bias keys  (gtsam::imuBias::ConstantBias)

/// Owns the iSAM2 factor graph, all optimised states, and the IMU preintegrator.
///
/// FgoNode creates one GraphManager, calls step() on every optimisation tick,
/// and reads results back through the const accessors.
class GraphManager
{
public:
  /// @param cfg         Full configuration (init pose, noise params, ISAM2 params, …).
  /// @param imu_preint  Optional IMU preintegrator; nullptr when IMU is disabled.
  GraphManager(const FgoConfig & cfg, std::unique_ptr<ImuPreintegrator> imu_preint);

  /// Re-initialise graph: update stored config, rebuild iSAM2, rebuild factor graph.
  /// Must be called under graph_mutex_ when shared with callbacks.
  void reinit(const FgoConfig & cfg);

  /// Build factors for one optimisation batch and run iSAM2.
  ///
  /// @param local_odom  Keyframe odometry samples drained since last step (non-empty).
  /// @param local_imu   IMU samples (by value; sorted internally before preintegration).
  /// @param local_scan  Scan-match pose estimates for this window.
  /// @param local_gps   GPS samples drained since last step (may be empty).
  /// @param trust       Per-sensor trust weights used to scale noise sigmas:
  ///                    scaled_sigma = base_sigma / max(w, EPSILON).  Passing a
  ///                    default-constructed TrustWeights{} (all weights = 1.0)
  ///                    reproduces the original static noise model exactly.
  /// @param logger      Caller's logger, used for debug / error messages.
  /// @return true  if iSAM2 was updated successfully.
  /// @return false if iSAM2 threw an exception (caller should skip publishing).
  bool step(
    const std::vector<OdomSample> & local_odom,
    std::vector<ImuSample>          local_imu,
    const std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> & local_scan,
    const std::vector<GpsSample> & local_gps,
    const TrustWeights & trust,
    const rclcpp::Logger & logger);

  // ── GPS factor interface ────────────────────────────────────────────────────

  /// Add a single GPSFactor to the pending new_factors_ batch.
  ///
  /// Called internally by addGpsBatch() after temporal matching, and exposed
  /// publicly so callers can inject one-off GPS constraints.
  ///
  /// @pre   X(@p key) must already exist in new_values_ (added by the odom
  ///        BetweenFactor loop earlier in the same step() call).
  ///        If the key is absent, the factor is skipped with a WARN log.
  ///
  /// @param key             GTSAM key of the target pose (use X(k)).
  /// @param utm_position    GPS measurement in local-UTM coordinates [x, y, 0].
  /// @param noise           3-vector Diagonal noise model [sx, sy, 999.0].
  ///                        Constructed inline in addGpsBatch() using HDOP-scaled sigmas.
  /// @param initial_values  Passed for API consistency; GPS does NOT insert a
  ///                        new variable — the parameter is intentionally unused.
  void addGpsFactor(gtsam::Key key,
                    const gtsam::Point3 & utm_position,
                    const gtsam::SharedNoiseModel & noise,
                    gtsam::Values & initial_values);

  // ── Accessors (all const; call under graph_mutex_) ────────────────────────
  const gtsam::Pose3 &                 optimizedPose()         const { return optimized_pose_; }
  const gtsam::Vector3 &               optimizedVelocity()     const { return optimized_velocity_; }
  const gtsam::imuBias::ConstantBias & optimizedBias()         const { return optimized_bias_; }
  const geometry_msgs::msg::Pose &     lastConsumedOdomPose()  const { return last_consumed_odom_pose_; }
  const rclcpp::Time &                 lastConsumedOdomStamp() const { return last_consumed_odom_stamp_; }
  int                                  currentKey()            const { return key_; }
  bool                                 hasImu()                const { return imu_preint_ != nullptr; }

private:
  void initIsam2();
  void initGraph();

  /// Temporal-match GPS samples to keyframes and call addGpsFactor() for each.
  /// Called from step() after the odom BetweenFactor loop, ensuring all
  /// X(k) keys are already present in new_values_ before GPS factors are built.
  /// @param w_gps  GPS trust weight; noise sigmas are divided by max(w_gps, EPSILON).
  void addGpsBatch(
    const std::vector<GpsSample> & local_gps,
    const std::vector<OdomSample> & local_odom,
    int batch_start_key,
    double w_gps,
    const rclcpp::Logger & logger);

  /// Add soft PriorFactor<Vector3> on each V(k) produced this batch to
  /// indirectly modulate the IMU CombinedImuFactor's influence.
  ///
  /// WHY: PreintegrationCombinedParams are baked into the preintegrator at
  /// construction time and cannot be rescaled per-interval without
  /// rebuilding the entire preintegration object.  Instead, a velocity
  /// continuity prior is added alongside the CombinedImuFactor:
  ///
  ///   sigma_v = cfg_.fallback_vel_sigma / max(w_imu, EPSILON)
  ///
  /// At w_imu = 1.0 the prior is (fallback_vel_sigma / 1.0) wide, adding a
  /// gentle regularisation consistent with the existing V(0) prior.
  /// As w_imu decreases toward EPSILON, sigma_v grows, the prior weakens,
  /// and the graph can deviate more freely from the IMU-predicted velocity —
  /// effectively reducing the IMU's influence on the trajectory estimate.
  void addImuVelocityPriors(
    int batch_start_key,
    int batch_end_key,
    double w_imu,
    const rclcpp::Logger & logger);

  /**
   * @brief Marginalize the oldest X/V/B key cluster until the active window
   *        is within cfg_.graph_max_size nodes.
   *
   * Uses gtsam::ISAM2::marginalizeLeaves() which converts the variable to a
   * dense marginal factor on its neighbors.  Stops (with a one-time warning)
   * if marginalization fails, because the variable is not yet a leaf in the
   * Bayes tree (e.g. a scan prior linked an old key to a recent one).
   */
  void trimOldestKeys(const rclcpp::Logger & logger);

  /// Build a geometry_msgs::Pose from the init-pose fields in \p cfg.
  static geometry_msgs::msg::Pose makePoseFromCfg(const FgoConfig & cfg);

  // ── Graph state ───────────────────────────────────────────────────────────
  FgoConfig cfg_;

  std::unique_ptr<gtsam::ISAM2>   isam2_;
  gtsam::NonlinearFactorGraph     new_factors_;
  gtsam::Values                   new_values_;
  int                             key_{0};
  int                             oldest_kept_key_{0};  ///< sliding-window lower bound: oldest key still in iSAM2

  // ── Optimised estimates ───────────────────────────────────────────────────
  gtsam::Pose3                    optimized_pose_;
  gtsam::Vector3                  optimized_velocity_{gtsam::Vector3::Zero()};
  gtsam::imuBias::ConstantBias    optimized_bias_{};

  // ── Odom tracking (anchor for map→odom TF) ────────────────────────────────
  geometry_msgs::msg::Pose        last_consumed_odom_pose_;
  rclcpp::Time                    last_consumed_odom_stamp_;

  // ── IMU preintegrator ─────────────────────────────────────────────────────
  std::unique_ptr<ImuPreintegrator> imu_preint_;
};

}  // namespace factor_graph_optimization
