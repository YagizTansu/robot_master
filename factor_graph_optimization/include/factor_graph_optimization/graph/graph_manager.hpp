#pragma once

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "factor_graph_optimization/config/fgo_config.hpp"
#include "factor_graph_optimization/odometry/sensor_buffer.hpp"  // OdomSample, ImuSample
#include "factor_graph_optimization/imu/imu_preintegrator.hpp"

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
  /// @param logger      Caller's logger, used for debug / error messages.
  /// @return true  if iSAM2 was updated successfully.
  /// @return false if iSAM2 threw an exception (caller should skip publishing).
  bool step(
    const std::vector<OdomSample> & local_odom,
    std::vector<ImuSample>          local_imu,
    const std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> & local_scan,
    const rclcpp::Logger & logger);

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

  /// Build a geometry_msgs::Pose from the init-pose fields in \p cfg.
  static geometry_msgs::msg::Pose makePoseFromCfg(const FgoConfig & cfg);

  // ── Graph state ───────────────────────────────────────────────────────────
  FgoConfig cfg_;

  std::unique_ptr<gtsam::ISAM2>   isam2_;
  gtsam::NonlinearFactorGraph     new_factors_;
  gtsam::Values                   new_values_;
  int                             key_{0};

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
