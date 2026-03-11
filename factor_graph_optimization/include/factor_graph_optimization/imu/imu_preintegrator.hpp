#pragma once

#include <memory>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

#include <boost/shared_ptr.hpp>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include "factor_graph_optimization/config/fgo_config.hpp"
#include "factor_graph_optimization/odometry/sensor_buffer.hpp"  // OdomSample, ImuSample

namespace factor_graph_optimization
{

/**
 * @brief Encapsulates IMU preintegration parameters and the per-interval
 *        factor-building algorithm used inside the FGO optimisation step.
 *
 * Construction:  Builds `PreintegrationCombinedParams` from a loaded
 *                `FgoConfig`; also stores fallback-prior sigmas for intervals
 *                where no IMU data is available.
 *
 * Usage (inside optimizationStep):
 * @code
 *   if (imu_preint_) {
 *     imu_preint_->addFactors(
 *       new_factors_, new_values_,
 *       std::move(local_imu), local_odom,
 *       batch_start_key, prev_consumed_stamp,
 *       optimized_bias_, optimized_velocity_);
 *   }
 * @endcode
 */
class ImuPreintegrator
{
public:
  using Params = gtsam::PreintegrationCombinedParams;

  /**
   * @brief Build preintegration params from the loaded configuration.
   *
   * @param cfg  Fully-initialised FgoConfig.  Only the `noise.imu.*`,
   *             `prior.*` and `fallback.*` fields are read.
   */
  explicit ImuPreintegrator(const FgoConfig & cfg);

  /**
   * @brief Populate GTSAM graph with CombinedImuFactor (or fallback priors)
   *        for every keyframe interval in the current optimisation batch.
   *
   * For each interval [from_key, to_key]:
   *  - If IMU samples span the interval → add a `CombinedImuFactor`.
   *  - Otherwise → add `PriorFactor<Vector3>` on V and
   *    `PriorFactor<ConstantBias>` on B (tight fallback priors so the
   *    graph remains fully-constrained).
   *
   * V(to_key) and B(to_key) are inserted into @p values if not already present,
   * seeded from @p current_velocity and @p current_bias.
   *
   * @param graph               Factor graph to append to.
   * @param values              Initial values map to append to.
   * @param imu_samples         IMU samples for this batch (will be sorted in-place).
   * @param odom_samples        Odom keyframes added this step; defines interval boundaries.
   * @param batch_start_key     Key index of the last consumed odom pose before this batch.
   * @param prev_consumed_stamp Timestamp of the last consumed odom pose before this batch
   *                            (used as t_from for the first interval).
   * @param current_bias        Latest optimised IMU bias (used to seed preintegration).
   * @param current_velocity    Latest optimised velocity (used to seed fallback priors).
   */
  void addFactors(
    gtsam::NonlinearFactorGraph & graph,
    gtsam::Values & values,
    std::vector<ImuSample> imu_samples,
    const std::vector<OdomSample> & odom_samples,
    int batch_start_key,
    const rclcpp::Time & prev_consumed_stamp,
    const gtsam::imuBias::ConstantBias & current_bias,
    const gtsam::Vector3 & current_velocity,
    const rclcpp::Logger & logger) const;

  /// Returns the underlying `PreintegrationCombinedParams` (read-only after construction).
  const Params & params() const { return *params_; }

private:
  boost::shared_ptr<Params> params_;

  // Fallback prior sigmas (used when no IMU data covers a keyframe interval)
  double fallback_vel_sigma_;
  double fallback_bias_accel_sigma_;
  double fallback_bias_gyro_sigma_;
};

}  // namespace factor_graph_optimization
