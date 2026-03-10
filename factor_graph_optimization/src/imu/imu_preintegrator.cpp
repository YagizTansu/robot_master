#include "factor_graph_optimization/imu/imu_preintegrator.hpp"

#include <algorithm>
#include <cmath>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/Vector.h>

namespace factor_graph_optimization
{

using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;

// ─────────────────────────────────────────────────────────────────────────────
// Construction — build PreintegrationCombinedParams from config
// ─────────────────────────────────────────────────────────────────────────────

ImuPreintegrator::ImuPreintegrator(const FgoConfig & cfg)
: fallback_vel_sigma_(cfg.fallback_vel_sigma)
, fallback_bias_accel_sigma_(cfg.fallback_bias_accel_sigma)
, fallback_bias_gyro_sigma_(cfg.fallback_bias_gyro_sigma)
{
  // ENU (Z-up) world frame convention; gravity acts along -Z.
  auto p = gtsam::PreintegrationCombinedParams::MakeSharedU(cfg.imu_gravity);

  // White-noise spectral densities (continuous time)
  p->accelerometerCovariance =
    gtsam::I_3x3 * (cfg.noise_imu_accel_sigma * cfg.noise_imu_accel_sigma);
  p->gyroscopeCovariance =
    gtsam::I_3x3 * (cfg.noise_imu_gyro_sigma * cfg.noise_imu_gyro_sigma);

  // Numerical integration uncertainty (trapezoidal discretisation error)
  p->integrationCovariance =
    gtsam::I_3x3 * (cfg.noise_imu_integration_sigma * cfg.noise_imu_integration_sigma);

  // Bias random-walk spectral densities
  p->biasAccCovariance =
    gtsam::I_3x3 * (cfg.noise_imu_accel_bias_sigma * cfg.noise_imu_accel_bias_sigma);
  p->biasOmegaCovariance =
    gtsam::I_3x3 * (cfg.noise_imu_gyro_bias_sigma * cfg.noise_imu_gyro_bias_sigma);

  // Small uncertainty on the initial bias-integral (numerical stability)
  p->biasAccOmegaInt = gtsam::I_6x6 * cfg.imu_bias_acc_omega_int;

  params_ = p;
}

// ─────────────────────────────────────────────────────────────────────────────
// addFactors — preintegrate IMU and populate graph for each odom interval
// ─────────────────────────────────────────────────────────────────────────────

void ImuPreintegrator::addFactors(
  gtsam::NonlinearFactorGraph & graph,
  gtsam::Values & values,
  std::vector<ImuSample> imu_samples,
  const std::vector<OdomSample> & odom_samples,
  int batch_start_key,
  const rclcpp::Time & prev_consumed_stamp,
  const gtsam::imuBias::ConstantBias & current_bias,
  const gtsam::Vector3 & current_velocity) const
{
  // Sort once; skip if already ordered (ROS subscriber usually delivers in order).
  auto imu_cmp = [](const ImuSample & a, const ImuSample & b) {
    return a.timestamp < b.timestamp;
  };
  if (!std::is_sorted(imu_samples.begin(), imu_samples.end(), imu_cmp)) {
    std::sort(imu_samples.begin(), imu_samples.end(), imu_cmp);
  }

  // Running lower bound into imu_samples: since keyframe intervals are sequential
  // we never need to revisit pairs whose right endpoint is <= the previous t_from.
  std::size_t imu_search_start = 1;

  for (int i = 0; i < static_cast<int>(odom_samples.size()); ++i) {
    const int from_key = batch_start_key + i;
    const int to_key   = from_key + 1;

    const rclcpp::Time t_from =
      (i == 0) ? prev_consumed_stamp : odom_samples[i - 1].timestamp;
    const rclcpp::Time t_to = odom_samples[i].timestamp;

    // Seed V and B from the closest preceding key that already has an estimate,
    // falling back to the step-start values.  This gives iSAM2 better
    // linearization points for later intervals within a large batch.
    gtsam::Vector3 v_seed = values.exists(V(from_key))
                            ? values.at<gtsam::Vector3>(V(from_key))
                            : current_velocity;
    gtsam::imuBias::ConstantBias b_seed =
      values.exists(B(from_key))
      ? values.at<gtsam::imuBias::ConstantBias>(B(from_key))
      : current_bias;

    if (!values.exists(V(to_key))) values.insert(V(to_key), v_seed);
    if (!values.exists(B(to_key))) values.insert(B(to_key), b_seed);

    bool imu_integrated = false;

    if (!imu_samples.empty() && t_from.nanoseconds() != 0 && t_to > t_from) {
      // Seed preintegration with the per-interval bias for better first-order
      // bias correction inside CombinedImuFactor.
      gtsam::PreintegratedCombinedMeasurements preint(params_, b_seed);
      int n_integrated = 0;

      // Advance past pairs whose right endpoint is already <= t_from; they
      // belong to an earlier keyframe interval and will never overlap this one.
      while (imu_search_start < imu_samples.size() &&
             imu_samples[imu_search_start].timestamp <= t_from) {
        ++imu_search_start;
      }

      for (std::size_t j = imu_search_start; j < imu_samples.size(); ++j) {
        const rclcpp::Time & t_a = imu_samples[j - 1].timestamp;
        const rclcpp::Time & t_b = imu_samples[j].timestamp;

        // Sorted data: once the pair starts at or after t_to there is no overlap.
        if (t_a >= t_to) break;

        const double t_a_eff = std::max(t_a.seconds(), t_from.seconds());
        const double t_b_eff = std::min(t_b.seconds(), t_to.seconds());
        const double dt = t_b_eff - t_a_eff;
        if (dt <= 0.0 || dt > 0.5) continue;

        // Linearly interpolate the IMU measurement to the clamped interval start.
        // When the pair straddles t_from (alpha > 0) this avoids injecting a
        // measurement value that pre-dates the integration window.
        const double seg_dt = t_b.seconds() - t_a.seconds();
        const double alpha = (seg_dt > 1e-9)
                             ? (t_a_eff - t_a.seconds()) / seg_dt
                             : 0.0;
        const gtsam::Vector3 accel =
          imu_samples[j - 1].accel + alpha * (imu_samples[j].accel - imu_samples[j - 1].accel);
        const gtsam::Vector3 gyro =
          imu_samples[j - 1].gyro  + alpha * (imu_samples[j].gyro  - imu_samples[j - 1].gyro);

        preint.integrateMeasurement(accel, gyro, dt);
        ++n_integrated;
      }

      if (n_integrated > 0) {
        graph.add(gtsam::CombinedImuFactor(
          X(from_key), V(from_key),
          X(to_key),   V(to_key),
          B(from_key), B(to_key),
          preint));
        imu_integrated = true;
      }
    }

    if (!imu_integrated) {
      // No IMU coverage — pin V and B with the per-interval seeds to prevent
      // unconstrained nodes while inheriting any intra-batch refinement.
      graph.add(gtsam::PriorFactor<gtsam::Vector3>(
        V(to_key), v_seed,
        gtsam::noiseModel::Isotropic::Sigma(3, fallback_vel_sigma_)));
      graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
        B(to_key), b_seed,
        gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector6() <<
            fallback_bias_accel_sigma_, fallback_bias_accel_sigma_, fallback_bias_accel_sigma_,
            fallback_bias_gyro_sigma_,  fallback_bias_gyro_sigma_,  fallback_bias_gyro_sigma_
          ).finished())));
    }
  }
}

}  // namespace factor_graph_optimization
