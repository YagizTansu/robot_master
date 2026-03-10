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
  // Sort once for the entire batch (chrono order required by integrateMeasurement)
  std::sort(imu_samples.begin(), imu_samples.end(),
    [](const ImuSample & a, const ImuSample & b) {
      return a.timestamp < b.timestamp;
    });

  for (int i = 0; i < static_cast<int>(odom_samples.size()); ++i) {
    const int from_key = batch_start_key + i;
    const int to_key   = from_key + 1;

    // Seed V and B initial values for this key if not already present
    if (!values.exists(V(to_key))) values.insert(V(to_key), current_velocity);
    if (!values.exists(B(to_key))) values.insert(B(to_key), current_bias);

    const rclcpp::Time t_from =
      (i == 0) ? prev_consumed_stamp : odom_samples[i - 1].timestamp;
    const rclcpp::Time t_to = odom_samples[i].timestamp;

    bool imu_integrated = false;

    if (!imu_samples.empty() && t_from.nanoseconds() != 0 && t_to > t_from) {
      gtsam::PreintegratedCombinedMeasurements preint(params_, current_bias);
      int n_integrated = 0;

      for (std::size_t j = 1; j < imu_samples.size(); ++j) {
        const rclcpp::Time & t_a = imu_samples[j - 1].timestamp;
        const rclcpp::Time & t_b = imu_samples[j].timestamp;
        if (t_b <= t_from || t_a >= t_to) continue;

        // Clamp to [t_from, t_to] to avoid double-counting across batches.
        const double t_a_eff = std::max(t_a.seconds(), t_from.seconds());
        const double t_b_eff = std::min(t_b.seconds(), t_to.seconds());
        const double dt = t_b_eff - t_a_eff;
        if (dt <= 0.0 || dt > 0.5) continue;

        preint.integrateMeasurement(imu_samples[j - 1].accel,
                                    imu_samples[j - 1].gyro, dt);
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
      // No IMU coverage — pin V and B to prevent unconstrained nodes.
      graph.add(gtsam::PriorFactor<gtsam::Vector3>(
        V(to_key), current_velocity,
        gtsam::noiseModel::Isotropic::Sigma(3, fallback_vel_sigma_)));
      graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
        B(to_key), current_bias,
        gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector6() <<
            fallback_bias_accel_sigma_, fallback_bias_accel_sigma_, fallback_bias_accel_sigma_,
            fallback_bias_gyro_sigma_,  fallback_bias_gyro_sigma_,  fallback_bias_gyro_sigma_
          ).finished())));
    }
  }
}

}  // namespace factor_graph_optimization
