#pragma once

#include <algorithm>
#include <cstdio>

namespace factor_graph_optimization
{

/// Per-sensor trust scores fed into the FGO noise-scaling layer.
///
/// Each weight w ∈ (0, 1] represents how much the localization system trusts
/// a given sensor modality during the current optimisation step.  A weight of
/// 1.0 reproduces the baseline static noise model exactly; lower values widen
/// the sensor's noise sigma (= weaken its information contribution).
///
/// The scaling law applied to every sigma element is:
///   scaled_sigma = base_sigma / max(w, EPSILON)
///
/// This keeps the information matrix non-singular for any w in [EPSILON, 1.0].
///
/// Thread safety: FgoNode stores a TrustWeights instance that is written by
/// the trust-weight subscriber callback and read by the 50 Hz optimisation
/// timer.  The enclosing mutex (graph_mutex_) serialises these accesses.
struct TrustWeights
{
  double w_encoder{1.0};  ///< Wheel-odometry trust  (BetweenFactor)
  double w_imu    {1.0};  ///< IMU preintegration trust (CombinedImuFactor proxy)
  double w_lidar  {1.0};  ///< LiDAR scan-match trust  (PriorFactor<Pose3>)
  double w_gps    {1.0};  ///< GPS/GNSS trust           (GPSFactor)

  /// Absolute lower bound on any weight.  Prevents information-matrix
  /// singularity: the largest possible sigma is base_sigma / EPSILON.
  static constexpr double EPSILON = 1e-6;

  /// Clamp all weights to [EPSILON, 1.0] in-place.
  /// Must be called whenever weights arrive from an external source.
  void clamp()
  {
    w_encoder = std::max(EPSILON, std::min(1.0, w_encoder));
    w_imu     = std::max(EPSILON, std::min(1.0, w_imu));
    w_lidar   = std::max(EPSILON, std::min(1.0, w_lidar));
    w_gps     = std::max(EPSILON, std::min(1.0, w_gps));
  }

  /// Print weights to stdout for quick debugging.
  void print() const
  {
    std::printf(
      "[TrustWeights] encoder=%.4f  imu=%.4f  lidar=%.4f  gps=%.4f\n",
      w_encoder, w_imu, w_lidar, w_gps);
  }

  /// Return true when all weights are at their unit default (no-op scaling).
  bool isUnity() const
  {
    constexpr double tol = 1e-9;
    return (std::fabs(w_encoder - 1.0) < tol &&
            std::fabs(w_imu     - 1.0) < tol &&
            std::fabs(w_lidar   - 1.0) < tol &&
            std::fabs(w_gps     - 1.0) < tol);
  }
};

}  // namespace factor_graph_optimization
