#include "factor_graph_optimization/core/noise_model.hpp"

#include <stdexcept>

namespace factor_graph_optimization
{

gtsam::noiseModel::Diagonal::shared_ptr makeDiagonalNoise(
  double x, double y, double z,
  double roll, double pitch, double yaw)
{
  // Zero or negative sigmas produce a singular information matrix (Σ⁻¹),
  // causing iSAM2's Cholesky factorization to encounter a zero pivot → NaN.
  if (x <= 0.0 || y <= 0.0 || z <= 0.0 ||
      roll <= 0.0 || pitch <= 0.0 || yaw <= 0.0)
  {
    throw std::invalid_argument(
      "makeDiagonalNoise: all sigma values must be strictly positive");
  }

  // GTSAM Pose3 tangent-space order: [roll, pitch, yaw, x, y, z]
  // Note: API arguments are (x,y,z,roll,pitch,yaw) — reordered internally.
  return gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector6() << roll, pitch, yaw, x, y, z).finished());
}

}  // namespace factor_graph_optimization
