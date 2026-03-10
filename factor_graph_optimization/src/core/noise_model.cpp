#include "factor_graph_optimization/core/noise_model.hpp"

namespace factor_graph_optimization
{

gtsam::noiseModel::Diagonal::shared_ptr makeDiagonalNoise(
  double x, double y, double z,
  double roll, double pitch, double yaw)
{
  // GTSAM Pose3 sigma order: [roll, pitch, yaw, x, y, z]
  return gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector6() << roll, pitch, yaw, x, y, z).finished());
}

}  // namespace factor_graph_optimization
