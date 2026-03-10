#pragma once

#include <gtsam/linear/NoiseModel.h>

namespace factor_graph_optimization
{

/// Build a 6-DOF diagonal noise model whose sigma vector follows the GTSAM
/// Pose3 convention: [roll, pitch, yaw, x, y, z].
gtsam::noiseModel::Diagonal::shared_ptr makeDiagonalNoise(
  double x, double y, double z,
  double roll, double pitch, double yaw);

}  // namespace factor_graph_optimization
