#pragma once

#include <gtsam/linear/NoiseModel.h>

namespace factor_graph_optimization
{

/// Build a 6-DOF diagonal noise model for a GTSAM Pose3 factor.
///
/// @param x,y,z         Translation sigmas (metres).
/// @param roll,pitch,yaw Rotation sigmas (radians).
///
/// Arguments are supplied in translation-first order for readability at the
/// call site.  Internally they are reordered to the GTSAM Pose3 tangent-space
/// convention: [roll, pitch, yaw, x, y, z].
///
/// @throws std::invalid_argument if any sigma value is ≤ 0 (would make the
///         information matrix singular and crash iSAM2's Cholesky solver).
gtsam::noiseModel::Diagonal::shared_ptr makeDiagonalNoise(
  double x, double y, double z,
  double roll, double pitch, double yaw);

}  // namespace factor_graph_optimization
