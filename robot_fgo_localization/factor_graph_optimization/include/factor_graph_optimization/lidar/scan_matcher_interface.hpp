#pragma once

#include <memory>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace factor_graph_optimization
{

/**
 * @brief Abstract interface for 2-D point-cloud registration algorithms.
 *
 * Both inputs (source, target) are expressed in the same coordinate frame.
 * The caller is responsible for supplying an initial_guess transform.
 * Implementations write the refined 4×4 rigid transform to result_transform
 * and return a fitness score (lower is better).  A score of 1e9 signals
 * non-convergence and will be gated out by the fitness threshold.
 *
 * The target cloud (map) is set once via setTarget() and its internal data
 * structures (NDT voxel grid, ICP KD-tree) are cached until the next
 * setTarget() call.  match() uses the cached target — callers must NOT
 * pass the target cloud to match(); it uses the one set via setTarget().
 */
class IScanMatcher
{
public:
  virtual ~IScanMatcher() = default;

  /**
   * @brief Set and preprocess the target (map) cloud.
   *
   * Call this whenever the map changes.  Implementations build their
   * internal spatial data structures (NDT voxel grid, ICP KD-tree) once
   * and reuse them for subsequent match() calls.
   *
   * @param target  Map point cloud (in the map frame).
   */
  virtual void setTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr & target) = 0;

  /**
   * @brief Run point-cloud registration against the cached target.
   *
   * @param source        Scan point cloud (in the base / lidar frame).
   * @param initial_guess Coarse transform from frame of source to frame of target.
   * @param result_transform  Output: refined 4×4 rigid body transform.
   * @return Fitness score (mean squared distance of correspondences; 1e9 on failure).
   */
  virtual double match(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & source,
    const Eigen::Matrix4f & initial_guess,
    Eigen::Matrix4f & result_transform) = 0;

  /// Returns true once setTarget() has been called with a non-empty cloud.
  virtual bool hasTarget() const = 0;
};

}  // namespace factor_graph_optimization
