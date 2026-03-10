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
 */
class IScanMatcher
{
public:
  virtual ~IScanMatcher() = default;

  /**
   * @brief Run point-cloud registration.
   *
   * @param source        Scan point cloud (in the base / lidar frame).
   * @param target        Map point cloud (in the map frame).
   * @param initial_guess Coarse transform from frame of source to frame of target.
   * @param result_transform  Output: refined 4×4 rigid body transform.
   * @return Fitness score (mean squared distance of correspondences; 1e9 on failure).
   */
  virtual double match(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & target,
    const Eigen::Matrix4f & initial_guess,
    Eigen::Matrix4f & result_transform) = 0;
};

}  // namespace factor_graph_optimization
