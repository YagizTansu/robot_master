#pragma once

#include <pcl/registration/ndt.h>

#include "factor_graph_optimization/lidar/scan_matcher_interface.hpp"

namespace factor_graph_optimization
{

/**
 * @brief NDT (Normal Distributions Transform) scan matcher.
 *
 * Wraps pcl::NormalDistributionsTransform.  The target cloud's NDT voxel grid
 * is built once in setTarget() and reused for all subsequent match() calls,
 * avoiding the expensive rebuild on every scan.
 */
class NdtMatcher : public IScanMatcher
{
public:
  /**
   * @param max_iterations         Maximum NDT iterations.
   * @param max_correspondence_dist Maximum distance for correspondences (m).
   * @param transformation_epsilon  Convergence criterion.
   * @param ndt_resolution          Voxel resolution of the NDT grid (m).
   * @param ndt_step_size           Newton gradient-descent step size (m).
   *                                 Smaller = more stable, more iterations. Default: 0.1.
   */
  NdtMatcher(int    max_iterations,
             double max_correspondence_dist,
             double transformation_epsilon,
             double ndt_resolution,
             double ndt_step_size = 0.1);

  void setTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr & target) override;
  bool hasTarget() const override { return has_target_; }

  double match(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & source,
    const Eigen::Matrix4f & initial_guess,
    Eigen::Matrix4f & result_transform) override;

private:
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
  bool has_target_{false};
};

}  // namespace factor_graph_optimization
