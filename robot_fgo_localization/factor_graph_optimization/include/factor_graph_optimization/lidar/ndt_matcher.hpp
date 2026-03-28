#pragma once

#include "factor_graph_optimization/lidar/scan_matcher_interface.hpp"

namespace factor_graph_optimization
{

/**
 * @brief NDT (Normal Distributions Transform) scan matcher.
 *
 * Wraps pcl::NormalDistributionsTransform.  All tuning parameters are
 * supplied at construction time and are immutable afterwards, ensuring
 * thread-safety for the match() call (provided the caller serialises its
 * own pcl::NdtRegistration instance — PCL NDT is not re-entrant).
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

  double match(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & target,
    const Eigen::Matrix4f & initial_guess,
    Eigen::Matrix4f & result_transform) override;

private:
  int    max_iterations_;
  double max_correspondence_dist_;
  double transformation_epsilon_;
  double ndt_resolution_;
  double ndt_step_size_;
};

}  // namespace factor_graph_optimization
