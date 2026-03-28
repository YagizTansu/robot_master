#pragma once

#include "factor_graph_optimization/lidar/scan_matcher_interface.hpp"

namespace factor_graph_optimization
{

/**
 * @brief ICP (Iterative Closest Point) scan matcher.
 *
 * Wraps pcl::IterativeClosestPoint.  The source cloud is pre-transformed by
 * initial_guess before ICP runs so that the algorithm only has to refine a
 * small residual, which keeps convergence fast and reliable.
 */
class IcpMatcher : public IScanMatcher
{
public:
  /**
   * @param max_iterations          Maximum ICP iterations.
   * @param max_correspondence_dist Maximum distance for correspondences (m).
   * @param transformation_epsilon  Convergence criterion.
   */
  /**
   * @param ransac_iterations        Number of RANSAC iterations for outlier rejection
   *                                 (0 = disabled; PCL default is 0).
   * @param ransac_outlier_threshold Maximum point-to-point distance to be considered
   *                                 an inlier for RANSAC (m).
   */
  IcpMatcher(int    max_iterations,
             double max_correspondence_dist,
             double transformation_epsilon,
             int    ransac_iterations        = 5,
             double ransac_outlier_threshold = 0.05);

  double match(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & target,
    const Eigen::Matrix4f & initial_guess,
    Eigen::Matrix4f & result_transform) override;

private:
  int    max_iterations_;
  double max_correspondence_dist_;
  double transformation_epsilon_;
  int    ransac_iterations_;
  double ransac_outlier_threshold_;
};

}  // namespace factor_graph_optimization
