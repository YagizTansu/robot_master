#include "factor_graph_optimization/lidar/icp_matcher.hpp"

#include <pcl/common/transforms.h>

namespace factor_graph_optimization
{

IcpMatcher::IcpMatcher(int    max_iterations,
                       double max_correspondence_dist,
                       double transformation_epsilon,
                       int    ransac_iterations,
                       double ransac_outlier_threshold)
{
  icp_.setMaximumIterations(max_iterations);
  icp_.setMaxCorrespondenceDistance(max_correspondence_dist);
  icp_.setTransformationEpsilon(transformation_epsilon);
  // Enable RANSAC outlier rejection when configured; guards against false
  // correspondences in cluttered environments.
  if (ransac_iterations > 0) {
    icp_.setRANSACIterations(ransac_iterations);
    icp_.setRANSACOutlierRejectionThreshold(ransac_outlier_threshold);
  }
}

void IcpMatcher::setTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr & target)
{
  icp_.setInputTarget(target);
  has_target_ = true;
}

double IcpMatcher::match(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & source,
  const Eigen::Matrix4f & initial_guess,
  Eigen::Matrix4f & result_transform)
{
  // Pre-transform source to the initial guess position so ICP only refines
  // a small residual — this keeps convergence fast and reliable.
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_init(
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*source, *source_init, initial_guess);
  icp_.setInputSource(source_init);

  pcl::PointCloud<pcl::PointXYZ> aligned;
  icp_.align(aligned);

  if (icp_.hasConverged()) {
    result_transform = icp_.getFinalTransformation() * initial_guess;
    return static_cast<double>(icp_.getFitnessScore());
  }

  result_transform = initial_guess;
  return 1e9;  // non-convergence → gated out by fitness threshold
}

}  // namespace factor_graph_optimization
