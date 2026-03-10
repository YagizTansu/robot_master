#include "factor_graph_optimization/lidar/icp_matcher.hpp"

#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

namespace factor_graph_optimization
{

IcpMatcher::IcpMatcher(int    max_iterations,
                       double max_correspondence_dist,
                       double transformation_epsilon)
: max_iterations_(max_iterations)
, max_correspondence_dist_(max_correspondence_dist)
, transformation_epsilon_(transformation_epsilon)
{}

double IcpMatcher::match(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & source,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & target,
  const Eigen::Matrix4f & initial_guess,
  Eigen::Matrix4f & result_transform)
{
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaximumIterations(max_iterations_);
  icp.setMaxCorrespondenceDistance(max_correspondence_dist_);
  icp.setTransformationEpsilon(transformation_epsilon_);

  icp.setInputTarget(target);

  // Pre-transform source to the initial guess position so ICP only refines
  // a small residual — this keeps convergence fast and reliable.
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_init(
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*source, *source_init, initial_guess);
  icp.setInputSource(source_init);

  pcl::PointCloud<pcl::PointXYZ> aligned;
  icp.align(aligned);

  if (icp.hasConverged()) {
    result_transform = icp.getFinalTransformation() * initial_guess;
    return static_cast<double>(icp.getFitnessScore());
  }

  result_transform = initial_guess;
  return 1e9;  // non-convergence → gated out by fitness threshold
}

}  // namespace factor_graph_optimization
