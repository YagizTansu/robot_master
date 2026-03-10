#include "factor_graph_optimization/lidar/ndt_matcher.hpp"

#include <pcl/registration/ndt.h>

namespace factor_graph_optimization
{

NdtMatcher::NdtMatcher(int    max_iterations,
                       double max_correspondence_dist,
                       double transformation_epsilon,
                       double ndt_resolution)
: max_iterations_(max_iterations)
, max_correspondence_dist_(max_correspondence_dist)
, transformation_epsilon_(transformation_epsilon)
, ndt_resolution_(ndt_resolution)
{}

double NdtMatcher::match(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & source,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & target,
  const Eigen::Matrix4f & initial_guess,
  Eigen::Matrix4f & result_transform)
{
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setMaximumIterations(max_iterations_);
  ndt.setMaxCorrespondenceDistance(max_correspondence_dist_);
  ndt.setTransformationEpsilon(transformation_epsilon_);
  ndt.setResolution(static_cast<float>(ndt_resolution_));

  ndt.setInputTarget(target);
  ndt.setInputSource(source);

  pcl::PointCloud<pcl::PointXYZ> aligned;
  ndt.align(aligned, initial_guess);

  if (ndt.hasConverged()) {
    result_transform = ndt.getFinalTransformation();
    return static_cast<double>(ndt.getFitnessScore());
  }

  result_transform = initial_guess;
  return 1e9;  // non-convergence → gated out by fitness threshold
}

}  // namespace factor_graph_optimization
