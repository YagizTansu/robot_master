#include "factor_graph_optimization/lidar/ndt_matcher.hpp"

namespace factor_graph_optimization
{

NdtMatcher::NdtMatcher(int    max_iterations,
                       double max_correspondence_dist,
                       double transformation_epsilon,
                       double ndt_resolution,
                       double ndt_step_size)
{
  ndt_.setMaximumIterations(max_iterations);
  ndt_.setMaxCorrespondenceDistance(max_correspondence_dist);
  ndt_.setTransformationEpsilon(transformation_epsilon);
  ndt_.setResolution(static_cast<float>(ndt_resolution));
  ndt_.setStepSize(static_cast<float>(ndt_step_size));
}

void NdtMatcher::setTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr & target)
{
  ndt_.setInputTarget(target);
  has_target_ = true;
}

double NdtMatcher::match(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & source,
  const Eigen::Matrix4f & initial_guess,
  Eigen::Matrix4f & result_transform)
{
  ndt_.setInputSource(source);

  pcl::PointCloud<pcl::PointXYZ> aligned;
  ndt_.align(aligned, initial_guess);

  if (ndt_.hasConverged()) {
    result_transform = ndt_.getFinalTransformation();
    return static_cast<double>(ndt_.getFitnessScore());
  }

  result_transform = initial_guess;
  return 1e9;  // non-convergence → gated out by fitness threshold
}

}  // namespace factor_graph_optimization
