#include "robot_fgo_localization/scan_matcher.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>

namespace robot_fgo_localization
{

ScanMatcher::ScanMatcher(rclcpp::Logger logger)
: logger_(logger), map_cloud_(nullptr)
{}

// ─── Map Loading ─────────────────────────────────────────────────────────────
void ScanMatcher::setMap(const nav_msgs::msg::OccupancyGrid & map)
{
  map_cloud_ = mapToCloud(map);
  RCLCPP_INFO(
    logger_,
    "[ScanMatcher] Map loaded: %zu occupied cells → point cloud",
    map_cloud_->size());
}

// ─── Scan → Cloud ─────────────────────────────────────────────────────────────
ScanMatcher::Cloud::Ptr
ScanMatcher::scanToCloud(const sensor_msgs::msg::LaserScan & scan) const
{
  auto cloud = std::make_shared<Cloud>();
  const float angle_min  = scan.angle_min;
  const float angle_inc  = scan.angle_increment;
  const float range_min  = scan.range_min;
  const float range_max  = scan.range_max;

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const float r = scan.ranges[i];
    if (!std::isfinite(r) || r < range_min || r > range_max) {
      continue;
    }
    const float angle = angle_min + static_cast<float>(i) * angle_inc;
    PointT p;
    p.x = r * std::cos(angle);
    p.y = r * std::sin(angle);
    p.z = 0.0f;  // 2D laser — z is always 0
    cloud->push_back(p);
  }
  return cloud;
}

// ─── OccupancyGrid → Cloud ────────────────────────────────────────────────────
ScanMatcher::Cloud::Ptr
ScanMatcher::mapToCloud(const nav_msgs::msg::OccupancyGrid & map) const
{
  auto cloud = std::make_shared<Cloud>();
  const double res = map.info.resolution;
  const double ox  = map.info.origin.position.x;
  const double oy  = map.info.origin.position.y;
  const uint32_t w = map.info.width;
  const uint32_t h = map.info.height;

  for (uint32_t row = 0; row < h; ++row) {
    for (uint32_t col = 0; col < w; ++col) {
      const int8_t cell = map.data[row * w + col];
      // Only occupied cells (value 100 = definitely occupied; > 50 = considered occupied)
      if (cell > 50) {
        PointT p;
        p.x = static_cast<float>(ox + (col + 0.5) * res);
        p.y = static_cast<float>(oy + (row + 0.5) * res);
        p.z = 0.0f;  // 2D map — z is always 0
        cloud->push_back(p);
      }
    }
  }

  // Downsample the map cloud to speed up ICP
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(
    static_cast<float>(voxel_leaf_size_),
    static_cast<float>(voxel_leaf_size_),
    0.01f);  // 2D cloud — tiny Z
  auto filtered = std::make_shared<Cloud>();
  vg.filter(*filtered);
  return filtered;
}

// ─── Match ────────────────────────────────────────────────────────────────────
ScanMatchResult ScanMatcher::match(
  const sensor_msgs::msg::LaserScan & scan,
  double guess_x, double guess_y, double guess_yaw,
  double min_fitness)
{
  ScanMatchResult result;

  if (!hasMap()) {
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 5000,
      "[ScanMatcher] No map loaded yet — skipping ICP");
    return result;
  }

  auto scan_cloud = scanToCloud(scan);
  if (scan_cloud->empty()) {
    return result;
  }

  // ── Transform scan cloud to map frame using current guess ──────────────────
  // Build 3×3 rotation + translation from guess pose
  Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
  const float cy = std::cos(static_cast<float>(guess_yaw));
  const float sy = std::sin(static_cast<float>(guess_yaw));
  init_guess(0, 0) = cy;  init_guess(0, 1) = -sy;
  init_guess(1, 0) = sy;  init_guess(1, 1) =  cy;
  init_guess(0, 3) = static_cast<float>(guess_x);
  init_guess(1, 3) = static_cast<float>(guess_y);

  // ── Voxel downsample the scan ──────────────────────────────────────────────
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(scan_cloud);
  vg.setLeafSize(
    static_cast<float>(voxel_leaf_size_),
    static_cast<float>(voxel_leaf_size_),
    0.01f);
  auto scan_ds = std::make_shared<Cloud>();
  vg.filter(*scan_ds);

  // ── ICP ───────────────────────────────────────────────────────────────────
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setInputSource(scan_ds);
  icp.setInputTarget(map_cloud_);
  icp.setMaxCorrespondenceDistance(icp_max_correspondence_dist_);
  icp.setMaximumIterations(icp_max_iterations_);
  icp.setTransformationEpsilon(icp_transformation_epsilon_);
  icp.setEuclideanFitnessEpsilon(icp_euclidean_fitness_epsilon_);

  Cloud aligned;
  icp.align(aligned, init_guess);

  if (!icp.hasConverged()) {
    RCLCPP_WARN(logger_, "[ScanMatcher] ICP did not converge");
    return result;
  }

  const double fitness = icp.getFitnessScore();
  // For ICP fitness: lower = better. We accept if below threshold.
  // min_fitness parameter is interpreted as max acceptable fitness score.
  if (fitness > min_fitness) {
    RCLCPP_DEBUG(logger_,
      "[ScanMatcher] ICP fitness %.4f exceeds threshold %.4f — rejecting", fitness, min_fitness);
    return result;
  }

  // ── Extract pose from final transform ─────────────────────────────────────
  const Eigen::Matrix4f T = icp.getFinalTransformation();
  result.x   = static_cast<double>(T(0, 3));
  result.y   = static_cast<double>(T(1, 3));
  result.yaw = static_cast<double>(std::atan2(T(1, 0), T(0, 0)));
  result.fitness_score = fitness;
  result.success = true;

  RCLCPP_DEBUG(logger_,
    "[ScanMatcher] ICP OK — x:%.3f y:%.3f yaw:%.3f fitness:%.4f",
    result.x, result.y, result.yaw, fitness);

  return result;
}

}  // namespace robot_fgo_localization
