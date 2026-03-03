#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>


namespace robot_fgo_localization
{

struct ScanMatchResult
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double fitness_score{1.0};  // ICP fitness: lower = better match
  bool success{false};
};

/**
 * @brief Lidar scan matcher using ICP (Iterative Closest Point).
 *
 * Converts a LaserScan to a 2D PCL point cloud and aligns it against
 * the pre-built map point cloud extracted from the OccupancyGrid.
 */
class ScanMatcher
{
public:
  explicit ScanMatcher(rclcpp::Logger logger);

  /**
   * @brief Load the map from an OccupancyGrid message and build the map point cloud.
   * @param map  The occupancy grid (from map_server).
   */
  void setMap(const nav_msgs::msg::OccupancyGrid & map);

  /**
   * @brief Match a laser scan against the map.
   * @param scan         Incoming laser scan.
   * @param guess_x      Initial guess X (from FGO predict step), metres.
   * @param guess_y      Initial guess Y.
   * @param guess_yaw    Initial guess yaw (rad).
   * @param min_fitness  Minimum ICP fitness score to accept (0–1, lower = better).
   * @return ScanMatchResult with pose correction and success flag.
   */
  ScanMatchResult match(
    const sensor_msgs::msg::LaserScan & scan,
    double guess_x, double guess_y, double guess_yaw,
    double min_fitness = 0.85);

  bool hasMap() const { return map_cloud_ != nullptr && !map_cloud_->empty(); }

private:
  using PointT = pcl::PointXYZ;  // ICP internals require z — set to 0.0 for 2D
  using Cloud  = pcl::PointCloud<PointT>;

  Cloud::Ptr scanToCloud(const sensor_msgs::msg::LaserScan & scan) const;
  Cloud::Ptr mapToCloud(const nav_msgs::msg::OccupancyGrid & map) const;

  rclcpp::Logger logger_;
  Cloud::Ptr map_cloud_;

  // ICP
  double icp_max_correspondence_dist_{0.5};   // metres
  int    icp_max_iterations_{50};
  double icp_transformation_epsilon_{1e-6};
  double icp_euclidean_fitness_epsilon_{1e-5};

  // Voxel filter leaf size for downsampling
  double voxel_leaf_size_{0.05};  // metres
};

}  // namespace robot_fgo_localization
