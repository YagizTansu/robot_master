#pragma once

#include <memory>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace factor_graph_optimization
{

/**
 * @brief Converts a ROS OccupancyGrid into a downsampled PCL point cloud
 *        suitable for NDT / ICP scan matching.
 *
 * Occupied cells (value == 100) are extracted at a configurable Z height,
 * then a VoxelGrid filter is applied to keep computation tractable.
 *
 * Usage:
 * @code
 *   MapBuilder builder(cfg_.map_z_height, cfg_.map_voxel_leaf_size);
 *   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = builder.build(grid_msg);
 * @endcode
 */
class MapBuilder
{
public:
  /**
   * @param map_z_height       Z coordinate assigned to every map point (m).
   *                           Typically 0 for flat 2-D environments.
   * @param map_voxel_leaf_size VoxelGrid leaf size (m).  Smaller values keep
   *                           more detail but increase matcher runtime.
   */
  MapBuilder(double map_z_height, double map_voxel_leaf_size);

  /**
   * @brief Convert an OccupancyGrid to a filtered PCL cloud.
   *
   * @param grid  Incoming occupancy grid message.
   * @return Shared pointer to the downsampled point cloud.
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr build(
    const nav_msgs::msg::OccupancyGrid & grid) const;

private:
  double map_z_height_;
  double map_voxel_leaf_size_;
};

}  // namespace factor_graph_optimization
