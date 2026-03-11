#include "factor_graph_optimization/lidar/map_builder.hpp"

#include <pcl/filters/voxel_grid.h>

namespace factor_graph_optimization
{

MapBuilder::MapBuilder(double map_z_height, double map_voxel_leaf_size)
: map_z_height_(map_z_height)
, map_voxel_leaf_size_(map_voxel_leaf_size)
{}

pcl::PointCloud<pcl::PointXYZ>::Ptr
MapBuilder::build(const nav_msgs::msg::OccupancyGrid & grid) const
{
  // ── 1. Extract occupied cells ───────────────────────────────────────────
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->header.frame_id = grid.header.frame_id;

  const double res     = grid.info.resolution;
  const double ox      = grid.info.origin.position.x;
  const double oy      = grid.info.origin.position.y;
  const unsigned int W = grid.info.width;
  const unsigned int H = grid.info.height;

  cloud->reserve(W * H / 4);  // rough upper bound

  for (unsigned int row = 0; row < H; ++row) {
    for (unsigned int col = 0; col < W; ++col) {
      if (grid.data[row * W + col] == 100) {  // occupied
        pcl::PointXYZ pt;
        pt.x = static_cast<float>(ox + (static_cast<double>(col) + 0.5) * res);
        pt.y = static_cast<float>(oy + (static_cast<double>(row) + 0.5) * res);
        pt.z = static_cast<float>(map_z_height_);
        cloud->push_back(pt);
      }
    }
  }
  cloud->width    = static_cast<uint32_t>(cloud->size());
  cloud->height   = 1;
  cloud->is_dense = true;

  // ── 2. Downsample with VoxelGrid ────────────────────────────────────────
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud);
  const auto leaf = static_cast<float>(map_voxel_leaf_size_);
  vg.setLeafSize(leaf, leaf, leaf);

  auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  vg.filter(*filtered);

  return filtered;
}

}  // namespace factor_graph_optimization
