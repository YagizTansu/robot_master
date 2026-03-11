#pragma once

#include <Eigen/Core>

#include <geometry_msgs/msg/quaternion.hpp>

namespace factor_graph_optimization
{

/// Extract the yaw angle (rad) from a ROS quaternion message.
/// Roll and pitch are discarded — intended for 2-D planar robots.
double extractYaw(const geometry_msgs::msg::Quaternion & q);

/// Enforce a planar (2-D) constraint on a 4×4 homogeneous transform:
///   - z translation is forced to 0
///   - Rotation is rebuilt from yaw only (roll = pitch = 0)
///
/// @note Operates in single-precision (float) to match the PCL pipeline.
///       Results fed back into double-precision GTSAM poses will be implicitly
///       widened; no explicit cast is required.
Eigen::Matrix4f enforce2D(const Eigen::Matrix4f & T);

}  // namespace factor_graph_optimization
