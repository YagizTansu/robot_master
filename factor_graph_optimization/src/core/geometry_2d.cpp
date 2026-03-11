#include "factor_graph_optimization/core/geometry_2d.hpp"

#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace factor_graph_optimization
{

double extractYaw(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  tq.normalize();  // guard against accumulated float drift in ROS messages
  double roll, pitch, yaw;
  tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
  return yaw;
}

Eigen::Matrix4f enforce2D(const Eigen::Matrix4f & T)
{
  Eigen::Matrix4f T2d = T;

  // Force z translation to 0
  T2d(2, 3) = 0.0f;

  // Reconstruct rotation: keep only yaw
  const float yaw = std::atan2(T2d(1, 0), T2d(0, 0));
  Eigen::Matrix3f R2d = Eigen::Matrix3f::Identity();
  R2d(0, 0) =  std::cos(yaw);
  R2d(0, 1) = -std::sin(yaw);
  R2d(1, 0) =  std::sin(yaw);
  R2d(1, 1) =  std::cos(yaw);
  T2d.block<3, 3>(0, 0) = R2d;

  return T2d;
}

}  // namespace factor_graph_optimization
