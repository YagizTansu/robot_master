#include "factor_graph_optimization/core/pose_conversion.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <gtsam/geometry/Point3.h>

namespace factor_graph_optimization
{

gtsam::Pose3 msgToGtsam(const geometry_msgs::msg::Pose & pose)
{
  // Normalize before constructing Rot3: non-unit quaternions from ROS messages
  // (serialization drift, dead-reckoning accumulation) produce invalid SO(3)
  // rotations that corrupt GTSAM Jacobians. Direct Rot3::Quaternion avoids the
  // RPY roundtrip singularity at pitch = ±π/2 (gimbal lock).
  tf2::Quaternion tq(
    pose.orientation.x, pose.orientation.y,
    pose.orientation.z, pose.orientation.w);
  tq.normalize();

  return gtsam::Pose3(
    gtsam::Rot3::Quaternion(tq.w(), tq.x(), tq.y(), tq.z()),
    gtsam::Point3(pose.position.x, pose.position.y, pose.position.z));
}

geometry_msgs::msg::Pose gtsamToMsg(const gtsam::Pose3 & pose)
{
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.translation().x();
  msg.position.y = pose.translation().y();
  msg.position.z = pose.translation().z();

  const gtsam::Quaternion q = pose.rotation().toQuaternion();
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();
  return msg;
}

}  // namespace factor_graph_optimization
