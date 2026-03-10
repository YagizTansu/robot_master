#include "factor_graph_optimization/odometry/keyframe_selector.hpp"

#include <cmath>

#include "factor_graph_optimization/core/geometry_2d.hpp"

namespace factor_graph_optimization
{

KeyframeSelector::KeyframeSelector(double translation_threshold,
                                   double rotation_threshold,
                                   const geometry_msgs::msg::Pose & initial_pose)
: translation_threshold_(translation_threshold)
, rotation_threshold_(rotation_threshold)
, last_pose_(initial_pose)
{}

bool KeyframeSelector::checkAndUpdate(const geometry_msgs::msg::Pose & pose)
{
  std::lock_guard<std::mutex> lk(mutex_);

  const double dx   = pose.position.x - last_pose_.position.x;
  const double dy   = pose.position.y - last_pose_.position.y;
  const double dz   = pose.position.z - last_pose_.position.z;
  const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

  double dyaw = std::fmod(
    std::fabs(extractYaw(pose.orientation) - extractYaw(last_pose_.orientation)),
    2.0 * M_PI);
  if (dyaw > M_PI) dyaw = 2.0 * M_PI - dyaw;

  const bool qualifies = (dist > translation_threshold_) ||
                         (dyaw > rotation_threshold_);

  if (qualifies) {
    last_pose_ = pose;
  }
  return qualifies;
}

void KeyframeSelector::reset(const geometry_msgs::msg::Pose & pose)
{
  std::lock_guard<std::mutex> lk(mutex_);
  last_pose_ = pose;
}

geometry_msgs::msg::Pose KeyframeSelector::lastPose() const
{
  std::lock_guard<std::mutex> lk(mutex_);
  return last_pose_;
}

}  // namespace factor_graph_optimization
