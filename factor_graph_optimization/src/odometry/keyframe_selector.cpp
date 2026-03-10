#include "factor_graph_optimization/odometry/keyframe_selector.hpp"

#include <cmath>

#include "factor_graph_optimization/core/geometry_2d.hpp"

namespace factor_graph_optimization
{

KeyframeSelector::KeyframeSelector(double translation_threshold,
                                   double rotation_threshold,
                                   double max_time_sec,
                                   const geometry_msgs::msg::Pose & initial_pose)
: translation_threshold_(translation_threshold)
, rotation_threshold_(rotation_threshold)
, max_time_sec_(max_time_sec)
, last_pose_(initial_pose)
, last_accepted_time_(0, 0, RCL_ROS_TIME)
, is_first_(true)
{}

bool KeyframeSelector::checkAndUpdate(const geometry_msgs::msg::Pose & pose,
                                      const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lk(mutex_);

  // Always accept the first pose after construction or reset — this guarantees
  // the graph sees at least one keyframe before any distance/time gating begins.
  if (is_first_) {
    last_pose_          = pose;
    last_accepted_time_ = stamp;
    is_first_           = false;
    return true;
  }

  const double dx   = pose.position.x - last_pose_.position.x;
  const double dy   = pose.position.y - last_pose_.position.y;
  const double dz   = pose.position.z - last_pose_.position.z;
  const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

  double dyaw = std::fmod(
    std::fabs(extractYaw(pose.orientation) - extractYaw(last_pose_.orientation)),
    2.0 * M_PI);
  if (dyaw > M_PI) dyaw = 2.0 * M_PI - dyaw;

  // Time-based forced keyframe: keeps the graph fresh when the robot is stationary.
  const bool timeout    = (max_time_sec_ > 0.0) &&
                          ((stamp - last_accepted_time_).seconds() > max_time_sec_);
  const bool qualifies  = timeout ||
                          (dist > translation_threshold_) ||
                          (dyaw > rotation_threshold_);

  if (qualifies) {
    last_pose_          = pose;
    last_accepted_time_ = stamp;
  }
  return qualifies;
}

void KeyframeSelector::reset(const geometry_msgs::msg::Pose & pose)
{
  std::lock_guard<std::mutex> lk(mutex_);
  last_pose_          = pose;
  is_first_           = true;
  last_accepted_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

geometry_msgs::msg::Pose KeyframeSelector::lastPose() const
{
  std::lock_guard<std::mutex> lk(mutex_);
  return last_pose_;
}

}  // namespace factor_graph_optimization
