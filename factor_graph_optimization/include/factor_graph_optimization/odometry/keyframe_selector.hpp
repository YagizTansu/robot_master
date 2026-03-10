#pragma once

#include <mutex>

#include <geometry_msgs/msg/pose.hpp>

namespace factor_graph_optimization
{

/**
 * @brief Decides whether a new odometry pose qualifies as a graph keyframe.
 *
 * A pose qualifies when it is farther from the last accepted keyframe than
 * either of two configurable thresholds:
 *  - translation (3-D Euclidean distance, metres), or
 *  - rotation    (shortest-path yaw delta, radians).
 *
 * Thread-safety: all public methods are individually mutex-protected so the
 * selector can be shared safely between a ROS callback thread (odomCallback)
 * and a reset path (initialPoseCallback).
 */
class KeyframeSelector
{
public:
  /**
   * @param translation_threshold  Minimum distance (m) to trigger a keyframe.
   * @param rotation_threshold     Minimum yaw change (rad) to trigger a keyframe.
   * @param initial_pose           Pose of the first pseudo-keyframe (typically
   *                               the configured initial pose of the robot).
   */
  KeyframeSelector(double translation_threshold,
                   double rotation_threshold,
                   const geometry_msgs::msg::Pose & initial_pose = geometry_msgs::msg::Pose{});

  /**
   * @brief Evaluate @p pose and, if it qualifies, atomically advance the selector.
   *
   * If the pose exceeds either threshold compared to the last accepted
   * keyframe, the internal pose is updated and @c true is returned.
   * Otherwise the internal state is unchanged and @c false is returned.
   *
   * This combined check-and-update is atomic under the internal mutex, which
   * prevents a race between two concurrent odomCallback invocations.
   *
   * @return @c true  if @p pose should be inserted as a new keyframe.
   * @return @c false otherwise.
   */
  bool checkAndUpdate(const geometry_msgs::msg::Pose & pose);

  /**
   * @brief Hard-reset to @p pose (used by /initialpose).
   *
   * After this call the next checkAndUpdate() will compare against @p pose.
   */
  void reset(const geometry_msgs::msg::Pose & pose);

  /// Read the last accepted keyframe pose (snapshot — may be immediately stale).
  geometry_msgs::msg::Pose lastPose() const;

private:
  double translation_threshold_;
  double rotation_threshold_;

  geometry_msgs::msg::Pose last_pose_;
  mutable std::mutex       mutex_;
};

}  // namespace factor_graph_optimization
