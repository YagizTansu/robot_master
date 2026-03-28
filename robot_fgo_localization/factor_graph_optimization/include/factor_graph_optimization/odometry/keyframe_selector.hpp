#pragma once

#include <mutex>

#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace factor_graph_optimization
{

/**
 * @brief Decides whether a new odometry pose qualifies as a graph keyframe.
 *
 * A pose qualifies when it is farther from the last accepted keyframe than
 * either of two configurable thresholds:
 *  - translation (2-D XY + Z Euclidean distance, metres), or
 *  - rotation    (shortest-path yaw delta, radians), or
 *  - time        (elapsed seconds since last keyframe; forces a keyframe even
 *                 when the robot is stationary, keeping the graph fresh)
 *
 * Additionally the very first call after construction or reset() always
 * returns true, guaranteeing that at least one keyframe is created before
 * any threshold comparison takes place.
 *
 * Thread-safety: all public methods are individually mutex-protected so the
 * selector can be shared safely between a ROS callback thread (odomCallback)
 * and a reset path (initialPoseCallback).
 */
class KeyframeSelector
{
public:
  /**
   * @param translation_threshold  Minimum XYZ distance (m) to trigger a keyframe.
   * @param rotation_threshold     Minimum yaw change (rad) to trigger a keyframe.
   * @param max_time_sec           Maximum seconds between keyframes even with no
   *                               motion.  Pass 0 to disable time-based forcing.
   * @param initial_pose           Pose of the first pseudo-keyframe (typically
   *                               the configured initial pose of the robot).
   */
  KeyframeSelector(double translation_threshold,
                   double rotation_threshold,
                   double max_time_sec,
                   const geometry_msgs::msg::Pose & initial_pose = geometry_msgs::msg::Pose{});

  /**
   * @brief Evaluate @p pose and, if it qualifies, atomically advance the selector.
   *
   * The pose qualifies if any of the following hold:
   *  - This is the first call after construction / reset (always accepted).
   *  - Distance from the last keyframe exceeds translation_threshold.
   *  - |dyaw| from the last keyframe exceeds rotation_threshold.
   *  - Time since the last keyframe exceeds max_time_sec (when > 0).
   *
   * @param pose   Current odometry pose.
   * @param stamp  Timestamp of the odometry message (used for time-based gating).
   * @return @c true  if @p pose should be inserted as a new keyframe.
   * @return @c false otherwise.
   */
  bool checkAndUpdate(const geometry_msgs::msg::Pose & pose, const rclcpp::Time & stamp);

  /**
   * @brief Hard-reset to @p pose (used by /initialpose).
   *
   * After this call the next checkAndUpdate() will always be accepted (first-pose logic).
   */
  void reset(const geometry_msgs::msg::Pose & pose);

  /// Read the last accepted keyframe pose (snapshot — may be immediately stale).
  geometry_msgs::msg::Pose lastPose() const;

private:
  double translation_threshold_;
  double rotation_threshold_;
  double max_time_sec_;  ///< 0 = disabled

  geometry_msgs::msg::Pose last_pose_;
  rclcpp::Time             last_accepted_time_;
  bool                     is_first_{true};  ///< force-accept the first call after ctor/reset
  mutable std::mutex       mutex_;
};

}  // namespace factor_graph_optimization
