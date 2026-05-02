#ifndef ROBOT_CUSTOM_LOCAL_PLANNER__CUSTOM_LOCAL_PLANNER_HPP_
#define ROBOT_CUSTOM_LOCAL_PLANNER__CUSTOM_LOCAL_PLANNER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <limits>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace robot_custom_local_planner
{

/**
 * @brief Trajectory for a Tricycle/Ackermann (forklift) robot.
 *
 * Motion is fully determined by (linear_vel, steering_angle).
 * angular_vel is derived:  ω = v·tan(δ) / L
 * rotate_in_place trajectories use v=0 with a direct angular_vel sample.
 */
struct Trajectory
{
  double linear_vel;       // cmd_vel.linear.x   [m/s]
  double steering_angle;   // front-wheel angle   [rad]  (internal, for simulation)
  double angular_vel;      // cmd_vel.angular.z   [rad/s]  = v·tan(δ)/L
  bool   rotate_in_place;  // position fixed, heading changes only
  double cost;
  std::vector<geometry_msgs::msg::PoseStamped> poses;
};

/**
 * @class CustomLocalPlanner
 * @brief DWA-based local planner for Ackermann/Tricycle (forklift) robots.
 *
 * Samples (v, δ) space respecting:
 *   - Traction acceleration limit   [m/s²]   → kinco TRACTION_SLEW_RATE × wheel_radius
 *   - Steering slew-rate limit      [rad/s]  → kinco STEERING_SLEW_RATE
 *   - Physical steering angle limit [rad]    → ±π/2 for single front wheel
 *
 * Tricycle kinematic model:  ω = v·tan(δ)/L  (identical to kinco_bridge.py)
 * Rotate-in-place via v=0 + direct wz samples when heading error is large.
 */
class CustomLocalPlanner : public nav2_core::Controller
{
public:
  CustomLocalPlanner() = default;
  ~CustomLocalPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /** Generate (v, δ) samples + optional rotate-in-place samples. */
  std::vector<Trajectory> generateTrajectories(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity);

  /**
   * @brief Simulate tricycle forward kinematics: x, y, θ integration.
   *
   * Normal:           ẋ = v·cos(θ),  ẏ = v·sin(θ),  θ̇ = v·tan(δ)/L
   * Rotate-in-place:  ẋ = 0,         ẏ = 0,          θ̇ = rotate_angular_vel
   */
  Trajectory simulateTrajectory(
    const geometry_msgs::msg::PoseStamped & pose,
    double linear_vel,
    double steering_angle,
    bool   rotate_in_place    = false,
    double rotate_angular_vel = 0.0);

  double scoreTrajectory(const Trajectory & traj);

  /** Weighted average perpendicular distance from trajectory to path. */
  double calculatePathAlignmentScore(const Trajectory & traj);

  /** Costmap lethal/inflation check; returns inf on collision. */
  double calculateObstacleScore(const Trajectory & traj);

  /** Distance from trajectory end to local carrot point. */
  double calculateGoalDistanceScore(const Trajectory & traj);

  /** Angular difference between trajectory end heading and path tangent. */
  double calculateHeadingAlignmentScore(const Trajectory & traj);

  /** Quadratic penalty on steering angle: δ² — prefers straight-ahead paths. */
  double calculateSteeringPenalty(const Trajectory & traj);

  /** Estimate current steering angle δ from (vx, wz) velocity feedback. */
  double estimateCurrentSteering(const geometry_msgs::msg::Twist & velocity) const;

  /** Extract forward-looking local window from the global plan. */
  nav_msgs::msg::Path pruneGlobalPlan(
    const geometry_msgs::msg::PoseStamped & pose,
    double prune_distance);

  void publishLocalPlan(const Trajectory & traj);

  // ── Node interfaces ──────────────────────────────────────────────────────
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("CustomLocalPlanner")};
  rclcpp::Clock::SharedPtr clock_;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr global_plan_transformed_pub_;

  std::string plugin_name_;
  nav_msgs::msg::Path global_plan_;
  nav_msgs::msg::Path pruned_plan_;
  size_t last_closest_idx_{0};

  // ── Ackermann geometry ───────────────────────────────────────────────────
  double wheelbase_{1.0957};           // [m]   front-to-rear axle distance
  double max_steering_angle_{1.5708};  // [rad] physical limit ≈ π/2

  // ── Velocity limits ──────────────────────────────────────────────────────
  double max_linear_vel_{0.5};    // [m/s]  forward maximum
  double min_linear_vel_{0.05};   // [m/s]  forward minimum (motor alive threshold)
  double max_reverse_vel_{0.0};   // [m/s]  reverse max (0 = disabled)
  double max_angular_vel_{0.8};   // [rad/s] cap for rotate-in-place

  // ── Acceleration / rate limits ───────────────────────────────────────────
  double linear_acc_limit_{0.545};   // [m/s²]  = TRACTION_SLEW_RATE × WHEEL_RADIUS
  double steering_slew_rate_{1.5};   // [rad/s] = kinco STEERING_SLEW_RATE

  // ── Sampling ─────────────────────────────────────────────────────────────
  int    linear_samples_{7};
  int    steering_samples_{11};
  int    rotate_samples_{5};
  double rotate_max_vel_{0.5};         // [rad/s] max wz for rotate-in-place
  double heading_error_trigger_{0.5};  // [rad]   threshold to add rotate samples
  double sim_time_{2.0};
  double sim_granularity_{0.1};
  double control_dt_{0.5};             // DWA velocity window width = acc × control_dt

  // ── Cost weights ─────────────────────────────────────────────────────────
  double path_distance_weight_{32.0};
  double heading_alignment_weight_{16.0};
  double goal_distance_weight_{20.0};
  double obstacle_weight_{1.0};
  double velocity_reward_weight_{5.0};
  double steering_penalty_weight_{2.0};

  // ── Tolerances ───────────────────────────────────────────────────────────
  double transform_tolerance_{0.5};
  double xy_goal_tolerance_{0.25};
  double yaw_goal_tolerance_{0.25};
  double prune_plan_distance_{4.0};

  // ── Runtime state ────────────────────────────────────────────────────────
  double speed_limit_ratio_{1.0};
  double current_steering_angle_{0.0};  // tracked/estimated current δ [rad]

  // ── Dynamic reconfigure ──────────────────────────────────────────────────
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace robot_custom_local_planner

#endif  // ROBOT_CUSTOM_LOCAL_PLANNER__CUSTOM_LOCAL_PLANNER_HPP_
