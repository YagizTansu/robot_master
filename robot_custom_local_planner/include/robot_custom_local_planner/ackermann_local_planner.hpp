#ifndef ROBOT_CUSTOM_LOCAL_PLANNER__ACKERMANN_LOCAL_PLANNER_HPP_
#define ROBOT_CUSTOM_LOCAL_PLANNER__ACKERMANN_LOCAL_PLANNER_HPP_

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
 * @brief Trajectory structure for Ackermann (non-holonomic) robots.
 *
 * Ackermann robots can only move forward/backward on a curve defined by
 * a single steering angle (or equivalently, an angular velocity). There
 * is no independent lateral velocity component.
 */
struct AckermannTrajectory
{
  double linear_vel;    ///< Forward (+) / backward (-) velocity  [m/s]
  double angular_vel;   ///< Rotational velocity (yaw rate)         [rad/s]
  double steering_angle;///< Equivalent front-wheel steering angle  [rad]
  double cost;
  std::vector<geometry_msgs::msg::PoseStamped> poses;
};

/**
 * @class AckermannLocalPlanner
 * @brief Nav2 local planner for Ackermann-steered (car-like) robots.
 *
 * Implements a DWA-style sampling-based local planner using the bicycle
 * kinematic model.  The robot can accelerate or decelerate along its
 * heading and steer by sending (v, ω) commands — it cannot move sideways.
 *
 * Key differences from the holonomic CustomLocalPlanner:
 *  - No lateral velocity (linear.y is always zero in the output).
 *  - Trajectory sampling is over (v, ω) pairs, enforcing a minimum turning
 *    radius derived from the wheelbase and max steering angle.
 *  - Simulation uses the bicycle kinematic model:
 *      x'     = v · cos(θ)
 *      y'     = v · sin(θ)
 *      θ'     = ω  (= v · tan(δ) / L)
 *  - An additional "heading alignment" score rewards trajectories whose
 *    final orientation matches the path direction.
 */
class AckermannLocalPlanner : public nav2_core::Controller
{
public:
  AckermannLocalPlanner() = default;
  ~AckermannLocalPlanner() override = default;

  // ------------------------------------------------------------------ //
  //  nav2_core::Controller interface                                     //
  // ------------------------------------------------------------------ //

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
  // ------------------------------------------------------------------ //
  //  Internal helpers                                                    //
  // ------------------------------------------------------------------ //

  /**
   * @brief Generate candidate (v, ω) trajectories and simulate them forward.
   */
  std::vector<AckermannTrajectory> generateTrajectories(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity);

  /**
   * @brief Simulate one trajectory using the bicycle kinematic model.
   * @param pose      Robot's current pose (global frame)
   * @param linear_vel Forward speed [m/s]
   * @param angular_vel Yaw rate [rad/s]
   */
  AckermannTrajectory simulateTrajectory(
    const geometry_msgs::msg::PoseStamped & pose,
    double linear_vel,
    double angular_vel);

  /**
   * @brief Compute the aggregate cost (lower is better) for a trajectory.
   */
  double scoreTrajectory(const AckermannTrajectory & trajectory);

  /** @brief Distance from trajectory poses to the nearest global-plan pose. */
  double calculatePathAlignmentScore(const AckermannTrajectory & trajectory);

  /** @brief Costmap-based collision / inflation cost. */
  double calculateObstacleScore(const AckermannTrajectory & trajectory);

  /** @brief Euclidean distance from trajectory end to the global goal. */
  double calculateGoalDistanceScore(const AckermannTrajectory & trajectory);

  /**
   * @brief Yaw alignment between trajectory end and the local path direction.
   *
   * Penalises trajectories whose final heading deviates from the path
   * direction — essential for car-like robots that cannot correct orientation
   * independently of position.
   */
  double calculateHeadingAlignmentScore(const AckermannTrajectory & trajectory);

  /**
   * @brief Extract the section of the global plan within prune_distance of
   *        the robot's current position.
   */
  nav_msgs::msg::Path pruneGlobalPlan(
    const geometry_msgs::msg::PoseStamped & pose,
    double prune_distance);

  /** @brief Publish the best trajectory as a nav_msgs::Path for RViz. */
  void publishLocalPlan(const AckermannTrajectory & trajectory);

  // ------------------------------------------------------------------ //
  //  ROS 2 interfaces                                                   //
  // ------------------------------------------------------------------ //
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("AckermannLocalPlanner")};
  rclcpp::Clock::SharedPtr clock_;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr global_plan_transformed_pub_;

  std::string plugin_name_;

  // ------------------------------------------------------------------ //
  //  State                                                               //
  // ------------------------------------------------------------------ //
  nav_msgs::msg::Path global_plan_;    ///< Full global plan
  nav_msgs::msg::Path pruned_plan_;    ///< Look-ahead window of global plan

  // ------------------------------------------------------------------ //
  //  Parameters                                                          //
  // ------------------------------------------------------------------ //

  // Velocity limits
  double max_linear_vel_;       ///< Max forward speed          [m/s]
  double min_linear_vel_;       ///< Min forward speed (≥ 0)    [m/s]
  double max_reverse_vel_;      ///< Max reverse speed (≥ 0)    [m/s] – set 0 to disallow reversing
  double max_angular_vel_;      ///< Max yaw rate               [rad/s]
  double min_angular_vel_;      ///< Min yaw rate (negative)    [rad/s]

  // Acceleration limits
  double linear_acc_limit_;     ///< Max linear acceleration    [m/s²]
  double angular_acc_limit_;    ///< Max angular acceleration   [rad/s²]

  // Ackermann-specific geometry
  double wheelbase_;            ///< Distance front–rear axle   [m]
  double max_steering_angle_;   ///< Physical max steering angle [rad]

  // Sampling
  int linear_samples_;          ///< # of linear velocity samples
  int angular_samples_;         ///< # of angular velocity samples
  double sim_time_;             ///< Forward simulation horizon  [s]
  double sim_granularity_;      ///< Time step for simulation    [s]

  // Scoring weights
  double path_distance_weight_;
  double goal_distance_weight_;
  double obstacle_weight_;
  double heading_alignment_weight_;  ///< Extra weight for Ackermann heading alignment

  // Tolerances / pruning
  double transform_tolerance_;
  double xy_goal_tolerance_;
  double yaw_goal_tolerance_;
  double prune_plan_distance_;

  // Speed scaling (from setSpeedLimit)
  double speed_limit_ratio_{1.0};
};

}  // namespace robot_custom_local_planner

#endif  // ROBOT_CUSTOM_LOCAL_PLANNER__ACKERMANN_LOCAL_PLANNER_HPP_
