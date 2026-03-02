#ifndef ROBOT_CUSTOM_LOCAL_PLANNER__CUSTOM_LOCAL_PLANNER_HPP_
#define ROBOT_CUSTOM_LOCAL_PLANNER__CUSTOM_LOCAL_PLANNER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <limits>

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
 * @brief Trajectory structure for holonomic robots (mecanum/omni wheels)
 */
struct Trajectory
{
  double linear_vel_x;   // Forward/backward velocity
  double linear_vel_y;   // Lateral (strafe) velocity
  double angular_vel;    // Rotational velocity
  double cost;
  std::vector<geometry_msgs::msg::PoseStamped> poses;
};

/**
 * @class CustomLocalPlanner
 * @brief Custom local planner for mecanum/omni-directional robots
 * Supports holonomic motion including lateral (strafe) movement
 */
class CustomLocalPlanner : public nav2_core::Controller
{
public:
  CustomLocalPlanner() = default;
  ~CustomLocalPlanner() override = default;

  /**
   * @brief Configure controller
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller
   */
  void cleanup() override;

  /**
   * @brief Activate controller
   */
  void activate() override;

  /**
   * @brief Deactivate controller
   */
  void deactivate() override;

  /**
   * @brief Compute velocity commands
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief Set the plan to follow
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Set the speed limit
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief Generate trajectories to evaluate
   */
  std::vector<Trajectory> generateTrajectories(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity);

  /**
   * @brief Simulate holonomic trajectory forward in time
   */
  Trajectory simulateTrajectory(
    const geometry_msgs::msg::PoseStamped & pose,
    double linear_vel_x,
    double linear_vel_y,
    double angular_vel);

  /**
   * @brief Score a trajectory
   */
  double scoreTrajectory(const Trajectory & trajectory);

  /**
   * @brief Calculate path alignment score
   */
  double calculatePathAlignmentScore(const Trajectory & trajectory);

  /**
   * @brief Calculate obstacle score
   */
  double calculateObstacleScore(const Trajectory & trajectory);

  /**
   * @brief Calculate goal distance score
   */
  double calculateGoalDistanceScore(const Trajectory & trajectory);

  /**
   * @brief Transform global plan to local frame
   */
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Prune global plan to get relevant local section
   */
  nav_msgs::msg::Path pruneGlobalPlan(
    const geometry_msgs::msg::PoseStamped & pose,
    double prune_distance);

  /**
   * @brief Publish local plan for visualization
   */
  void publishLocalPlan(const Trajectory & trajectory);

  /**
   * @brief Publish all trajectories for visualization
   */
  void publishTrajectories(const std::vector<Trajectory> & trajectories);

  // Node interfaces
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_{rclcpp::get_logger("CustomLocalPlanner")};
  rclcpp::Clock::SharedPtr clock_;

  // Publishers for visualization
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr global_plan_transformed_pub_;
  
  // Plugin name
  std::string plugin_name_;

  // Global plan
  nav_msgs::msg::Path global_plan_;
  nav_msgs::msg::Path pruned_plan_;  // Pruned section of global plan for tracking

  // Parameters
  double max_linear_vel_x_;      // Max forward/backward velocity
  double min_linear_vel_x_;      // Min forward/backward velocity (negative for reverse)
  double max_linear_vel_y_;      // Max lateral (strafe) velocity
  double min_linear_vel_y_;      // Min lateral (strafe) velocity
  double max_angular_vel_;       // Max rotational velocity
  double min_angular_vel_;       // Min rotational velocity
  double linear_acc_limit_x_;    // Forward/backward acceleration limit
  double linear_acc_limit_y_;    // Lateral acceleration limit
  double angular_acc_limit_;     // Rotational acceleration limit
  
  int linear_samples_x_;         // Number of forward/backward velocity samples
  int linear_samples_y_;         // Number of lateral velocity samples
  int angular_samples_;          // Number of rotational velocity samples
  double sim_time_;              // Trajectory simulation time
  double sim_granularity_;       // Simulation time step
  
  double path_distance_weight_;
  double goal_distance_weight_;
  double obstacle_weight_;
  
  double transform_tolerance_;
  double xy_goal_tolerance_;
  double yaw_goal_tolerance_;
  double prune_plan_distance_;   // Distance to look ahead on global plan
  
  double speed_limit_ratio_;
};

}  // namespace robot_custom_local_planner

#endif  // ROBOT_CUSTOM_LOCAL_PLANNER__CUSTOM_LOCAL_PLANNER_HPP_
