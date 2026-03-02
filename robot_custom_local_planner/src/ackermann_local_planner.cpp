/**
 * @file ackermann_local_planner.cpp
 * @brief Nav2 local planner plugin for Ackermann (car-like) robots.
 *
 * Motion model used  (bicycle / Ackermann kinematics):
 *   x(t+dt)     = x(t) + v · cos(θ) · dt
 *   y(t+dt)     = y(t) + v · sin(θ) · dt
 *   θ(t+dt)     = θ(t) + ω · dt
 *
 * where ω = v · tan(δ) / L  and  δ is the front-wheel steering angle.
 * The planner samples (v, ω) pairs inside the dynamic window, simulates
 * trajectories forward in time, scores them and picks the best one.
 */

#include "robot_custom_local_planner/ackermann_local_planner.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace robot_custom_local_planner
{

// =========================================================================
//  configure
// =========================================================================
void AckermannLocalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  node_         = parent;
  plugin_name_  = name;
  tf_buffer_    = tf;
  costmap_ros_  = costmap_ros;
  costmap_      = costmap_ros_->getCostmap();
  logger_       = node->get_logger();
  clock_        = node->get_clock();

  RCLCPP_INFO(logger_, "Configuring AckermannLocalPlanner: %s", plugin_name_.c_str());

  // ------------------------------------------------------------------
  //  Declare parameters
  // ------------------------------------------------------------------
  // Velocity limits
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_vel",   rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_linear_vel",   rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_reverse_vel",  rclcpp::ParameterValue(0.0));   // 0 = no reversing
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_vel",  rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_angular_vel",  rclcpp::ParameterValue(-0.8));

  // Acceleration limits
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".linear_acc_limit",  rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".angular_acc_limit", rclcpp::ParameterValue(1.5));

  // Ackermann geometry
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".wheelbase",         rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_steering_angle", rclcpp::ParameterValue(0.6));  // ~34 deg

  // Sampling / simulation
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".linear_samples",   rclcpp::ParameterValue(5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".angular_samples",  rclcpp::ParameterValue(11));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".sim_time",         rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".sim_granularity",  rclcpp::ParameterValue(0.1));

  // Scoring weights
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".path_distance_weight",      rclcpp::ParameterValue(32.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".goal_distance_weight",      rclcpp::ParameterValue(20.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".obstacle_weight",           rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".heading_alignment_weight",  rclcpp::ParameterValue(16.0));

  // Tolerances
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".xy_goal_tolerance",   rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".yaw_goal_tolerance",  rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".prune_plan_distance",  rclcpp::ParameterValue(3.0));

  // ------------------------------------------------------------------
  //  Read parameters
  // ------------------------------------------------------------------
  node->get_parameter(plugin_name_ + ".max_linear_vel",          max_linear_vel_);
  node->get_parameter(plugin_name_ + ".min_linear_vel",          min_linear_vel_);
  node->get_parameter(plugin_name_ + ".max_reverse_vel",         max_reverse_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_vel",         max_angular_vel_);
  node->get_parameter(plugin_name_ + ".min_angular_vel",         min_angular_vel_);

  node->get_parameter(plugin_name_ + ".linear_acc_limit",        linear_acc_limit_);
  node->get_parameter(plugin_name_ + ".angular_acc_limit",       angular_acc_limit_);

  node->get_parameter(plugin_name_ + ".wheelbase",               wheelbase_);
  node->get_parameter(plugin_name_ + ".max_steering_angle",      max_steering_angle_);

  node->get_parameter(plugin_name_ + ".linear_samples",          linear_samples_);
  node->get_parameter(plugin_name_ + ".angular_samples",         angular_samples_);
  node->get_parameter(plugin_name_ + ".sim_time",                sim_time_);
  node->get_parameter(plugin_name_ + ".sim_granularity",         sim_granularity_);

  node->get_parameter(plugin_name_ + ".path_distance_weight",    path_distance_weight_);
  node->get_parameter(plugin_name_ + ".goal_distance_weight",    goal_distance_weight_);
  node->get_parameter(plugin_name_ + ".obstacle_weight",         obstacle_weight_);
  node->get_parameter(plugin_name_ + ".heading_alignment_weight", heading_alignment_weight_);

  node->get_parameter(plugin_name_ + ".transform_tolerance",     transform_tolerance_);
  node->get_parameter(plugin_name_ + ".xy_goal_tolerance",       xy_goal_tolerance_);
  node->get_parameter(plugin_name_ + ".yaw_goal_tolerance",      yaw_goal_tolerance_);
  node->get_parameter(plugin_name_ + ".prune_plan_distance",     prune_plan_distance_);

  speed_limit_ratio_ = 1.0;

  // ------------------------------------------------------------------
  //  Publishers
  // ------------------------------------------------------------------
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  global_plan_transformed_pub_ =
    node->create_publisher<nav_msgs::msg::Path>("transformed_global_plan", 1);

  // Sanity-check: enforce max_angular_vel against physical steering limits.
  // ω_max from geometry = v_max · tan(δ_max) / L
  double omega_max_geometry = max_linear_vel_ * std::tan(max_steering_angle_) / wheelbase_;
  if (max_angular_vel_ > omega_max_geometry) {
    RCLCPP_WARN(
      logger_,
      "max_angular_vel (%.3f rad/s) exceeds what Ackermann geometry allows (%.3f rad/s) "
      "at max speed. Clamping to geometry limit.",
      max_angular_vel_, omega_max_geometry);
    max_angular_vel_  =  omega_max_geometry;
    min_angular_vel_  = -omega_max_geometry;
  }

  RCLCPP_INFO(logger_, "AckermannLocalPlanner configured successfully.");
  RCLCPP_INFO(logger_,
    "  Velocity  – fwd: [%.2f, %.2f] m/s, rev: %.2f m/s, ω: [%.2f, %.2f] rad/s",
    min_linear_vel_, max_linear_vel_, max_reverse_vel_,
    min_angular_vel_, max_angular_vel_);
  RCLCPP_INFO(logger_,
    "  Geometry  – wheelbase: %.3f m, max_steer: %.2f rad (%.1f°)",
    wheelbase_, max_steering_angle_, max_steering_angle_ * 180.0 / M_PI);
}

// =========================================================================
//  lifecycle callbacks
// =========================================================================
void AckermannLocalPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up AckermannLocalPlanner");
  global_plan_.poses.clear();
  pruned_plan_.poses.clear();
}

void AckermannLocalPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating AckermannLocalPlanner");
  local_plan_pub_->on_activate();
  global_plan_transformed_pub_->on_activate();
}

void AckermannLocalPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating AckermannLocalPlanner");
  local_plan_pub_->on_deactivate();
  global_plan_transformed_pub_->on_deactivate();
}

// =========================================================================
//  setPlan / setSpeedLimit
// =========================================================================
void AckermannLocalPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  RCLCPP_INFO(logger_, "Received new global plan with %zu poses", global_plan_.poses.size());
}

void AckermannLocalPlanner::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    speed_limit_ratio_ = speed_limit / 100.0;
  } else {
    speed_limit_ratio_ = speed_limit / max_linear_vel_;
  }
  speed_limit_ratio_ = std::clamp(speed_limit_ratio_, 0.0, 1.0);
  RCLCPP_INFO(logger_, "Speed limit set to %.1f%%", speed_limit_ratio_ * 100.0);
}

// =========================================================================
//  computeVelocityCommands
// =========================================================================
geometry_msgs::msg::TwistStamped AckermannLocalPlanner::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp    = clock_->now();

  if (global_plan_.poses.empty()) {
    RCLCPP_WARN(logger_, "No global plan available");
    return cmd_vel;
  }

  // Prune the global plan to a local window
  pruned_plan_ = pruneGlobalPlan(pose, prune_plan_distance_);
  if (pruned_plan_.poses.empty()) {
    RCLCPP_WARN(logger_, "Pruned plan is empty");
    return cmd_vel;
  }

  RCLCPP_DEBUG(logger_, "Tracking %zu poses from global plan", pruned_plan_.poses.size());

  // Generate and score trajectories
  auto trajectories = generateTrajectories(pose, velocity);

  if (trajectories.empty()) {
    RCLCPP_WARN(logger_, "No valid trajectories generated — stopping");
    return cmd_vel;  // zero velocity (stop safely)
  }

  // Select lowest-cost trajectory
  auto best = std::min_element(
    trajectories.begin(), trajectories.end(),
    [](const AckermannTrajectory & a, const AckermannTrajectory & b) {
      return a.cost < b.cost;
    });

  // Publish for visualisation
  publishLocalPlan(*best);
  if (global_plan_transformed_pub_->get_subscription_count() > 0) {
    pruned_plan_.header.stamp = clock_->now();
    global_plan_transformed_pub_->publish(pruned_plan_);
  }

  // Apply speed limit — scale linear velocity only; angular follows
  double scaled_v = best->linear_vel * speed_limit_ratio_;

  // Recompute omega to keep the same curvature at the new speed
  // ω_new = scaled_v * tan(δ) / L  = (scaled_v / v) * ω_orig
  double scaled_omega = (std::abs(best->linear_vel) > 1e-6)
    ? best->angular_vel * (scaled_v / best->linear_vel)
    : best->angular_vel;

  cmd_vel.twist.linear.x  = scaled_v;
  cmd_vel.twist.linear.y  = 0.0;   // Ackermann robots have NO lateral velocity
  cmd_vel.twist.angular.z = scaled_omega;

  RCLCPP_DEBUG(
    logger_, "Best: v=%.3f m/s, ω=%.3f rad/s, δ=%.2f°, cost=%.3f",
    scaled_v, scaled_omega,
    best->steering_angle * 180.0 / M_PI, best->cost);

  return cmd_vel;
}

// =========================================================================
//  generateTrajectories
// =========================================================================
std::vector<AckermannTrajectory> AckermannLocalPlanner::generateTrajectories(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  std::vector<AckermannTrajectory> trajectories;

  // ------------------------------------------------------------------
  //  Dynamic Window: narrow the reachable (v, ω) set by acceleration
  // ------------------------------------------------------------------
  double cur_v = velocity.linear.x;
  double cur_omega = velocity.angular.z;

  // Forward velocity window
  double dv = linear_acc_limit_ * sim_granularity_;
  double v_min = std::max(-max_reverse_vel_, cur_v - dv);
  double v_max = std::min(max_linear_vel_,   cur_v + dv);
  // Enforce min_linear_vel if moving forward
  // (allow deceleration through zero)
  v_min = std::max(v_min, -max_reverse_vel_);
  v_max = std::min(v_max,  max_linear_vel_);

  // Angular velocity window
  double domega = angular_acc_limit_ * sim_granularity_;
  double omega_min = std::max(min_angular_vel_, cur_omega - domega);
  double omega_max = std::min(max_angular_vel_, cur_omega + domega);

  if (linear_samples_ < 2) {
    // degenerate case: just use v_max
    v_min = v_max;
  }
  if (angular_samples_ < 2) {
    omega_min = omega_max;
  }

  double v_step     = (linear_samples_ <= 1) ? 0.0 : (v_max - v_min) / (linear_samples_ - 1);
  double omega_step = (angular_samples_ <= 1) ? 0.0 : (omega_max - omega_min) / (angular_samples_ - 1);

  for (int i = 0; i < linear_samples_; ++i) {
    double v = v_min + i * v_step;

    for (int j = 0; j < angular_samples_; ++j) {
      double omega = omega_min + j * omega_step;

      // ------------------------------------------------------------------
      //  Ackermann constraint: reject (v, ω) that imply a steering angle
      //  beyond the physical limit — δ = atan(L · ω / v)
      // ------------------------------------------------------------------
      double steering_angle = 0.0;
      if (std::abs(v) > 1e-4) {
        steering_angle = std::atan(wheelbase_ * omega / v);
        if (std::abs(steering_angle) > max_steering_angle_) {
          continue;  // mechanically unreachable — skip
        }
      } else {
        // Near-zero speed: allow any ω (in-place turn is not possible for
        // Ackermann, but we keep small ω to avoid halting the robot);
        // clip to what the steering can produce at min_linear_vel.
        double omega_at_min_v =
          min_linear_vel_ * std::tan(max_steering_angle_) / wheelbase_;
        if (std::abs(omega) > omega_at_min_v + 1e-6) {
          continue;
        }
      }

      // Simulate forward
      auto traj = simulateTrajectory(pose, v, omega);
      traj.steering_angle = steering_angle;
      traj.cost = scoreTrajectory(traj);

      if (std::isfinite(traj.cost)) {
        trajectories.push_back(traj);
      }
    }
  }

  return trajectories;
}

// =========================================================================
//  simulateTrajectory  (bicycle kinematic model)
// =========================================================================
AckermannTrajectory AckermannLocalPlanner::simulateTrajectory(
  const geometry_msgs::msg::PoseStamped & pose,
  double linear_vel,
  double angular_vel)
{
  AckermannTrajectory traj;
  traj.linear_vel   = linear_vel;
  traj.angular_vel  = angular_vel;
  traj.steering_angle = 0.0;
  traj.cost = std::numeric_limits<double>::infinity();

  double x     = pose.pose.position.x;
  double y     = pose.pose.position.y;
  double theta = tf2::getYaw(pose.pose.orientation);

  int num_steps = static_cast<int>(sim_time_ / sim_granularity_);

  for (int step = 0; step < num_steps; ++step) {
    // Bicycle kinematic model (forward Euler)
    x     += linear_vel  * std::cos(theta) * sim_granularity_;
    y     += linear_vel  * std::sin(theta) * sim_granularity_;
    theta += angular_vel * sim_granularity_;

    // Normalise theta to [-π, π]
    theta = std::atan2(std::sin(theta), std::cos(theta));

    geometry_msgs::msg::PoseStamped p;
    p.header = pose.header;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.orientation =
      tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), theta));

    traj.poses.push_back(p);
  }

  return traj;
}

// =========================================================================
//  scoreTrajectory
// =========================================================================
double AckermannLocalPlanner::scoreTrajectory(const AckermannTrajectory & traj)
{
  if (traj.poses.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  double obstacle_score = calculateObstacleScore(traj);
  if (std::isinf(obstacle_score)) {
    return std::numeric_limits<double>::infinity();  // collision — discard
  }

  double path_score    = calculatePathAlignmentScore(traj);
  double goal_score    = calculateGoalDistanceScore(traj);
  double heading_score = calculateHeadingAlignmentScore(traj);

  return path_distance_weight_      * path_score
       + goal_distance_weight_      * goal_score
       + obstacle_weight_           * obstacle_score
       + heading_alignment_weight_  * heading_score;
}

// =========================================================================
//  calculatePathAlignmentScore
//  (same weighted squared-distance formula as holonomic planner)
// =========================================================================
double AckermannLocalPlanner::calculatePathAlignmentScore(const AckermannTrajectory & traj)
{
  if (pruned_plan_.poses.empty() || traj.poses.empty()) {
    return 1000.0;
  }

  double total_distance = 0.0;
  double total_weight   = 0.0;
  size_t n = traj.poses.size();

  for (size_t i = 0; i < n; ++i) {
    const auto & tp = traj.poses[i];
    double min_dist = std::numeric_limits<double>::max();

    for (const auto & pp : pruned_plan_.poses) {
      double dx = tp.pose.position.x - pp.pose.position.x;
      double dy = tp.pose.position.y - pp.pose.position.y;
      double d  = std::sqrt(dx * dx + dy * dy);
      min_dist  = std::min(min_dist, d);
    }

    // Weight emphasises the end of the trajectory
    double w = 1.0 + (static_cast<double>(i) / n) * 2.0;
    total_distance += min_dist * min_dist * w;
    total_weight   += w;
  }

  return (total_weight > 0.0) ? (total_distance / total_weight) : 1000.0;
}

// =========================================================================
//  calculateObstacleScore
// =========================================================================
double AckermannLocalPlanner::calculateObstacleScore(const AckermannTrajectory & traj)
{
  double obstacle_cost = 0.0;

  for (const auto & p : traj.poses) {
    unsigned int mx, my;
    if (!costmap_->worldToMap(p.pose.position.x, p.pose.position.y, mx, my)) {
      return std::numeric_limits<double>::infinity();  // outside map
    }

    unsigned char cost = costmap_->getCost(mx, my);
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      return std::numeric_limits<double>::infinity();  // collision
    }

    obstacle_cost += static_cast<double>(cost);
  }

  return obstacle_cost / static_cast<double>(traj.poses.size());
}

// =========================================================================
//  calculateGoalDistanceScore
// =========================================================================
double AckermannLocalPlanner::calculateGoalDistanceScore(const AckermannTrajectory & traj)
{
  if (global_plan_.poses.empty() || traj.poses.empty()) {
    return 0.0;
  }

  const auto & goal = global_plan_.poses.back();
  const auto & end  = traj.poses.back();

  double dx = goal.pose.position.x - end.pose.position.x;
  double dy = goal.pose.position.y - end.pose.position.y;

  return std::sqrt(dx * dx + dy * dy);
}

// =========================================================================
//  calculateHeadingAlignmentScore
//  (unique to Ackermann — penalise heading mismatch with the path)
// =========================================================================
double AckermannLocalPlanner::calculateHeadingAlignmentScore(const AckermannTrajectory & traj)
{
  if (pruned_plan_.poses.size() < 2 || traj.poses.empty()) {
    return 0.0;
  }

  // Desired heading: direction from second-to-last to last pose on pruned plan
  const auto & p1 = pruned_plan_.poses[pruned_plan_.poses.size() - 2];
  const auto & p2 = pruned_plan_.poses[pruned_plan_.poses.size() - 1];

  double desired_yaw = std::atan2(
    p2.pose.position.y - p1.pose.position.y,
    p2.pose.position.x - p1.pose.position.x);

  // Actual heading at end of simulated trajectory
  double actual_yaw = tf2::getYaw(traj.poses.back().pose.orientation);

  double diff = std::atan2(std::sin(desired_yaw - actual_yaw),
                            std::cos(desired_yaw - actual_yaw));

  // Return absolute angular error (in radians) — lower is better
  return std::abs(diff);
}

// =========================================================================
//  pruneGlobalPlan
// =========================================================================
nav_msgs::msg::Path AckermannLocalPlanner::pruneGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose,
  double prune_distance)
{
  nav_msgs::msg::Path pruned;
  pruned.header = global_plan_.header;

  if (global_plan_.poses.empty()) {
    return pruned;
  }

  // Find the closest point on the global plan
  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    double dx = pose.pose.position.x - global_plan_.poses[i].pose.position.x;
    double dy = pose.pose.position.y - global_plan_.poses[i].pose.position.y;
    double d  = std::sqrt(dx * dx + dy * dy);
    if (d < min_dist) {
      min_dist    = d;
      closest_idx = i;
    }
  }

  RCLCPP_DEBUG(logger_, "Closest plan point: idx=%zu, dist=%.3f m", closest_idx, min_dist);

  // Walk forward up to prune_distance
  double accumulated = 0.0;
  pruned.poses.push_back(global_plan_.poses[closest_idx]);

  for (size_t i = closest_idx + 1; i < global_plan_.poses.size(); ++i) {
    const auto & prev = global_plan_.poses[i - 1];
    const auto & curr = global_plan_.poses[i];
    double dx = curr.pose.position.x - prev.pose.position.x;
    double dy = curr.pose.position.y - prev.pose.position.y;
    accumulated += std::sqrt(dx * dx + dy * dy);

    if (accumulated > prune_distance) {
      break;
    }
    pruned.poses.push_back(curr);
  }

  RCLCPP_DEBUG(logger_, "Pruned plan: %zu poses, %.2f m", pruned.poses.size(), accumulated);
  return pruned;
}

// =========================================================================
//  publishLocalPlan
// =========================================================================
void AckermannLocalPlanner::publishLocalPlan(const AckermannTrajectory & traj)
{
  if (local_plan_pub_->get_subscription_count() > 0 && !traj.poses.empty()) {
    nav_msgs::msg::Path local_plan;
    local_plan.header.frame_id = traj.poses.front().header.frame_id;
    local_plan.header.stamp    = clock_->now();
    local_plan.poses           = traj.poses;
    local_plan_pub_->publish(local_plan);
  }
}

}  // namespace robot_custom_local_planner

PLUGINLIB_EXPORT_CLASS(
  robot_custom_local_planner::AckermannLocalPlanner,
  nav2_core::Controller)
