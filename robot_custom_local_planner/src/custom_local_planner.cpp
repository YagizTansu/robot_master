#include "robot_custom_local_planner/custom_local_planner.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace robot_custom_local_planner
{

void CustomLocalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  node_ = parent;
  plugin_name_ = name;
  tf_buffer_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  RCLCPP_INFO(logger_, "Configuring CustomLocalPlanner for Mecanum/Omni wheels: %s", plugin_name_.c_str());

  // Declare and get parameters for holonomic robot
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_vel_x", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_linear_vel_x", rclcpp::ParameterValue(-0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_vel_y", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_linear_vel_y", rclcpp::ParameterValue(-0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_angular_vel", rclcpp::ParameterValue(-1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".linear_acc_limit_x", rclcpp::ParameterValue(2.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".linear_acc_limit_y", rclcpp::ParameterValue(2.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".angular_acc_limit", rclcpp::ParameterValue(3.2));
  
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".linear_samples_x", rclcpp::ParameterValue(5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".linear_samples_y", rclcpp::ParameterValue(5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".angular_samples", rclcpp::ParameterValue(7));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".sim_time", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".sim_granularity", rclcpp::ParameterValue(0.1));
  
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".path_distance_weight", rclcpp::ParameterValue(32.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".goal_distance_weight", rclcpp::ParameterValue(24.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".obstacle_weight", rclcpp::ParameterValue(1.0));
  
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".xy_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".prune_plan_distance", rclcpp::ParameterValue(3.0));

  node->get_parameter(plugin_name_ + ".max_linear_vel_x", max_linear_vel_x_);
  node->get_parameter(plugin_name_ + ".min_linear_vel_x", min_linear_vel_x_);
  node->get_parameter(plugin_name_ + ".max_linear_vel_y", max_linear_vel_y_);
  node->get_parameter(plugin_name_ + ".min_linear_vel_y", min_linear_vel_y_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(plugin_name_ + ".min_angular_vel", min_angular_vel_);
  node->get_parameter(plugin_name_ + ".linear_acc_limit_x", linear_acc_limit_x_);
  node->get_parameter(plugin_name_ + ".linear_acc_limit_y", linear_acc_limit_y_);
  node->get_parameter(plugin_name_ + ".angular_acc_limit", angular_acc_limit_);
  
  node->get_parameter(plugin_name_ + ".linear_samples_x", linear_samples_x_);
  node->get_parameter(plugin_name_ + ".linear_samples_y", linear_samples_y_);
  node->get_parameter(plugin_name_ + ".angular_samples", angular_samples_);
  node->get_parameter(plugin_name_ + ".sim_time", sim_time_);
  node->get_parameter(plugin_name_ + ".sim_granularity", sim_granularity_);
  
  node->get_parameter(plugin_name_ + ".path_distance_weight", path_distance_weight_);
  node->get_parameter(plugin_name_ + ".goal_distance_weight", goal_distance_weight_);
  node->get_parameter(plugin_name_ + ".obstacle_weight", obstacle_weight_);
  
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(plugin_name_ + ".xy_goal_tolerance", xy_goal_tolerance_);
  node->get_parameter(plugin_name_ + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(plugin_name_ + ".prune_plan_distance", prune_plan_distance_);

  speed_limit_ratio_ = 1.0;

  // Create publishers for visualization
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "local_plan", 1);
  global_plan_transformed_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "transformed_global_plan", 1);

  RCLCPP_INFO(logger_, "CustomLocalPlanner configured for holonomic robot successfully");
  RCLCPP_INFO(logger_, "  Max velocities - X: %.2f, Y: %.2f, Theta: %.2f", 
    max_linear_vel_x_, max_linear_vel_y_, max_angular_vel_);
  RCLCPP_INFO(logger_, "  Plan pruning distance: %.2f m", prune_plan_distance_);
  RCLCPP_INFO(logger_, "  Publishing local_plan on topic: local_plan");
}

void CustomLocalPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up CustomLocalPlanner");
  global_plan_.poses.clear();
}

void CustomLocalPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating CustomLocalPlanner");
  local_plan_pub_->on_activate();
  global_plan_transformed_pub_->on_activate();
}

void CustomLocalPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating CustomLocalPlanner");
  local_plan_pub_->on_deactivate();
  global_plan_transformed_pub_->on_deactivate();
}

void CustomLocalPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  RCLCPP_INFO(logger_, "Received new global plan with %zu poses", global_plan_.poses.size());
}

void CustomLocalPlanner::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    speed_limit_ratio_ = speed_limit / 100.0;
  } else {
    // Use max of x and y velocities for ratio calculation
    double max_vel = std::max(max_linear_vel_x_, max_linear_vel_y_);
    speed_limit_ratio_ = speed_limit / max_vel;
  }
  RCLCPP_INFO(logger_, "Speed limit set to %.2f%%", speed_limit_ratio_ * 100.0);
}

geometry_msgs::msg::TwistStamped CustomLocalPlanner::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();

  // Check if we have a valid plan
  if (global_plan_.poses.empty()) {
    RCLCPP_WARN(logger_, "No global plan available");
    return cmd_vel;
  }

  // Prune global plan to get relevant section for tracking
  pruned_plan_ = pruneGlobalPlan(pose, prune_plan_distance_);
  
  if (pruned_plan_.poses.empty()) {
    RCLCPP_WARN(logger_, "Pruned plan is empty");
    return cmd_vel;
  }

  RCLCPP_DEBUG(logger_, "Tracking %zu poses from global plan", pruned_plan_.poses.size());

  // Generate and evaluate trajectories
  auto trajectories = generateTrajectories(pose, velocity);
  
  if (trajectories.empty()) {
    RCLCPP_WARN(logger_, "No valid trajectories generated");
    return cmd_vel;
  }

  // Find best trajectory
  auto best_trajectory = std::min_element(
    trajectories.begin(), trajectories.end(),
    [](const Trajectory & a, const Trajectory & b) { return a.cost < b.cost; });

  // Publish local plan for visualization
  publishLocalPlan(*best_trajectory);
  
  // Optionally publish pruned global plan for debugging
  if (global_plan_transformed_pub_->get_subscription_count() > 0) {
    pruned_plan_.header.stamp = clock_->now();
    global_plan_transformed_pub_->publish(pruned_plan_);
  }

  // Apply speed limit to both X and Y velocities for holonomic motion
  cmd_vel.twist.linear.x = best_trajectory->linear_vel_x * speed_limit_ratio_;
  cmd_vel.twist.linear.y = best_trajectory->linear_vel_y * speed_limit_ratio_;
  cmd_vel.twist.angular.z = best_trajectory->angular_vel;

  RCLCPP_DEBUG(
    logger_, "Best trajectory - Vx: %.2f, Vy: %.2f, W: %.2f, Cost: %.2f",
    cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, 
    cmd_vel.twist.angular.z, best_trajectory->cost);

  return cmd_vel;
}

std::vector<Trajectory> CustomLocalPlanner::generateTrajectories(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  std::vector<Trajectory> trajectories;

  // Calculate velocity limits based on acceleration for holonomic motion
  double max_vel_x = std::min(
    max_linear_vel_x_,
    velocity.linear.x + linear_acc_limit_x_ * sim_granularity_);
  double min_vel_x = std::max(
    min_linear_vel_x_,
    velocity.linear.x - linear_acc_limit_x_ * sim_granularity_);
  
  double max_vel_y = std::min(
    max_linear_vel_y_,
    velocity.linear.y + linear_acc_limit_y_ * sim_granularity_);
  double min_vel_y = std::max(
    min_linear_vel_y_,
    velocity.linear.y - linear_acc_limit_y_ * sim_granularity_);
  
  double max_ang_vel = std::min(
    max_angular_vel_,
    velocity.angular.z + angular_acc_limit_ * sim_granularity_);
  double min_ang_vel = std::max(
    min_angular_vel_,
    velocity.angular.z - angular_acc_limit_ * sim_granularity_);

  // Sample velocities for holonomic robot (X, Y, Theta)
  double vel_x_step = (max_vel_x - min_vel_x) / std::max(1, linear_samples_x_ - 1);
  double vel_y_step = (max_vel_y - min_vel_y) / std::max(1, linear_samples_y_ - 1);
  double ang_vel_step = (max_ang_vel - min_ang_vel) / std::max(1, angular_samples_ - 1);

  for (int i = 0; i < linear_samples_x_; ++i) {
    double vel_x = min_vel_x + i * vel_x_step;
    
    for (int j = 0; j < linear_samples_y_; ++j) {
      double vel_y = min_vel_y + j * vel_y_step;
      
      for (int k = 0; k < angular_samples_; ++k) {
        double ang_vel = min_ang_vel + k * ang_vel_step;
        
        // Simulate holonomic trajectory
        auto trajectory = simulateTrajectory(pose, vel_x, vel_y, ang_vel);
        
        // Score trajectory
        trajectory.cost = scoreTrajectory(trajectory);
        
        // Add to list if valid (cost is not infinity)
        if (std::isfinite(trajectory.cost)) {
          trajectories.push_back(trajectory);
        }
      }
    }
  }

  return trajectories;
}

Trajectory CustomLocalPlanner::simulateTrajectory(
  const geometry_msgs::msg::PoseStamped & pose,
  double linear_vel_x,
  double linear_vel_y,
  double angular_vel)
{
  Trajectory trajectory;
  trajectory.linear_vel_x = linear_vel_x;
  trajectory.linear_vel_y = linear_vel_y;
  trajectory.angular_vel = angular_vel;
  trajectory.cost = std::numeric_limits<double>::infinity();

  // Start from current pose
  double x = pose.pose.position.x;
  double y = pose.pose.position.y;
  double theta = tf2::getYaw(pose.pose.orientation);

  // Simulate forward in time for holonomic robot
  int num_steps = static_cast<int>(sim_time_ / sim_granularity_);
  for (int i = 0; i < num_steps; ++i) {
    // Holonomic motion model
    // Transform velocities from robot frame to global frame
    double global_vel_x = linear_vel_x * cos(theta) - linear_vel_y * sin(theta);
    double global_vel_y = linear_vel_x * sin(theta) + linear_vel_y * cos(theta);
    
    x += global_vel_x * sim_granularity_;
    y += global_vel_y * sim_granularity_;
    theta += angular_vel * sim_granularity_;
    
    // Normalize theta to [-pi, pi]
    theta = atan2(sin(theta), cos(theta));

    // Store pose
    geometry_msgs::msg::PoseStamped predicted_pose;
    predicted_pose.header = pose.header;
    predicted_pose.pose.position.x = x;
    predicted_pose.pose.position.y = y;
    predicted_pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), theta));
    trajectory.poses.push_back(predicted_pose);
  }

  return trajectory;
}

double CustomLocalPlanner::scoreTrajectory(const Trajectory & trajectory)
{
  if (trajectory.poses.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  double path_score = calculatePathAlignmentScore(trajectory);
  double obstacle_score = calculateObstacleScore(trajectory);
  double goal_score = calculateGoalDistanceScore(trajectory);

  // Check if trajectory collides
  if (std::isinf(obstacle_score)) {
    return std::numeric_limits<double>::infinity();
  }

  // Combine scores
  double total_cost = 
    path_distance_weight_ * path_score +
    obstacle_weight_ * obstacle_score +
    goal_distance_weight_ * goal_score;

  return total_cost;
}

double CustomLocalPlanner::calculatePathAlignmentScore(const Trajectory & trajectory)
{
  if (pruned_plan_.poses.empty() || trajectory.poses.empty()) {
    return 1000.0;  // High penalty for invalid trajectory
  }

  // Improved scoring: weighted distance with emphasis on trajectory end
  double total_distance = 0.0;
  double total_weight = 0.0;
  
  for (size_t i = 0; i < trajectory.poses.size(); ++i) {
    const auto & traj_pose = trajectory.poses[i];
    double min_dist = std::numeric_limits<double>::max();
    
    // Find closest point on pruned plan
    for (const auto & plan_pose : pruned_plan_.poses) {
      double dx = traj_pose.pose.position.x - plan_pose.pose.position.x;
      double dy = traj_pose.pose.position.y - plan_pose.pose.position.y;
      double dist = sqrt(dx * dx + dy * dy);
      
      if (dist < min_dist) {
        min_dist = dist;
      }
    }
    
    // Weight increases linearly along trajectory (emphasize end position)
    // This makes the robot look further ahead on the path
    double weight = 1.0 + (static_cast<double>(i) / trajectory.poses.size()) * 2.0;
    total_distance += min_dist * min_dist * weight;  // Square distance for stronger penalty
    total_weight += weight;
  }

  return total_weight > 0.0 ? total_distance / total_weight : 1000.0;
}

double CustomLocalPlanner::calculateObstacleScore(const Trajectory & trajectory)
{
  double obstacle_cost = 0.0;

  for (const auto & pose : trajectory.poses) {
    unsigned int mx, my;
    if (!costmap_->worldToMap(
        pose.pose.position.x,
        pose.pose.position.y,
        mx, my))
    {
      // Outside map bounds
      return std::numeric_limits<double>::infinity();
    }

    unsigned char cost = costmap_->getCost(mx, my);
    
    // Check for collision
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      return std::numeric_limits<double>::infinity();
    }

    // Accumulate obstacle cost
    obstacle_cost += cost;
  }

  return obstacle_cost / trajectory.poses.size();
}

double CustomLocalPlanner::calculateGoalDistanceScore(const Trajectory & trajectory)
{
  if (global_plan_.poses.empty() || trajectory.poses.empty()) {
    return 0.0;
  }

  // Distance from end of trajectory to goal
  const auto & goal = global_plan_.poses.back();
  const auto & traj_end = trajectory.poses.back();

  double dx = goal.pose.position.x - traj_end.pose.position.x;
  double dy = goal.pose.position.y - traj_end.pose.position.y;

  return sqrt(dx * dx + dy * dy);
}

nav_msgs::msg::Path CustomLocalPlanner::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header.frame_id = pose.header.frame_id;
  transformed_plan.header.stamp = pose.header.stamp;

  // Transform global plan to local frame
  // For simplicity, we'll just use the plan as-is
  // In a full implementation, you'd transform each pose
  transformed_plan = global_plan_;

  return transformed_plan;
}

nav_msgs::msg::Path CustomLocalPlanner::pruneGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose,
  double prune_distance)
{
  nav_msgs::msg::Path pruned_plan;
  pruned_plan.header = global_plan_.header;
  
  if (global_plan_.poses.empty()) {
    return pruned_plan;
  }

  // Find closest point on global plan to current pose
  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  
  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    double dx = pose.pose.position.x - global_plan_.poses[i].pose.position.x;
    double dy = pose.pose.position.y - global_plan_.poses[i].pose.position.y;
    double dist = sqrt(dx * dx + dy * dy);
    
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  RCLCPP_DEBUG(logger_, "Closest point on plan at index %zu, distance: %.2f m", 
    closest_idx, min_dist);

  // Add points from closest point onwards, up to prune_distance
  double accumulated_dist = 0.0;
  pruned_plan.poses.push_back(global_plan_.poses[closest_idx]);
  
  for (size_t i = closest_idx + 1; i < global_plan_.poses.size(); ++i) {
    // Calculate distance from previous point
    const auto & prev = global_plan_.poses[i - 1];
    const auto & curr = global_plan_.poses[i];
    
    double dx = curr.pose.position.x - prev.pose.position.x;
    double dy = curr.pose.position.y - prev.pose.position.y;
    double segment_dist = sqrt(dx * dx + dy * dy);
    
    accumulated_dist += segment_dist;
    
    if (accumulated_dist > prune_distance) {
      break;
    }
    
    pruned_plan.poses.push_back(curr);
  }

  RCLCPP_DEBUG(logger_, "Pruned plan contains %zu poses covering %.2f m", 
    pruned_plan.poses.size(), accumulated_dist);

  return pruned_plan;
}

void CustomLocalPlanner::publishLocalPlan(const Trajectory & trajectory)
{
  if (local_plan_pub_->get_subscription_count() > 0 && !trajectory.poses.empty()) {
    nav_msgs::msg::Path local_plan;
    local_plan.header.frame_id = trajectory.poses[0].header.frame_id;
    local_plan.header.stamp = clock_->now();
    local_plan.poses = trajectory.poses;
    
    local_plan_pub_->publish(local_plan);
  }
}

void CustomLocalPlanner::publishTrajectories(const std::vector<Trajectory> & trajectories)
{
  // This could be used to publish all sampled trajectories for debugging
  // Not implemented in this version to reduce computational overhead
  (void)trajectories;  // Suppress unused parameter warning
}

}  // namespace robot_custom_local_planner

PLUGINLIB_EXPORT_CLASS(
  robot_custom_local_planner::CustomLocalPlanner,
  nav2_core::Controller)
