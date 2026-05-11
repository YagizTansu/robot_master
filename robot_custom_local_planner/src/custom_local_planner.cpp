#include "robot_custom_local_planner/custom_local_planner.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>
#include <algorithm>

namespace robot_custom_local_planner
{

// ═══════════════════════════════════════════════════════════════════════════
// configure
// ═══════════════════════════════════════════════════════════════════════════

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

  node_        = parent;
  plugin_name_ = name;
  tf_buffer_   = tf;
  costmap_ros_ = costmap_ros;
  costmap_     = costmap_ros_->getCostmap();
  logger_      = node->get_logger();
  clock_       = node->get_clock();

  // ── Ackermann geometry ───────────────────────────────────────────────────
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".wheelbase", rclcpp::ParameterValue(1.0957));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_steering_angle", rclcpp::ParameterValue(1.5708));

  // ── Velocity limits ──────────────────────────────────────────────────────
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_vel", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_linear_vel", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_reverse_vel", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(0.8));

  // ── Acceleration / rate limits ───────────────────────────────────────────
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".linear_acc_limit", rclcpp::ParameterValue(0.545));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".steering_slew_rate", rclcpp::ParameterValue(1.5));

  // ── Sampling ─────────────────────────────────────────────────────────────
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".linear_samples", rclcpp::ParameterValue(7));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".steering_samples", rclcpp::ParameterValue(11));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_samples", rclcpp::ParameterValue(5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_max_vel", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".heading_error_trigger", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".sim_time", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".sim_granularity", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".control_duration", rclcpp::ParameterValue(0.5));

  // ── Cost weights ─────────────────────────────────────────────────────────
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".path_distance_weight", rclcpp::ParameterValue(32.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".heading_alignment_weight", rclcpp::ParameterValue(16.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".goal_distance_weight", rclcpp::ParameterValue(20.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".obstacle_weight", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".velocity_reward_weight", rclcpp::ParameterValue(5.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".steering_penalty_weight", rclcpp::ParameterValue(2.0));

  // ── Tolerances ───────────────────────────────────────────────────────────
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".xy_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".prune_plan_distance", rclcpp::ParameterValue(4.0));

  // ── Read all parameters ──────────────────────────────────────────────────
  node->get_parameter(plugin_name_ + ".wheelbase",            wheelbase_);
  node->get_parameter(plugin_name_ + ".max_steering_angle",   max_steering_angle_);
  node->get_parameter(plugin_name_ + ".max_linear_vel",       max_linear_vel_);
  node->get_parameter(plugin_name_ + ".min_linear_vel",       min_linear_vel_);
  node->get_parameter(plugin_name_ + ".max_reverse_vel",      max_reverse_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_vel",      max_angular_vel_);
  node->get_parameter(plugin_name_ + ".linear_acc_limit",     linear_acc_limit_);
  node->get_parameter(plugin_name_ + ".steering_slew_rate",   steering_slew_rate_);
  node->get_parameter(plugin_name_ + ".linear_samples",       linear_samples_);
  node->get_parameter(plugin_name_ + ".steering_samples",     steering_samples_);
  node->get_parameter(plugin_name_ + ".rotate_samples",       rotate_samples_);
  node->get_parameter(plugin_name_ + ".rotate_max_vel",       rotate_max_vel_);
  node->get_parameter(plugin_name_ + ".heading_error_trigger", heading_error_trigger_);
  node->get_parameter(plugin_name_ + ".sim_time",             sim_time_);
  node->get_parameter(plugin_name_ + ".sim_granularity",      sim_granularity_);
  node->get_parameter(plugin_name_ + ".control_duration",     control_dt_);
  node->get_parameter(plugin_name_ + ".path_distance_weight",    path_distance_weight_);
  node->get_parameter(plugin_name_ + ".heading_alignment_weight", heading_alignment_weight_);
  node->get_parameter(plugin_name_ + ".goal_distance_weight",    goal_distance_weight_);
  node->get_parameter(plugin_name_ + ".obstacle_weight",         obstacle_weight_);
  node->get_parameter(plugin_name_ + ".velocity_reward_weight",  velocity_reward_weight_);
  node->get_parameter(plugin_name_ + ".steering_penalty_weight", steering_penalty_weight_);
  node->get_parameter(plugin_name_ + ".transform_tolerance",  transform_tolerance_);
  node->get_parameter(plugin_name_ + ".xy_goal_tolerance",    xy_goal_tolerance_);
  node->get_parameter(plugin_name_ + ".yaw_goal_tolerance",   yaw_goal_tolerance_);
  node->get_parameter(plugin_name_ + ".prune_plan_distance",  prune_plan_distance_);


  speed_limit_ratio_      = 1.0;
  last_closest_idx_       = 0;
  current_steering_angle_ = 0.0;

  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  global_plan_transformed_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "transformed_global_plan", 1);

  // ── Dynamic reconfigure callback ─────────────────────────────────────────
  // Allows live tuning via rqt (Plugins → Configuration → Parameter Reconfigure)
  // or:  ros2 param set /controller_server FollowPath.path_distance_weight 120.0
  param_callback_handle_ = node->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params)
    -> rcl_interfaces::msg::SetParametersResult
    {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;

      for (const auto & p : params) {
        const std::string & n = p.get_name();
        // Only handle parameters belonging to this plugin instance
        if (n.rfind(plugin_name_ + ".", 0) != 0) { continue; }
        const std::string key = n.substr(plugin_name_.size() + 1);

        try {
          if      (key == "path_distance_weight")     path_distance_weight_     = p.as_double();
          else if (key == "heading_alignment_weight") heading_alignment_weight_ = p.as_double();
          else if (key == "goal_distance_weight")     goal_distance_weight_     = p.as_double();
          else if (key == "obstacle_weight")          obstacle_weight_          = p.as_double();
          else if (key == "velocity_reward_weight")   velocity_reward_weight_   = p.as_double();
          else if (key == "steering_penalty_weight")  steering_penalty_weight_  = p.as_double();
          else if (key == "max_linear_vel")           max_linear_vel_           = p.as_double();
          else if (key == "min_linear_vel")           min_linear_vel_           = p.as_double();
          else if (key == "max_angular_vel")          max_angular_vel_          = p.as_double();
          else if (key == "rotate_max_vel")           rotate_max_vel_           = p.as_double();
          else if (key == "heading_error_trigger")    heading_error_trigger_    = p.as_double();
          else if (key == "prune_plan_distance")      prune_plan_distance_      = p.as_double();
          else if (key == "sim_time")                 sim_time_                 = p.as_double();
          else if (key == "linear_acc_limit")         linear_acc_limit_         = p.as_double();
          else if (key == "steering_slew_rate")       steering_slew_rate_       = p.as_double();
          else if (key == "linear_samples")           linear_samples_           = p.as_int();
          else if (key == "steering_samples")         steering_samples_         = p.as_int();
          else if (key == "rotate_samples")           rotate_samples_           = p.as_int();
          else { continue; }  // unknown key — ignore silently

          RCLCPP_INFO(logger_, "[DynParam] %s = %s", n.c_str(), p.value_to_string().c_str());
        } catch (const std::exception & e) {
          result.successful = false;
          result.reason = e.what();
        }
      }
      return result;
    });

  RCLCPP_INFO(logger_,
    "AckermannLocalPlanner configured — "
    "wheelbase=%.4f m, max_steer=%.2f rad (%.1f°), "
    "v=[%.2f..%.2f] m/s, rev=%.2f m/s",
    wheelbase_, max_steering_angle_, max_steering_angle_ * 180.0 / M_PI,
    min_linear_vel_, max_linear_vel_, max_reverse_vel_);
  RCLCPP_INFO(logger_,
    "  Sampling: %d linear × %d steering + %d rotate-in-place = %d max trajectories",
    linear_samples_, steering_samples_, rotate_samples_,
    linear_samples_ * steering_samples_ + rotate_samples_);
}

// ═══════════════════════════════════════════════════════════════════════════
// Lifecycle
// ═══════════════════════════════════════════════════════════════════════════

void CustomLocalPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up AckermannLocalPlanner");
  global_plan_.poses.clear();
}

void CustomLocalPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating AckermannLocalPlanner");
  local_plan_pub_->on_activate();
  global_plan_transformed_pub_->on_activate();
}

void CustomLocalPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating AckermannLocalPlanner");
  local_plan_pub_->on_deactivate();
  global_plan_transformed_pub_->on_deactivate();
}

void CustomLocalPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_      = path;
  last_closest_idx_ = 0;
  RCLCPP_INFO(logger_, "Received new global plan with %zu poses", global_plan_.poses.size());
}

void CustomLocalPlanner::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    speed_limit_ratio_ = speed_limit / 100.0;
  } else {
    speed_limit_ratio_ = (max_linear_vel_ > 1e-6) ? speed_limit / max_linear_vel_ : 1.0;
  }
  RCLCPP_INFO(logger_, "Speed limit set to %.1f%%", speed_limit_ratio_ * 100.0);
}

// ═══════════════════════════════════════════════════════════════════════════
// computeVelocityCommands
// ═══════════════════════════════════════════════════════════════════════════

geometry_msgs::msg::TwistStamped CustomLocalPlanner::computeVelocityCommands(
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

  // Update current steering estimate from velocity feedback
  current_steering_angle_ = estimateCurrentSteering(velocity);

  // Build local plan window
  pruned_plan_ = pruneGlobalPlan(pose, prune_plan_distance_);
  if (pruned_plan_.poses.empty()) {
    RCLCPP_WARN(logger_, "Pruned plan is empty");
    return cmd_vel;
  }

  // Publish pruned plan for debugging
  if (global_plan_transformed_pub_->get_subscription_count() > 0) {
    pruned_plan_.header.stamp = clock_->now();
    global_plan_transformed_pub_->publish(pruned_plan_);
  }

  // Sample & evaluate trajectories
  auto trajectories = generateTrajectories(pose, velocity);
  if (trajectories.empty()) {
    RCLCPP_WARN(logger_, "No valid trajectories generated");
    return cmd_vel;
  }

  // Pick minimum-cost trajectory
  const auto & best = *std::min_element(
    trajectories.begin(), trajectories.end(),
    [](const Trajectory & a, const Trajectory & b) { return a.cost < b.cost; });

  publishLocalPlan(best);

  cmd_vel.twist.linear.x  = best.linear_vel  * speed_limit_ratio_;
  // Clamp angular velocity to max_angular_vel_ (applies to both Ackermann and rotate-in-place)
  double raw_wz = best.angular_vel * speed_limit_ratio_;
  cmd_vel.twist.angular.z = std::max(-max_angular_vel_, std::min(max_angular_vel_, raw_wz));

  RCLCPP_DEBUG(logger_,
    "Best: v=%.3f m/s  δ=%.1f°  ω=%.3f rad/s  cost=%.2f  rotate=%d",
    best.linear_vel, best.steering_angle * 180.0 / M_PI,
    best.angular_vel, best.cost, static_cast<int>(best.rotate_in_place));

  return cmd_vel;
}

// ═══════════════════════════════════════════════════════════════════════════
// estimateCurrentSteering
//
// Reconstruct δ from (vx, wz) using the Ackermann relation:
//   ω = v·tan(δ)/L  →  δ = atan(ω·L / v)
// When the robot is nearly stopped, keep the last known value.
// ═══════════════════════════════════════════════════════════════════════════

double CustomLocalPlanner::estimateCurrentSteering(
  const geometry_msgs::msg::Twist & velocity) const
{
  if (std::abs(velocity.linear.x) > 0.05) {
    double delta = std::atan2(velocity.angular.z * wheelbase_, velocity.linear.x);
    return std::max(-max_steering_angle_, std::min(max_steering_angle_, delta));
  }
  return current_steering_angle_;
}

// ═══════════════════════════════════════════════════════════════════════════
// generateTrajectories
//
// Samples the physically reachable (v, δ) space:
//   v window   : current_v ± linear_acc_limit * control_dt
//   δ window   : current_δ ± steering_slew_rate * control_dt   (slew constraint)
//             intersected with ±max_steering_angle               (hardware limit)
//
// Additionally, when the heading error to the plan is large, rotate-in-place
// candidates (v=0, direct wz) are added so the robot can realign before driving.
// ═══════════════════════════════════════════════════════════════════════════

std::vector<Trajectory> CustomLocalPlanner::generateTrajectories(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  std::vector<Trajectory> trajectories;

  // ── Reachable velocity window ────────────────────────────────────────────
  double v_cur       = velocity.linear.x;
  double v_max_reach = std::min( max_linear_vel_,  v_cur + linear_acc_limit_ * control_dt_);
  double v_min_reach = std::max(-max_reverse_vel_, v_cur - linear_acc_limit_ * control_dt_);
  if (max_reverse_vel_ < 1e-6) {
    // Forward-only: never sample negative speeds
    v_min_reach = std::max(0.0, v_min_reach);
  }

  // v=0 in the (v,δ) grid gives ω = v·tan(δ)/L = 0 for EVERY delta → cmd_vel=(0,0).
  // That zero-motion trajectory scores best at corners (path_distance≈0 while
  // forward samples diverge from the turning path), causing the robot to freeze.
  // Floor v_min to min_linear_vel_ so the forward grid always produces motion.
  if (v_min_reach < min_linear_vel_ && v_max_reach >= min_linear_vel_) {
    v_min_reach = min_linear_vel_;
  }

  // ── Reachable steering window (slew-rate constrained) ───────────────────
  double d_cur       = current_steering_angle_;
  double d_max_reach = std::min( max_steering_angle_, d_cur + steering_slew_rate_ * control_dt_);
  double d_min_reach = std::max(-max_steering_angle_, d_cur - steering_slew_rate_ * control_dt_);

  // ── Sample (v, δ) grid ───────────────────────────────────────────────────
  double v_step = (linear_samples_ > 1)
    ? (v_max_reach - v_min_reach) / (linear_samples_ - 1) : 0.0;
  double d_step = (steering_samples_ > 1)
    ? (d_max_reach - d_min_reach) / (steering_samples_ - 1) : 0.0;

  for (int i = 0; i < linear_samples_; ++i) {
    double v = v_min_reach + i * v_step;

    for (int j = 0; j < steering_samples_; ++j) {
      double delta = d_min_reach + j * d_step;

      auto traj  = simulateTrajectory(pose, v, delta);
      traj.cost  = scoreTrajectory(traj);
      if (std::isfinite(traj.cost)) {
        trajectories.push_back(traj);
      }
    }
  }

  // ── Rotate-in-place candidates ───────────────────────────────────────────
  // Triggered when:
  //   (a) heading error to the carrot (end of pruned window) exceeds the
  //       threshold — uses carrot direction instead of the near-tangent so
  //       upcoming corners are anticipated before the robot arrives at them.
  //   (b) ALL forward trajectories are blocked (obstacle inf-cost) — fallback
  //       that prevents the robot from freezing with cmd_vel=0,0.
  //
  // When v=0, Ackermann gives ω = v·tan(δ)/L = 0 for every delta sample, so
  // rotation is impossible through the normal (v,δ) grid.  Rotate-in-place
  // samples are the only way to produce wz≠0 from a standstill.
  double heading_error = 0.0;
  if (!pruned_plan_.poses.empty()) {
    const auto & carrot = pruned_plan_.poses.back();
    double cdx = carrot.pose.position.x - pose.pose.position.x;
    double cdy = carrot.pose.position.y - pose.pose.position.y;
    double dist = std::sqrt(cdx * cdx + cdy * cdy);
    if (dist > 0.3) {  // meaningful direction only when carrot is not on top of robot
      double carrot_hdg = std::atan2(cdy, cdx);
      double robot_hdg  = tf2::getYaw(pose.pose.orientation);
      heading_error = std::fabs(std::atan2(
        std::sin(carrot_hdg - robot_hdg),
        std::cos(carrot_hdg - robot_hdg)));
    } else {
      // Carrot is on top of robot (end of pruned window == robot position).
      // Scan the global plan ahead for the first point ≥1m away to get a
      // meaningful look-ahead heading — critical at tight corners where the
      // pruned window shrinks to a single near-by pose.
      double robot_hdg = tf2::getYaw(pose.pose.orientation);
      bool found_lookahead = false;
      for (size_t i = last_closest_idx_; i < global_plan_.poses.size(); ++i) {
        double fdx = global_plan_.poses[i].pose.position.x - pose.pose.position.x;
        double fdy = global_plan_.poses[i].pose.position.y - pose.pose.position.y;
        if (std::sqrt(fdx * fdx + fdy * fdy) >= 1.0) {
          double ahead_hdg = std::atan2(fdy, fdx);
          heading_error = std::fabs(std::atan2(
            std::sin(ahead_hdg - robot_hdg),
            std::cos(ahead_hdg - robot_hdg)));
          found_lookahead = true;
          break;
        }
      }
      // At end of global plan: no point ≥1m ahead exists.
      // Without this fallback, heading_error stays 0.0 → rotate-in-place is
      // never triggered → robot is forced to drive forward (min_linear_vel
      // floor) → small circles near the goal instead of spinning in place.
      if (!found_lookahead && !global_plan_.poses.empty()) {
        double goal_yaw = tf2::getYaw(global_plan_.poses.back().pose.orientation);
        heading_error = std::fabs(std::atan2(
          std::sin(goal_yaw - robot_hdg),
          std::cos(goal_yaw - robot_hdg)));
      }
    }
  }

  bool all_forward_blocked = trajectories.empty();
  if (heading_error > heading_error_trigger_ || all_forward_blocked) {
    if (all_forward_blocked) {
      RCLCPP_WARN(logger_, "All forward trajectories blocked — forcing rotate-in-place");
    }
    double rot_step = (rotate_samples_ > 1)
      ? (2.0 * rotate_max_vel_) / (rotate_samples_ - 1) : 0.0;

    for (int k = 0; k < rotate_samples_; ++k) {
      double wz = -rotate_max_vel_ + k * rot_step;
      if (std::abs(wz) < 1e-6) { continue; }

      auto traj  = simulateTrajectory(pose, 0.0, 0.0, true, wz);
      traj.cost  = scoreTrajectory(traj);
      if (std::isfinite(traj.cost)) {
        trajectories.push_back(traj);
      }
    }
  }

  return trajectories;
}

// ═══════════════════════════════════════════════════════════════════════════
// simulateTrajectory  —  Tricycle (bicycle) kinematic model
//
//   Normal:           ω = v·tan(δ)/L
//                     ẋ = v·cos(θ),  ẏ = v·sin(θ),  θ̇ = ω
//
//   Rotate-in-place:  ẋ = 0,  ẏ = 0,  θ̇ = rotate_angular_vel
//                     (motor controller handles δ→±90° internally)
// ═══════════════════════════════════════════════════════════════════════════

Trajectory CustomLocalPlanner::simulateTrajectory(
  const geometry_msgs::msg::PoseStamped & pose,
  double linear_vel,
  double steering_angle,
  bool   rotate_in_place,
  double rotate_angular_vel)
{
  Trajectory traj;
  traj.linear_vel      = linear_vel;
  traj.steering_angle  = steering_angle;
  traj.rotate_in_place = rotate_in_place;
  traj.cost            = std::numeric_limits<double>::infinity();

  // Derive angular_vel (= cmd_vel.angular.z output)
  if (rotate_in_place) {
    traj.angular_vel = rotate_angular_vel;
  } else {
    // ω = v·tan(δ)/L  —  identical formula to kinco_bridge.py compute_commands()
    traj.angular_vel = (std::abs(linear_vel) > 1e-4)
      ? (linear_vel * std::tan(steering_angle)) / wheelbase_
      : 0.0;
  }

  double x     = pose.pose.position.x;
  double y     = pose.pose.position.y;
  double theta = tf2::getYaw(pose.pose.orientation);

  const int num_steps = static_cast<int>(sim_time_ / sim_granularity_);
  traj.poses.reserve(num_steps);

  for (int i = 0; i < num_steps; ++i) {
    if (rotate_in_place) {
      // Position unchanged; only heading integrates
      theta += rotate_angular_vel * sim_granularity_;
    } else {
      x     += linear_vel       * std::cos(theta) * sim_granularity_;
      y     += linear_vel       * std::sin(theta) * sim_granularity_;
      theta += traj.angular_vel                   * sim_granularity_;
    }

    // Normalise θ to [-π, π]
    theta = std::atan2(std::sin(theta), std::cos(theta));

    geometry_msgs::msg::PoseStamped p;
    p.header             = pose.header;
    p.pose.position.x    = x;
    p.pose.position.y    = y;
    p.pose.orientation   = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), theta));
    traj.poses.push_back(p);
  }

  return traj;
}

// ═══════════════════════════════════════════════════════════════════════════
// scoreTrajectory
// ═══════════════════════════════════════════════════════════════════════════

double CustomLocalPlanner::scoreTrajectory(const Trajectory & traj)
{
  if (traj.poses.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  // Obstacle check first — fast rejection for colliding trajectories
  double obs_score = calculateObstacleScore(traj);
  if (std::isinf(obs_score)) {
    return std::numeric_limits<double>::infinity();
  }

  double path_score    = calculatePathAlignmentScore(traj);
  double heading_score = calculateHeadingAlignmentScore(traj);
  double goal_score    = calculateGoalDistanceScore(traj);
  double steer_penalty = calculateSteeringPenalty(traj);

  // Velocity reward: higher |v| → lower cost (negative contribution)
  double vel_reward = -velocity_reward_weight_ * std::abs(traj.linear_vel);

  return path_distance_weight_     * path_score    +
         heading_alignment_weight_ * heading_score +
         goal_distance_weight_     * goal_score    +
         obstacle_weight_          * obs_score     +
         steering_penalty_weight_  * steer_penalty +
         vel_reward;
}

// ═══════════════════════════════════════════════════════════════════════════
// Score components
// ═══════════════════════════════════════════════════════════════════════════

double CustomLocalPlanner::calculatePathAlignmentScore(const Trajectory & traj)
{
  if (pruned_plan_.poses.empty() || traj.poses.empty()) {
    return 1000.0;
  }

  // Weighted average distance to nearest path point.
  // Weight increases linearly along the trajectory — trajectory end matters more
  // than the start, pushing the robot to look further ahead.
  double total_dist   = 0.0;
  double total_weight = 0.0;
  const size_t N = traj.poses.size();

  for (size_t i = 0; i < N; ++i) {
    double min_dist = std::numeric_limits<double>::max();
    const auto & tp = traj.poses[i];

    for (const auto & pp : pruned_plan_.poses) {
      double dx = tp.pose.position.x - pp.pose.position.x;
      double dy = tp.pose.position.y - pp.pose.position.y;
      double d  = std::sqrt(dx * dx + dy * dy);
      if (d < min_dist) { min_dist = d; }
    }

    // w ∈ [1, 3]  — 3× more weight on end pose than on start
    double w = 1.0 + 2.0 * static_cast<double>(i) / static_cast<double>(N);
    total_dist   += min_dist * w;
    total_weight += w;
  }

  return total_weight > 0.0 ? total_dist / total_weight : 1000.0;
}

double CustomLocalPlanner::calculateObstacleScore(const Trajectory & traj)
{
  double total_cost  = 0.0;
  int    valid_steps = 0;

  for (const auto & p : traj.poses) {
    unsigned int mx, my;
    if (!costmap_->worldToMap(p.pose.position.x, p.pose.position.y, mx, my)) {
      // Left the local costmap window — stop scoring but keep the trajectory.
      // Discarding would penalise fast trajectories unfairly.
      break;
    }

    unsigned char c = costmap_->getCost(mx, my);

    if (c == nav2_costmap_2d::LETHAL_OBSTACLE ||
        c == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      return std::numeric_limits<double>::infinity();
    }

    total_cost += c;
    ++valid_steps;
  }

  if (valid_steps == 0) {
    return std::numeric_limits<double>::infinity();
  }

  return total_cost / valid_steps;
}

double CustomLocalPlanner::calculateGoalDistanceScore(const Trajectory & traj)
{
  if (pruned_plan_.poses.empty() || traj.poses.empty()) {
    return 0.0;
  }

  // "Carrot": end of the pruned local plan window.
  // Using the local carrot instead of the global goal gives meaningful
  // differentiation between trajectories even when the goal is far away.
  const auto & carrot   = pruned_plan_.poses.back();
  const auto & traj_end = traj.poses.back();

  double dx = carrot.pose.position.x - traj_end.pose.position.x;
  double dy = carrot.pose.position.y - traj_end.pose.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

double CustomLocalPlanner::calculateHeadingAlignmentScore(const Trajectory & traj)
{
  if (pruned_plan_.poses.size() < 2 || traj.poses.empty()) {
    return 0.0;
  }

  // Robot heading at end of trajectory
  double traj_heading = tf2::getYaw(traj.poses.back().pose.orientation);

  // Find the plan pose closest to the trajectory end
  size_t closest_idx = 0;
  double min_dist     = std::numeric_limits<double>::max();
  const auto & traj_end = traj.poses.back();

  for (size_t i = 0; i < pruned_plan_.poses.size(); ++i) {
    double dx = traj_end.pose.position.x - pruned_plan_.poses[i].pose.position.x;
    double dy = traj_end.pose.position.y - pruned_plan_.poses[i].pose.position.y;
    double d  = std::sqrt(dx * dx + dy * dy);
    if (d < min_dist) { min_dist = d; closest_idx = i; }
  }

  // Path tangent at closest point
  // SAFETY: closest_idx is size_t — never subtract without checking > 0,
  //         otherwise 0 - 1 wraps to SIZE_MAX and causes undefined behaviour.
  double path_heading;
  if (closest_idx + 1 < pruned_plan_.poses.size()) {
    const auto & a = pruned_plan_.poses[closest_idx];
    const auto & b = pruned_plan_.poses[closest_idx + 1];
    path_heading = std::atan2(
      b.pose.position.y - a.pose.position.y,
      b.pose.position.x - a.pose.position.x);
  } else if (closest_idx > 0) {
    const auto & a = pruned_plan_.poses[closest_idx - 1];
    const auto & b = pruned_plan_.poses[closest_idx];
    path_heading = std::atan2(
      b.pose.position.y - a.pose.position.y,
      b.pose.position.x - a.pose.position.x);
  } else {
    // Single-pose pruned plan — use final goal yaw so rotate-in-place
    // candidates that end closer to the goal heading score better.
    if (!global_plan_.poses.empty()) {
      path_heading = tf2::getYaw(global_plan_.poses.back().pose.orientation);
    } else {
      return 0.0;
    }
  }

  // Absolute angular difference in [0, π]
  return std::fabs(std::atan2(
    std::sin(traj_heading - path_heading),
    std::cos(traj_heading - path_heading)));
}

double CustomLocalPlanner::calculateSteeringPenalty(const Trajectory & traj)
{
  if (traj.rotate_in_place) {
    return 0.0;  // In-place rotation has no steering angle cost
  }
  // δ² penalty: discourages unnecessary steering and prefers straight paths.
  return traj.steering_angle * traj.steering_angle;
}

// ═══════════════════════════════════════════════════════════════════════════
// pruneGlobalPlan
//
// Forward-only search from last_closest_idx_ to prevent re-visiting
// already-passed waypoints after a brief detour or overshoot.
// ═══════════════════════════════════════════════════════════════════════════

nav_msgs::msg::Path CustomLocalPlanner::pruneGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose,
  double prune_distance)
{
  nav_msgs::msg::Path pruned;
  pruned.header = global_plan_.header;

  if (global_plan_.poses.empty()) {
    return pruned;
  }

  // Forward-only closest-point search
  size_t closest_idx = last_closest_idx_;
  double min_dist     = std::numeric_limits<double>::max();

  for (size_t i = last_closest_idx_; i < global_plan_.poses.size(); ++i) {
    double dx = pose.pose.position.x - global_plan_.poses[i].pose.position.x;
    double dy = pose.pose.position.y - global_plan_.poses[i].pose.position.y;
    double d  = std::sqrt(dx * dx + dy * dy);
    if (d < min_dist) { min_dist = d; closest_idx = i; }
  }

  last_closest_idx_ = closest_idx;  // index never retreats

  // Collect window ahead up to prune_distance
  double acc_dist = 0.0;
  pruned.poses.push_back(global_plan_.poses[closest_idx]);

  for (size_t i = closest_idx + 1; i < global_plan_.poses.size(); ++i) {
    const auto & prev = global_plan_.poses[i - 1];
    const auto & curr = global_plan_.poses[i];
    double dx = curr.pose.position.x - prev.pose.position.x;
    double dy = curr.pose.position.y - prev.pose.position.y;
    acc_dist += std::sqrt(dx * dx + dy * dy);
    if (acc_dist > prune_distance) { break; }
    pruned.poses.push_back(curr);
  }

  RCLCPP_DEBUG(logger_, "Pruned plan: %zu poses, %.2f m window (idx=%zu/%zu)",
    pruned.poses.size(), acc_dist, closest_idx, global_plan_.poses.size());

  return pruned;
}

// ═══════════════════════════════════════════════════════════════════════════
// publishLocalPlan
// ═══════════════════════════════════════════════════════════════════════════

void CustomLocalPlanner::publishLocalPlan(const Trajectory & traj)
{
  if (local_plan_pub_->get_subscription_count() > 0 && !traj.poses.empty()) {
    nav_msgs::msg::Path plan;
    plan.header.frame_id = traj.poses[0].header.frame_id;
    plan.header.stamp    = clock_->now();
    plan.poses           = traj.poses;
    local_plan_pub_->publish(plan);
  }
}

}  // namespace robot_custom_local_planner

PLUGINLIB_EXPORT_CLASS(
  robot_custom_local_planner::CustomLocalPlanner,
  nav2_core::Controller)
