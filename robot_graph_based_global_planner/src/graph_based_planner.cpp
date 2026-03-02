#include "robot_graph_based_global_planner/graph_based_planner.hpp"
#include <cmath>
#include <string>
#include <memory>
#include <fstream>
#include <algorithm>
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace robot_graph_based_global_planner
{

void GraphBasedPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  logger_ = node->get_logger();
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();

  RCLCPP_INFO(
    logger_, "Configuring plugin %s of type GraphBasedPlanner",
    name_.c_str());

  // Read parameters
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".interpolation_resolution",
    rclcpp::ParameterValue(0.1));
  node->get_parameter(name + ".interpolation_resolution", interpolation_resolution_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".graph_file_path",
    rclcpp::ParameterValue(""));
  node->get_parameter(name + ".graph_file_path", graph_file_path_);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_node_search_radius",
    rclcpp::ParameterValue(5.0));
  node->get_parameter(name + ".max_node_search_radius", max_node_search_radius_);

  // Load navigation graph
  if (graph_file_path_.empty()) {
    RCLCPP_ERROR(logger_, "Graph file path not specified!");
    return;
  }

  if (!loadGraphFromJson(graph_file_path_)) {
    RCLCPP_ERROR(logger_, "Failed to load navigation graph from: %s", graph_file_path_.c_str());
    return;
  }

  RCLCPP_INFO(
    logger_, 
    "GraphBasedPlanner configured with %zu nodes and interpolation resolution: %.2f",
    graph_.nodes.size(), interpolation_resolution_);
}

void GraphBasedPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up plugin %s", name_.c_str());
  graph_.nodes.clear();
  graph_.adjacency_list.clear();
}

void GraphBasedPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s", name_.c_str());
}

void GraphBasedPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating plugin %s", name_.c_str());
}

bool GraphBasedPlanner::loadGraphFromJson(const std::string & file_path)
{
  try {
    std::ifstream file(file_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(logger_, "Cannot open graph file: %s", file_path.c_str());
      return false;
    }

    json j;
    file >> j;

    // Parse nodes
    for (const auto & node_json : j["graph"]["nodes"]) {
      GraphNode node;
      node.id = node_json["id"];
      node.x = node_json["x"];
      node.y = node_json["y"];
      node.z = node_json["z"];
      node.type = node_json["type"];
      node.description = node_json["description"];
      
      graph_.nodes[node.id] = node;
      graph_.adjacency_list[node.id] = std::vector<GraphEdge>();
    }

    // Parse edges
    for (const auto & edge_json : j["graph"]["edges"]) {
      GraphEdge edge;
      edge.from = edge_json["from"];
      edge.to = edge_json["to"];
      // Calculate dynamic cost based on Euclidean distance
      edge.cost = calculateEdgeCost(edge.from, edge.to);
      edge.bidirectional = edge_json["bidirectional"];
      edge.max_speed = edge_json["max_speed"];

      graph_.adjacency_list[edge.from].push_back(edge);

      // Add reverse edge if bidirectional
      if (edge.bidirectional) {
        GraphEdge reverse_edge = edge;
        reverse_edge.from = edge.to;
        reverse_edge.to = edge.from;
        reverse_edge.cost = edge.cost; // Same distance for both directions
        graph_.adjacency_list[edge.to].push_back(reverse_edge);
      }
    }

    RCLCPP_INFO(
      logger_, 
      "Successfully loaded graph with %zu nodes", 
      graph_.nodes.size());
    
    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Error parsing JSON: %s", e.what());
    return false;
  }
}

std::string GraphBasedPlanner::findBestNode(double x, double y, double goal_x, double goal_y)
{
  std::string best_id;
  double best_score = std::numeric_limits<double>::max();

  // Calculate direction vector from current position to goal
  double goal_dx = goal_x - x;
  double goal_dy = goal_y - y;
  double goal_distance = std::sqrt(goal_dx * goal_dx + goal_dy * goal_dy);
  
  // Normalize goal direction vector
  if (goal_distance > 0.001) {
    goal_dx /= goal_distance;
    goal_dy /= goal_distance;
  }

  for (const auto & [id, node] : graph_.nodes) {
    double dx = node.x - x;
    double dy = node.y - y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // Skip nodes that are too far away
    if (distance > max_node_search_radius_) {
      continue;
    }

    // Calculate direction vector from current position to this node
    double node_dx = dx / (distance + 0.001); // Add small epsilon to avoid division by zero
    double node_dy = dy / (distance + 0.001);

    // Calculate dot product to measure alignment with goal direction
    double direction_alignment = node_dx * goal_dx + node_dy * goal_dy;
    
    // Penalize nodes that are in opposite direction to goal
    // direction_alignment ranges from -1 (opposite) to 1 (same direction)
    double direction_penalty = 1.0;
    if (direction_alignment < 0) {
      // Node is behind us relative to goal direction
      direction_penalty = 3.0; // Increase penalty for backward nodes
    } else if (direction_alignment < 0.5) {
      // Node is sideways relative to goal direction
      direction_penalty = 1.5;
    }

    // Combined score: distance weighted by direction penalty
    double score = distance * direction_penalty;

    if (score < best_score) {
      best_score = score;
      best_id = id;
    }
  }

  if (best_score == std::numeric_limits<double>::max()) {
    RCLCPP_WARN(
      logger_,
      "No suitable node found within radius %.2f meters",
      max_node_search_radius_);
    // Fallback to simple nearest neighbor
    return findNearestNode(x, y);
  }

  RCLCPP_INFO(
    logger_,
    "Selected node '%s' with score %.2f (considering direction to goal)",
    best_id.c_str(), best_score);

  return best_id;
}

std::string GraphBasedPlanner::findNearestNode(double x, double y)
{
  std::string nearest_id;
  double min_distance = std::numeric_limits<double>::max();

  for (const auto & [id, node] : graph_.nodes) {
    double dx = node.x - x;
    double dy = node.y - y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < min_distance) {
      min_distance = distance;
      nearest_id = id;
    }
  }

  if (min_distance > max_node_search_radius_) {
    RCLCPP_WARN(
      logger_,
      "Nearest node '%s' is %.2f meters away (max: %.2f)",
      nearest_id.c_str(), min_distance, max_node_search_radius_);
  }

  return nearest_id;
}

double GraphBasedPlanner::calculateEdgeCost(const std::string & from_node, const std::string & to_node)
{
  const auto & from = graph_.nodes[from_node];
  const auto & to = graph_.nodes[to_node];
  
  double dx = to.x - from.x;
  double dy = to.y - from.y;
  double dz = to.z - from.z;
  
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double GraphBasedPlanner::heuristic(const std::string & node1, const std::string & node2)
{
  const auto & n1 = graph_.nodes[node1];
  const auto & n2 = graph_.nodes[node2];
  
  double dx = n2.x - n1.x;
  double dy = n2.y - n1.y;
  
  return std::sqrt(dx * dx + dy * dy);
}

std::vector<std::string> GraphBasedPlanner::computeShortestPath(
  const std::string & start_node,
  const std::string & goal_node)
{
  // A* algorithm implementation
  struct AStarNode {
    std::string id;
    double g_score;  // Cost from start
    double f_score;  // g_score + heuristic
    std::string parent;

    bool operator>(const AStarNode & other) const {
      return f_score > other.f_score;
    }
  };

  std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
  std::unordered_map<std::string, double> g_scores;
  std::unordered_map<std::string, std::string> came_from;

  // Initialize
  for (const auto & [id, _] : graph_.nodes) {
    g_scores[id] = std::numeric_limits<double>::max();
  }
  g_scores[start_node] = 0.0;

  AStarNode start;
  start.id = start_node;
  start.g_score = 0.0;
  start.f_score = heuristic(start_node, goal_node);
  start.parent = "";

  open_set.push(start);

  while (!open_set.empty()) {
    AStarNode current = open_set.top();
    open_set.pop();

    if (current.id == goal_node) {
      // Reconstruct path
      std::vector<std::string> path;
      std::string node = goal_node;
      
      while (!node.empty()) {
        path.push_back(node);
        node = came_from[node];
      }
      
      std::reverse(path.begin(), path.end());
      return path;
    }

    // Check neighbors
    for (const auto & edge : graph_.adjacency_list[current.id]) {
      double tentative_g_score = g_scores[current.id] + edge.cost;

      if (tentative_g_score < g_scores[edge.to]) {
        came_from[edge.to] = current.id;
        g_scores[edge.to] = tentative_g_score;

        AStarNode neighbor;
        neighbor.id = edge.to;
        neighbor.g_score = tentative_g_score;
        neighbor.f_score = tentative_g_score + heuristic(edge.to, goal_node);
        neighbor.parent = current.id;

        open_set.push(neighbor);
      }
    }
  }

  // No path found
  RCLCPP_WARN(logger_, "No path found from %s to %s", start_node.c_str(), goal_node.c_str());
  return std::vector<std::string>();
}

void GraphBasedPlanner::interpolatePath(
  nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & from,
  const geometry_msgs::msg::PoseStamped & to,
  double resolution)
{
  double dx = to.pose.position.x - from.pose.position.x;
  double dy = to.pose.position.y - from.pose.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  int num_steps = static_cast<int>(std::ceil(distance / resolution));
  if (num_steps < 1) {
    num_steps = 1;
  }

  // Start from i=1 to skip the 'from' pose (already added)
  for (int i = 1; i < num_steps; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = path.header.frame_id;
    pose.header.stamp = path.header.stamp;

    double t = static_cast<double>(i) / static_cast<double>(num_steps);
    pose.pose.position.x = from.pose.position.x + t * dx;
    pose.pose.position.y = from.pose.position.y + t * dy;
    pose.pose.position.z = 0.0;

    // Calculate orientation
    double yaw = std::atan2(dy, dx);
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);

    path.poses.push_back(pose);
  }
}

nav_msgs::msg::Path GraphBasedPlanner::convertToPath(
  const std::vector<std::string> & node_path,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = goal.header.frame_id;
  path.header.stamp = goal.header.stamp;

  if (node_path.empty()) {
    return path;
  }

  // Create poses for each node in the path
  for (size_t i = 0; i < node_path.size(); ++i) {
    const auto & node = graph_.nodes[node_path[i]];
    
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = path.header.frame_id;
    pose.header.stamp = path.header.stamp;
    pose.pose.position.x = node.x;
    pose.pose.position.y = node.y;
    pose.pose.position.z = node.z;

    // Calculate orientation towards next node
    if (i < node_path.size() - 1) {
      const auto & next_node = graph_.nodes[node_path[i + 1]];
      double dx = next_node.x - node.x;
      double dy = next_node.y - node.y;
      double yaw = std::atan2(dy, dx);
      
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = std::sin(yaw * 0.5);
      pose.pose.orientation.w = std::cos(yaw * 0.5);
    } else {
      // Last node - use goal orientation
      pose.pose.orientation = goal.pose.orientation;
    }

    path.poses.push_back(pose);

    // Interpolate between nodes
    if (i < node_path.size() - 1) {
      const auto & next_node = graph_.nodes[node_path[i + 1]];
      geometry_msgs::msg::PoseStamped next_pose;
      next_pose.header.frame_id = path.header.frame_id;
      next_pose.header.stamp = path.header.stamp;
      next_pose.pose.position.x = next_node.x;
      next_pose.pose.position.y = next_node.y;
      next_pose.pose.position.z = next_node.z;

      interpolatePath(path, pose, next_pose, interpolation_resolution_);
    }
  }

  return path;
}

nav_msgs::msg::Path GraphBasedPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  if (cancel_checker && cancel_checker()) {
    RCLCPP_WARN(logger_, "Planning was cancelled");
    return nav_msgs::msg::Path();
  }

  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(logger_, "Unable to lock node");
    return nav_msgs::msg::Path();
  }

  RCLCPP_INFO(
    logger_,
    "Creating plan from (%.2f, %.2f) to (%.2f, %.2f)",
    start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);

  // Find best nodes considering direction to goal
  std::string start_node = findBestNode(
    start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);
  std::string goal_node = findNearestNode(goal.pose.position.x, goal.pose.position.y);

  RCLCPP_INFO(
    logger_,
    "Nearest nodes: start='%s', goal='%s'",
    start_node.c_str(), goal_node.c_str());

  // Compute shortest path
  std::vector<std::string> node_path = computeShortestPath(start_node, goal_node);

  if (node_path.empty()) {
    RCLCPP_ERROR(logger_, "Failed to find path in graph");
    return nav_msgs::msg::Path();
  }

  RCLCPP_INFO(logger_, "Path through nodes:");
  for (const auto & node_id : node_path) {
    RCLCPP_INFO(logger_, "  -> %s (%s)", 
      node_id.c_str(), 
      graph_.nodes[node_id].description.c_str());
  }

  // Convert node path to ROS path
  nav_msgs::msg::Path path = convertToPath(node_path, goal);

  // Ensure path has correct timestamp and frame
  auto current_time = node->now();
  path.header.stamp = current_time;
  path.header.frame_id = goal.header.frame_id;

  // Add interpolation from robot position to first waypoint and from last waypoint to goal
  if (!path.poses.empty()) {
    // Interpolate from robot's current position to first waypoint
    geometry_msgs::msg::PoseStamped start_pose = start;
    start_pose.header.stamp = current_time;
    start_pose.header.frame_id = goal.header.frame_id;
    
    geometry_msgs::msg::PoseStamped first_waypoint = path.poses.front();
    first_waypoint.header.stamp = current_time;
    
    // Calculate distance to first waypoint
    double dx = first_waypoint.pose.position.x - start_pose.pose.position.x;
    double dy = first_waypoint.pose.position.y - start_pose.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    // Only add interpolation if robot is not already at the waypoint
    if (distance > 0.1) {  // 10cm threshold
      nav_msgs::msg::Path temp_path;
      temp_path.header = path.header;
      interpolatePath(temp_path, start_pose, first_waypoint, interpolation_resolution_);
      
      // Insert interpolated poses at beginning
      path.poses.insert(path.poses.begin(), temp_path.poses.begin(), temp_path.poses.end());
    }
    
    // Interpolate from last waypoint to goal position
    geometry_msgs::msg::PoseStamped goal_pose = goal;
    goal_pose.header.stamp = current_time;
    goal_pose.header.frame_id = goal.header.frame_id;
    
    geometry_msgs::msg::PoseStamped last_waypoint = path.poses.back();
    last_waypoint.header.stamp = current_time;
    
    // Calculate distance to goal
    dx = goal_pose.pose.position.x - last_waypoint.pose.position.x;
    dy = goal_pose.pose.position.y - last_waypoint.pose.position.y;
    distance = std::sqrt(dx * dx + dy * dy);
    
    // Only add interpolation if last waypoint is not already at goal
    if (distance > 0.1) {  // 10cm threshold
      interpolatePath(path, last_waypoint, goal_pose, interpolation_resolution_);
    }
    
    // Add final goal pose
    path.poses.push_back(goal_pose);
    
    // Update all pose timestamps
    for (auto & pose : path.poses) {
      pose.header.stamp = current_time;
      pose.header.frame_id = goal.header.frame_id;
    }
  }

  RCLCPP_INFO(
    logger_,
    "Created plan with %zu poses through %zu waypoints",
    path.poses.size(), node_path.size());
  
  // Debug: Log first and last pose
  if (!path.poses.empty()) {
    RCLCPP_INFO(logger_, "First pose: (%.2f, %.2f) in frame '%s'",
      path.poses.front().pose.position.x,
      path.poses.front().pose.position.y,
      path.poses.front().header.frame_id.c_str());
    RCLCPP_INFO(logger_, "Last pose: (%.2f, %.2f) in frame '%s'",
      path.poses.back().pose.position.x,
      path.poses.back().pose.position.y,
      path.poses.back().header.frame_id.c_str());
  }

  return path;
}

}  // namespace robot_graph_based_global_planner

// Register as plugin
PLUGINLIB_EXPORT_CLASS(robot_graph_based_global_planner::GraphBasedPlanner, nav2_core::GlobalPlanner)
