#ifndef ROBOT_GRAPH_BASED_GLOBAL_PLANNER__GRAPH_BASED_PLANNER_HPP_
#define ROBOT_GRAPH_BASED_GLOBAL_PLANNER__GRAPH_BASED_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <queue>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace robot_graph_based_global_planner
{

/**
 * @brief Graph node structure
 */
struct GraphNode
{
  std::string id;
  double x;
  double y;
  double z;
  std::string type;
  std::string description;
};

/**
 * @brief Graph edge structure
 */
struct GraphEdge
{
  std::string from;
  std::string to;
  double cost;
  bool bidirectional;
  double max_speed;
};

/**
 * @brief Navigation graph structure
 */
struct NavigationGraph
{
  std::unordered_map<std::string, GraphNode> nodes;
  std::unordered_map<std::string, std::vector<GraphEdge>> adjacency_list;
};

/**
 * @class GraphBasedPlanner
 * @brief Graph-based global planner - Uses predefined waypoint graph for navigation
 */
class GraphBasedPlanner : public nav2_core::GlobalPlanner
{
public:
  GraphBasedPlanner() = default;
  ~GraphBasedPlanner() = default;

  /**
   * @brief Configures the plugin
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleans up the plugin
   */
  void cleanup() override;

  /**
   * @brief Activates the plugin
   */
  void activate() override;

  /**
   * @brief Deactivates the plugin
   */
  void deactivate() override;

  /**
   * @brief Creates a plan using the navigation graph
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  /**
   * @brief Loads navigation graph from JSON file
   */
  bool loadGraphFromJson(const std::string & file_path);

  /**
   * @brief Finds best graph node considering distance and direction to goal
   */
  std::string findBestNode(double x, double y, double goal_x, double goal_y);

  /**
   * @brief Finds nearest graph node to a given position (fallback method)
   */
  std::string findNearestNode(double x, double y);

  /**
   * @brief Computes shortest path using A* algorithm
   */
  std::vector<std::string> computeShortestPath(
    const std::string & start_node,
    const std::string & goal_node);

  /**
   * @brief Converts node path to ROS Path message
   */
  nav_msgs::msg::Path convertToPath(
    const std::vector<std::string> & node_path,
    const geometry_msgs::msg::PoseStamped & goal);

  /**
   * @brief Interpolates path between two poses
   */
  void interpolatePath(
    nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & from,
    const geometry_msgs::msg::PoseStamped & to,
    double resolution);

  /**
   * @brief Calculates heuristic for A* (Euclidean distance)
   */
  double heuristic(const std::string & node1, const std::string & node2);

  /**
   * @brief Calculates dynamic edge cost based on Euclidean distance
   */
  double calculateEdgeCost(const std::string & from_node, const std::string & to_node);

  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Logger logger_{rclcpp::get_logger("GraphBasedPlanner")};
  std::string name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  
  // Navigation graph
  NavigationGraph graph_;
  
  // Parameters
  double interpolation_resolution_;
  std::string graph_file_path_;
  double max_node_search_radius_;
};

}  // namespace robot_graph_based_global_planner

#endif  // ROBOT_GRAPH_BASED_GLOBAL_PLANNER__GRAPH_BASED_PLANNER_HPP_
