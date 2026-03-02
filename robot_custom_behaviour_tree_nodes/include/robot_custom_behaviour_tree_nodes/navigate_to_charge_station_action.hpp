#ifndef ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__NAVIGATE_TO_CHARGE_STATION_ACTION_HPP_
#define ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__NAVIGATE_TO_CHARGE_STATION_ACTION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace robot_custom_behaviour_tree_nodes
{

/**
 * @brief Custom BT Node to navigate to charge station using Nav2
 * 
 * This node takes a charge station pose from the blackboard and uses
 * Nav2's NavigateToPose action to navigate the robot to that position.
 * 
 * Input Ports:
 *   - charge_station_pose: Charge station pose (geometry_msgs::msg::PoseStamped)
 *   - server_timeout: Action server timeout in seconds (default: 10.0)
 *   - action_timeout: Navigation action timeout in seconds (default: 300.0)
 * 
 * This is an asynchronous action node that waits for the navigation to complete.
 */
class NavigateToChargeStationAction : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigateToChargeStationAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  ~NavigateToChargeStationAction() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("charge_station_pose", 
                                                       "Charge station pose to navigate to"),
      BT::InputPort<double>("server_timeout", 10.0, "Action server timeout (seconds)"),
      BT::InputPort<double>("action_timeout", 300.0, "Navigation timeout (seconds)")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  std::shared_ptr<GoalHandleNavigateToPose> goal_handle_;
  
  rclcpp::Time start_time_;
  double action_timeout_;
  bool goal_sent_;
  bool goal_result_available_;
  rclcpp_action::ResultCode result_code_;
  
  /**
   * @brief Send navigation goal to Nav2
   */
  bool sendNavigationGoal(const geometry_msgs::msg::PoseStamped & target_pose);
  
  /**
   * @brief Check if action server is available
   */
  bool waitForActionServer(double timeout);
  
  /**
   * @brief Goal response callback
   */
  void goalResponseCallback(const GoalHandleNavigateToPose::SharedPtr & goal_handle);
  
  /**
   * @brief Feedback callback
   */
  void feedbackCallback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  
  /**
   * @brief Result callback
   */
  void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result);
};

}  // namespace robot_custom_behaviour_tree_nodes

#endif  // ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__NAVIGATE_TO_CHARGE_STATION_ACTION_HPP_
