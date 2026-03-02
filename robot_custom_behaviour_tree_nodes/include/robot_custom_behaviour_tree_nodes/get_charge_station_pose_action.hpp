#ifndef ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__GET_CHARGE_STATION_POSE_ACTION_HPP_
#define ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__GET_CHARGE_STATION_POSE_ACTION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace robot_custom_behaviour_tree_nodes
{

/**
 * @brief Custom BT Node to get charge station pose
 * 
 * This node retrieves the charge station position and writes it to the blackboard.
 * The charge station position can be determined by 3 different methods:
 * 1. From parameter file (x, y, yaw values)
 * 2. From TF frame (charge_station frame)
 * 3. From topic (geometry_msgs/PoseStamped)
 * 
 * Output Ports:
 *   - charge_station_pose: Charge station pose (geometry_msgs::msg::PoseStamped)
 * 
 * Input Ports:
 *   - charge_station_x: Charge station X coordinate (default: 0.0)
 *   - charge_station_y: Charge station Y coordinate (default: 0.0)
 *   - charge_station_yaw: Charge station yaw angle (default: 0.0)
 *   - frame_id: Target frame ID (default: "map")
 *   - use_tf: Use TF to get charge station pose (default: false)
 *   - charge_station_frame: Charge station TF frame ID (default: "charge_station")
 */
class GetChargeStationPoseAction : public BT::SyncActionNode
{
public:
  GetChargeStationPoseAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  ~GetChargeStationPoseAction() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("charge_station_pose", 
                                                       "Charge station pose output"),
      BT::InputPort<double>("charge_station_x", 0.0, "Charge station X coordinate"),
      BT::InputPort<double>("charge_station_y", 0.0, "Charge station Y coordinate"),
      BT::InputPort<double>("charge_station_yaw", 0.0, "Charge station yaw angle (radians)"),
      BT::InputPort<std::string>("frame_id", "map", "Target frame ID"),
      BT::InputPort<bool>("use_tf", false, "Use TF to get charge station pose"),
      BT::InputPort<std::string>("charge_station_frame", "charge_station", 
                                  "Charge station TF frame ID")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  /**
   * @brief Get charge station position using TF
   */
  bool getChargeStationPoseFromTF(
    const std::string & target_frame,
    const std::string & charge_station_frame,
    geometry_msgs::msg::PoseStamped & pose);
    
  /**
   * @brief Create charge station position from parameters
   */
  geometry_msgs::msg::PoseStamped createChargeStationPoseFromParams(
    double x, double y, double yaw, const std::string & frame_id);
};

}  // namespace robot_custom_behaviour_tree_nodes

#endif  // ROBOT_CUSTOM_BEHAVIOUR_TREE_NODES__GET_CHARGE_STATION_POSE_ACTION_HPP_
