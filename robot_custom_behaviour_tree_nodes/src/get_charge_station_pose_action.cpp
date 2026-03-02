#include "robot_custom_behaviour_tree_nodes/get_charge_station_pose_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/LinearMath/Quaternion.h>

namespace robot_custom_behaviour_tree_nodes
{

GetChargeStationPoseAction::GetChargeStationPoseAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  // Create ROS node
  node_ = rclcpp::Node::make_shared("get_charge_station_pose_action_node");
  
  // Create TF buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  RCLCPP_INFO(node_->get_logger(), "GetChargeStationPoseAction initialized");
}

BT::NodeStatus GetChargeStationPoseAction::tick()
{
  // Get parameters
  bool use_tf = false;
  getInput("use_tf", use_tf);
  
  geometry_msgs::msg::PoseStamped charge_station_pose;
  
  if (use_tf) {
    // Get charge station position using TF
    std::string frame_id;
    std::string charge_station_frame;
    
    if (!getInput("frame_id", frame_id)) {
      frame_id = "map";
    }
    
    if (!getInput("charge_station_frame", charge_station_frame)) {
      charge_station_frame = "charge_station";
    }
    
    RCLCPP_INFO(node_->get_logger(), 
      "Getting charge station pose from TF: %s -> %s", 
      frame_id.c_str(), charge_station_frame.c_str());
    
    if (!getChargeStationPoseFromTF(frame_id, charge_station_frame, charge_station_pose)) {
      RCLCPP_ERROR(node_->get_logger(), 
        "Failed to get charge station pose from TF. Falling back to parameters.");
      use_tf = false;  // TF failed, fall back to parameters
    }
  }
  
  if (!use_tf) {
    // Create charge station position from parameters
    double x, y, yaw;
    std::string frame_id;
    
    if (!getInput("charge_station_x", x)) {
      x = 0.0;
    }
    
    if (!getInput("charge_station_y", y)) {
      y = 0.0;
    }
    
    if (!getInput("charge_station_yaw", yaw)) {
      yaw = 0.0;
    }
    
    if (!getInput("frame_id", frame_id)) {
      frame_id = "map";
    }
    
    RCLCPP_INFO(node_->get_logger(), 
      "Creating charge station pose from parameters: x=%.2f, y=%.2f, yaw=%.2f in frame '%s'", 
      x, y, yaw, frame_id.c_str());
    
    charge_station_pose = createChargeStationPoseFromParams(x, y, yaw, frame_id);
  }
  
  // Write to output port
  setOutput("charge_station_pose", charge_station_pose);
  
  RCLCPP_INFO(node_->get_logger(), 
    "Charge station pose set: [%.2f, %.2f] in frame '%s'", 
    charge_station_pose.pose.position.x,
    charge_station_pose.pose.position.y,
    charge_station_pose.header.frame_id.c_str());
  
  return BT::NodeStatus::SUCCESS;
}

bool GetChargeStationPoseAction::getChargeStationPoseFromTF(
  const std::string & target_frame,
  const std::string & charge_station_frame,
  geometry_msgs::msg::PoseStamped & pose)
{
  try {
    // Get TF transform (timeout: 1 second)
    geometry_msgs::msg::TransformStamped transform_stamped = 
      tf_buffer_->lookupTransform(
        target_frame, 
        charge_station_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(1.0));
    
    // Convert transform to PoseStamped
    pose.header = transform_stamped.header;
    pose.pose.position.x = transform_stamped.transform.translation.x;
    pose.pose.position.y = transform_stamped.transform.translation.y;
    pose.pose.position.z = transform_stamped.transform.translation.z;
    pose.pose.orientation = transform_stamped.transform.rotation;
    
    return true;
  }
  catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), 
      "Could not get transform from '%s' to '%s': %s", 
      target_frame.c_str(), charge_station_frame.c_str(), ex.what());
    return false;
  }
}

geometry_msgs::msg::PoseStamped 
GetChargeStationPoseAction::createChargeStationPoseFromParams(
  double x, double y, double yaw, const std::string & frame_id)
{
  geometry_msgs::msg::PoseStamped pose;
  
  // Set header
  pose.header.frame_id = frame_id;
  pose.header.stamp = node_->now();
  
  // Set position
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  
  // Set orientation (yaw -> quaternion)
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
  
  return pose;
}

}  // namespace robot_custom_behaviour_tree_nodes

// Register as BT plugin
#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robot_custom_behaviour_tree_nodes::GetChargeStationPoseAction>("GetChargeStationPose");
}
