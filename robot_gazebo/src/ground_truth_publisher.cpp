/**
 * Ground Truth Publisher (C++)
 * ============================
 * Subscribes directly to Gazebo's /world/default/dynamic_pose/info transport
 * topic via gz::transport (bypasses ros_gz_bridge name-mapping bug).
 *
 * Reads gz::msgs::Pose_V, finds the entry whose .name() == robot_name param,
 * and publishes:
 *   /ground_truth/odom  (nav_msgs/msg/Odometry)
 *   /ground_truth/pose  (geometry_msgs/msg/PoseStamped)
 *
 * Why C++ instead of Python: gz-python transport bindings are not installed
 * and the ros_gz_bridge Pose_V→TFMessage conversion discards model names
 * (child_frame_id ends up empty).
 */

#include <string>
#include <memory>
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>

using namespace std::chrono_literals;

class GroundTruthPublisher : public rclcpp::Node
{
public:
  GroundTruthPublisher()
  : Node("ground_truth_publisher"),
    msg_count_(0)
  {
    // ── Parameters ─────────────────────────────────────────────────────────
    this->declare_parameter<std::string>("robot_name", "robot");
    this->declare_parameter<std::string>("world_name", "default");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("ground_truth_child_frame", "ground_truth_base_link");
    // Offset from Gazebo world frame to the ROS map frame (translation + yaw).
    // The SLAM map frame origin is wherever the robot was in the Gazebo world
    // when the mapping session started.  If that starting position was NOT at
    // Gazebo world (0, 0), the two frames differ by exactly that offset.
    // Set these to the robot's Gazebo world (x, y) position at SLAM-start
    // (equivalently: observe where /fgo/odometry settles when the robot is at
    // Gazebo world origin after NDT locks on; that reported position is the
    // offset you need here, with opposite sign).
    this->declare_parameter<double>("world_to_map_offset_x",   0.0);
    this->declare_parameter<double>("world_to_map_offset_y",   0.0);
    this->declare_parameter<double>("world_to_map_offset_yaw", 0.0);

    robot_name_   = this->get_parameter("robot_name").as_string();
    world_name_   = this->get_parameter("world_name").as_string();
    map_frame_    = this->get_parameter("map_frame").as_string();
    child_frame_  = this->get_parameter("ground_truth_child_frame").as_string();
    off_x_   = this->get_parameter("world_to_map_offset_x").as_double();
    off_y_   = this->get_parameter("world_to_map_offset_y").as_double();
    off_yaw_ = this->get_parameter("world_to_map_offset_yaw").as_double();

    // ── ROS publishers ──────────────────────────────────────────────────────
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/ground_truth/odom", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/ground_truth/pose", 10);

    // ── gz::transport subscriber ────────────────────────────────────────────
    // Direct connection to Gazebo physics engine – zero sensor noise.
    std::string gz_topic =
      "/world/" + world_name_ + "/dynamic_pose/info";

    bool ok = gz_node_.Subscribe(
      gz_topic,
      &GroundTruthPublisher::onGzPose,
      this);

    if (!ok) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to subscribe to gz topic: %s", gz_topic.c_str());
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "GroundTruthPublisher started\n"
        "  Gz topic   : %s\n"
        "  Robot name : %s\n"
        "  Publishing : /ground_truth/odom  (nav_msgs/Odometry)\n"
        "  Publishing : /ground_truth/pose  (geometry_msgs/PoseStamped)",
        gz_topic.c_str(), robot_name_.c_str());
    }

    // Watchdog: warn every 5 s if no data received
    watchdog_ = this->create_wall_timer(
      5s, [this]() {
        if (msg_count_ == 0) {
          RCLCPP_WARN(
            this->get_logger(),
            "No ground truth poses received yet. "
            "Check that Gazebo is running and model name matches '%s'.",
            robot_name_.c_str());
        } else {
          RCLCPP_INFO(
            this->get_logger(),
            "[ground_truth] Publishing OK – %lu msgs in last 5 s.", msg_count_);
          msg_count_ = 0;
        }
      });
  }

private:
  // ── gz callback (called from gz transport thread) ─────────────────────────
  void onGzPose(const gz::msgs::Pose_V & poses_msg)
  {
    for (int i = 0; i < poses_msg.pose_size(); ++i) {
      const auto & pose = poses_msg.pose(i);

      if (pose.name() != robot_name_) {
        continue;
      }

      // Timestamp from Gazebo header (simulation time)
      builtin_interfaces::msg::Time stamp;
      stamp.sec     = poses_msg.header().stamp().sec();
      stamp.nanosec = poses_msg.header().stamp().nsec();

      const auto & pos = pose.position();
      const auto & ori = pose.orientation();

      // ── nav_msgs/Odometry ─────────────────────────────────────────────────
      nav_msgs::msg::Odometry odom;
      odom.header.stamp    = stamp;
      odom.header.frame_id = map_frame_;
      odom.child_frame_id  = child_frame_;

      // Apply world→map frame offset so the published pose is in the same
      // coordinate system as /fgo/odometry (the ROS map frame).
      // T_map_robot = T_world_map^{-1} * T_world_robot
      // For a pure 2-D offset (tx, ty, yaw) the inverse is: (-tx*cos(yaw)-ty*sin(yaw),
      // tx*sin(yaw)-ty*cos(yaw), -yaw), but since Gazebo world and map frame have the
      // same Z-up orientation (yaw offset = 0 by convention for a correctly built map),
      // this simplifies to a plain XY translation:
      //   x_map = cos(off_yaw)*(x_world - off_x) + sin(off_yaw)*(y_world - off_y)
      //   y_map = -sin(off_yaw)*(x_world - off_x) + cos(off_yaw)*(y_world - off_y)
      const double cos_yaw = std::cos(off_yaw_);
      const double sin_yaw = std::sin(off_yaw_);
      const double dx = pos.x() - off_x_;
      const double dy = pos.y() - off_y_;
      const double map_x =  cos_yaw * dx + sin_yaw * dy;
      const double map_y = -sin_yaw * dx + cos_yaw * dy;

      // Yaw in world frame, corrected by map offset yaw
      // (Extract yaw from quaternion, subtract offset, rebuild quaternion)
      const double gz_qw = ori.w(), gz_qx = ori.x(), gz_qy = ori.y(), gz_qz = ori.z();
      const double world_yaw = std::atan2(
        2.0 * (gz_qw * gz_qz + gz_qx * gz_qy),
        1.0 - 2.0 * (gz_qy * gz_qy + gz_qz * gz_qz));
      const double map_yaw = world_yaw - off_yaw_;
      const double half = map_yaw * 0.5;
      const double map_qw =  std::cos(half);
      const double map_qz =  std::sin(half);

      odom.pose.pose.position.x    = map_x;
      odom.pose.pose.position.y    = map_y;
      odom.pose.pose.position.z    = pos.z();
      odom.pose.pose.orientation.x = 0.0;
      odom.pose.pose.orientation.y = 0.0;
      odom.pose.pose.orientation.z = map_qz;
      odom.pose.pose.orientation.w = map_qw;
      // All covariances are zero → perfect ground truth
      odom.pose.covariance.fill(0.0);
      odom.twist.covariance.fill(0.0);

      odom_pub_->publish(odom);

      // ── geometry_msgs/PoseStamped ─────────────────────────────────────────
      geometry_msgs::msg::PoseStamped ps;
      ps.header.stamp    = stamp;
      ps.header.frame_id = map_frame_;
      ps.pose            = odom.pose.pose;

      pose_pub_->publish(ps);

      ++msg_count_;
      return;  // found the robot, no need to iterate further
    }
  }

  // ── Members ───────────────────────────────────────────────────────────────
  std::string robot_name_;
  std::string world_name_;
  std::string map_frame_;
  std::string child_frame_;
  double off_x_{};
  double off_y_{};
  double off_yaw_{};

  gz::transport::Node gz_node_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  rclcpp::TimerBase::SharedPtr watchdog_;
  std::size_t msg_count_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundTruthPublisher>());
  rclcpp::shutdown();
  return 0;
}
