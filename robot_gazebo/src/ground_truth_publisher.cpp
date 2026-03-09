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

    robot_name_   = this->get_parameter("robot_name").as_string();
    world_name_   = this->get_parameter("world_name").as_string();
    map_frame_    = this->get_parameter("map_frame").as_string();
    child_frame_  = this->get_parameter("ground_truth_child_frame").as_string();

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

      odom.pose.pose.position.x    = pos.x();
      odom.pose.pose.position.y    = pos.y();
      odom.pose.pose.position.z    = pos.z();
      odom.pose.pose.orientation.x = ori.x();
      odom.pose.pose.orientation.y = ori.y();
      odom.pose.pose.orientation.z = ori.z();
      odom.pose.pose.orientation.w = ori.w();
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
