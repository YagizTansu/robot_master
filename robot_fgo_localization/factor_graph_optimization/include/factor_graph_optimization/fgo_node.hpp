#pragma once

#include <memory>
#include <mutex>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>

// ROS 2 messages
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// TF2
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Core utilities (ROS-free math helpers)
#include "factor_graph_optimization/core/pose_conversion.hpp"

// Configuration struct
#include "factor_graph_optimization/config/fgo_config.hpp"

// Odometry module
#include "factor_graph_optimization/odometry/sensor_buffer.hpp"
#include "factor_graph_optimization/odometry/keyframe_selector.hpp"

// Graph module (owns iSAM2, optimised states, IMU preintegrator)
#include "factor_graph_optimization/graph/graph_manager.hpp"

namespace factor_graph_optimization
{

// ImuSample and OdomSample are defined in odometry/sensor_buffer.hpp.

class FgoNode : public rclcpp::Node
{
public:
  explicit FgoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── Parameter helpers ─────────────────────────────────────────────────────
  // Parameters are loaded via FgoConfig::fromNode() — see config/fgo_config.hpp.

  // ── Initialisation ────────────────────────────────────────────────────────
  // ── Callbacks ─────────────────────────────────────────────────────────────
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void scanPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  // ── Optimization timer ────────────────────────────────────────────────────
  void optimizationStep();

  // ── TF publishing helpers ─────────────────────────────────────────────────
  void publishOdomToBase(const rclcpp::Time & stamp,
                         const geometry_msgs::msg::Pose & raw_pose);
  /// Recompute and cache the map→odom transform from the latest optimised pose
  /// and the corresponding keyframe odom anchor (last_consumed_odom_pose_).
  /// Must be called after iSAM2 update and before publishMapToOdom.
  void updateMapToOdomCache();
  /// Publish the cached map→odom transform with an updated stamp.
  void publishMapToOdom(const rclcpp::Time & stamp);

  // Conversion helpers are now free functions in the factor_graph_optimization
  // namespace — see core/pose_conversion.hpp and core/noise_model.hpp.

  // ── Graph module (owns iSAM2, factor graph, optimised states, IMU preint) ──
  std::unique_ptr<GraphManager>     graph_mgr_;

  // ── GPS module ───────────────────────────────────────────────────────────
  std::unique_ptr<GpsHandler>       gps_handler_;

  // ── Keyframe selector ─────────────────────────────────────────────────────
  std::unique_ptr<KeyframeSelector> keyframe_sel_;     ///< keyframe decision logic

  /// Cached map→odom transform. Recomputed only when optimisation runs.
  /// Re-published every timer tick so Nav2 does not time out.
  geometry_msgs::msg::TransformStamped  cached_map_to_odom_tf_;
  bool                                  has_map_to_odom_cache_{false};

  // ── Thread-safe sensor buffers ────────────────────────────────────────────
  SensorBuffer<OdomSample>                                        odom_buf_;
  SensorBuffer<ImuSample>                                         imu_buf_;
  SensorBuffer<geometry_msgs::msg::PoseWithCovarianceStamped>     scan_buf_;

  std::mutex graph_mutex_;  ///< protects iSAM2 state, optimized estimates, path

  // ── Publishers / Subscribers / Timer ─────────────────────────────────────
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                       sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr                        sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr                  sub_gps_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_scan_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_init_pose_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr     pub_path_;

  rclcpp::TimerBase::SharedPtr optimization_timer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster>  tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer>                tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>     tf_listener_;

  /// Cached rotation from IMU frame to base frame (looked up once at startup).
  gtsam::Rot3 R_imu_to_base_;
  bool        imu_tf_ready_{false};  ///< true once R_imu_to_base_ is valid

  // ── Path history (for /fgo/path) ─────────────────────────────────────────
  nav_msgs::msg::Path path_msg_;

  // ── Parameters ───────────────────────────────────────────────
  FgoConfig cfg_;
};

}  // namespace factor_graph_optimization
