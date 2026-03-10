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

// TF2
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// GTSAM — core
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/linear/NoiseModel.h>

// Core utilities (ROS-free math helpers)
#include "factor_graph_optimization/core/pose_conversion.hpp"
#include "factor_graph_optimization/core/noise_model.hpp"
#include "factor_graph_optimization/core/geometry_2d.hpp"

// Configuration struct
#include "factor_graph_optimization/config/fgo_config.hpp"

// Odometry module
#include "factor_graph_optimization/odometry/sensor_buffer.hpp"
#include "factor_graph_optimization/odometry/keyframe_selector.hpp"

// IMU module
#include "factor_graph_optimization/imu/imu_preintegrator.hpp"
// GTSAM — navigation / IMU preintegration
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <boost/shared_ptr.hpp>

namespace factor_graph_optimization
{

using gtsam::symbol_shorthand::X;  // Pose3 keys
using gtsam::symbol_shorthand::V;  // Velocity keys  (gtsam::Vector3)
using gtsam::symbol_shorthand::B;  // IMU-bias keys  (gtsam::imuBias::ConstantBias)

// ImuSample and OdomSample are defined in odometry/sensor_buffer.hpp.

class FgoNode : public rclcpp::Node
{
public:
  explicit FgoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── Parameter helpers ─────────────────────────────────────────────────────
  // Parameters are loaded via FgoConfig::fromNode() — see config/fgo_config.hpp.

  // ── Initialisation ────────────────────────────────────────────────────────
  void initIsam2();
  void initGraph();              ///< Adds X(0)/V(0)/B(0) PriorFactors and commits to iSAM2

  // ── Callbacks ─────────────────────────────────────────────────────────────
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
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

  // ── iSAM2 state ───────────────────────────────────────────────────────────
  std::unique_ptr<gtsam::ISAM2>               isam2_;
  gtsam::NonlinearFactorGraph                 new_factors_;
  gtsam::Values                               new_values_;
  int                                         key_{0};       ///< current pose key index

  // ── Optimised states (pose, velocity, IMU bias) ───────────────────────────
  gtsam::Pose3                         optimized_pose_;
  gtsam::Vector3                       optimized_velocity_{gtsam::Vector3::Zero()};
  gtsam::imuBias::ConstantBias         optimized_bias_{};

  /// IMU preintegrator — null when cfg_.enable_imu is false.
  std::unique_ptr<ImuPreintegrator> imu_preint_;

  // ── Pose / stamp tracking ─────────────────────────────────────────────────
  std::unique_ptr<KeyframeSelector> keyframe_sel_;     ///< keyframe decision logic
  geometry_msgs::msg::Pose  last_consumed_odom_pose_;  ///< keyframe pose corresponding to optimized_pose_
  rclcpp::Time              last_consumed_odom_stamp_;  ///< timestamp of last_consumed_odom_pose_

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
