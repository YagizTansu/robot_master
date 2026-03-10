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

/// Full 6-DOF IMU sample (accelerometer + gyroscope).
struct ImuSample
{
  rclcpp::Time   timestamp;
  gtsam::Vector3 accel{0.0, 0.0, 9.81};  ///< linear acceleration  (m/s²)
  gtsam::Vector3 gyro {0.0, 0.0, 0.0};   ///< angular velocity      (rad/s)
};

/// Odometry keyframe stored with its ROS timestamp for IMU alignment.
struct OdomSample
{
  geometry_msgs::msg::Pose pose;
  rclcpp::Time             timestamp;
};

class FgoNode : public rclcpp::Node
{
public:
  explicit FgoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── Parameter helpers ─────────────────────────────────────────────────────
  void declareParameters();
  void loadParameters();

  // ── Initialisation ────────────────────────────────────────────────────────
  void initIsam2();
  void initGraph();              ///< Adds X(0)/V(0)/B(0) PriorFactors and commits to iSAM2
  void initImuPreintegration();  ///< Builds PreintegrationCombinedParams from loaded noise params

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

  /// IMU preintegration params (built once from YAML, reused every step).
  boost::shared_ptr<gtsam::PreintegrationCombinedParams> imu_preint_params_;

  // ── Pose / stamp tracking ─────────────────────────────────────────────────
  geometry_msgs::msg::Pose  last_keyframe_odom_pose_;  ///< last pose that triggered a keyframe
  geometry_msgs::msg::Pose  last_consumed_odom_pose_;  ///< keyframe pose corresponding to optimized_pose_
  rclcpp::Time              last_consumed_odom_stamp_;  ///< timestamp of last_consumed_odom_pose_

  /// Cached map→odom transform. Recomputed only when optimisation runs.
  /// Re-published every timer tick so Nav2 does not time out.
  geometry_msgs::msg::TransformStamped  cached_map_to_odom_tf_;
  bool                                  has_map_to_odom_cache_{false};

  // ── Thread-safe sensor buffers ────────────────────────────────────────────
  std::vector<OdomSample>                                        odom_buffer_;
  std::vector<ImuSample>                                         imu_buffer_;
  std::vector<geometry_msgs::msg::PoseWithCovarianceStamped>     scan_pose_buffer_;

  std::mutex graph_mutex_;  ///< protects iSAM2 state, optimized estimates, path
  std::mutex odom_mutex_;
  std::mutex imu_mutex_;
  std::mutex scan_mutex_;

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

  // ── Parameters (loaded once at startup) ──────────────────────────────────
  // Sensor toggles
  bool enable_odom_{true};
  bool enable_imu_{true};
  bool enable_lidar_{true};

  // Topics
  std::string odom_topic_;
  std::string imu_topic_;
  std::string scan_match_pose_topic_;
  std::string initial_pose_topic_;

  // Frames
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string imu_frame_;   ///< frame_id of the IMU sensor

  // TF flags
  bool publish_map_to_odom_{true};
  bool publish_odom_to_base_{true};

  // Initial pose
  double init_x_{0.0}, init_y_{0.0}, init_z_{0.0};
  double init_roll_{0.0}, init_pitch_{0.0}, init_yaw_{0.0};

  // Odometry noise
  double noise_odom_x_, noise_odom_y_, noise_odom_z_;
  double noise_odom_roll_, noise_odom_pitch_, noise_odom_yaw_;

  // IMU preintegration noise
  double noise_imu_accel_sigma_;       ///< accelerometer white noise  σ (m/s²/√Hz)
  double noise_imu_gyro_sigma_;        ///< gyroscope white noise       σ (rad/s/√Hz)
  double noise_imu_accel_bias_sigma_;  ///< accelerometer bias RW       σ (m/s³/√Hz)
  double noise_imu_gyro_bias_sigma_;   ///< gyroscope bias RW           σ (rad/s²/√Hz)
  double noise_imu_integration_sigma_; ///< numerical integration noise σ
  double imu_gravity_;                 ///< gravity magnitude             (m/s²)

  // LiDAR gating (noise is read directly from scan_matcher's published covariance)
  double lidar_rotation_gate_rad_;    ///< skip scan prior if keyframe |dyaw| > this (rad)
  double max_scan_age_sec_;           ///< discard scan poses older than this (sec)

  // iSAM2
  double isam2_relinearize_threshold_;
  int    isam2_relinearize_skip_;
  double optimization_rate_hz_;

  // Keyframe
  double keyframe_translation_threshold_;
  double keyframe_rotation_threshold_;
};

}  // namespace factor_graph_optimization
