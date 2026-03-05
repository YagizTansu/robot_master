#pragma once

#include <memory>
#include <mutex>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>

// ROS 2 messages
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
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

// GTSAM
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/linear/NoiseModel.h>

namespace factor_graph_optimization
{

using gtsam::symbol_shorthand::X;  // Pose3 keys: X(0), X(1), ...

/// One gyroscope sample captured by the IMU callback.
struct ImuSample
{
  rclcpp::Time timestamp;
  double gyro_z{0.0};
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
  void initGraph();          ///< Adds X(0) PriorFactor and commits to iSAM2

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

  // ── Conversion helpers ────────────────────────────────────────────────────
  static gtsam::Pose3 msgToGtsam(const geometry_msgs::msg::Pose & pose);
  static geometry_msgs::msg::Pose gtsamToMsg(const gtsam::Pose3 & pose);
  static gtsam::noiseModel::Diagonal::shared_ptr makeDiagonalNoise(
    double x, double y, double z,
    double roll, double pitch, double yaw);

  // ── iSAM2 state ───────────────────────────────────────────────────────────
  std::unique_ptr<gtsam::ISAM2>               isam2_;
  gtsam::NonlinearFactorGraph                 new_factors_;
  gtsam::Values                               new_values_;
  int                                         key_{0};       ///< current pose key index

  // ── Optimised pose (written by opt thread, read by TF publisher) ──────────
  gtsam::Pose3                                optimized_pose_;
  bool                                        has_optimized_pose_{false};

  // ── Pose tracking ─────────────────────────────────────────────────────────
  geometry_msgs::msg::Pose  last_keyframe_odom_pose_;  ///< last pose that opened a key
  geometry_msgs::msg::Pose  last_consumed_odom_pose_;  ///< last pose used for delta in opt
  geometry_msgs::msg::Pose  last_raw_odom_pose_;       ///< for odom→base TF

  /// Cached map→odom transform. Recomputed only when optimisation runs.
  /// Re-published every timer tick so Nav2 does not time out.
  geometry_msgs::msg::TransformStamped  cached_map_to_odom_tf_;
  bool                                  has_map_to_odom_cache_{false};

  // ── Thread-safe sensor buffers ────────────────────────────────────────────
  std::vector<geometry_msgs::msg::Pose>                          odom_buffer_;
  std::vector<ImuSample>                                         imu_buffer_;
  std::vector<geometry_msgs::msg::PoseWithCovarianceStamped>     scan_pose_buffer_;

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

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

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

  // TF flags
  bool publish_map_to_odom_{true};
  bool publish_odom_to_base_{true};

  // Initial pose
  double init_x_{0.0}, init_y_{0.0}, init_z_{0.0};
  double init_roll_{0.0}, init_pitch_{0.0}, init_yaw_{0.0};

  // Odometry noise
  double noise_odom_x_, noise_odom_y_, noise_odom_z_;
  double noise_odom_roll_, noise_odom_pitch_, noise_odom_yaw_;

  // IMU noise
  double noise_imu_gyro_z_sigma_;
  double noise_imu_x_, noise_imu_y_, noise_imu_z_;
  double noise_imu_roll_, noise_imu_pitch_;

  // LiDAR noise + gating
  double noise_lidar_x_, noise_lidar_y_, noise_lidar_z_;
  double noise_lidar_roll_, noise_lidar_pitch_, noise_lidar_yaw_;
  double icp_fitness_score_threshold_;
  double icp_covariance_threshold_;

  // iSAM2
  double isam2_relinearize_threshold_;
  int    isam2_relinearize_skip_;
  double optimization_rate_hz_;

  // Keyframe
  double keyframe_translation_threshold_;
  double keyframe_rotation_threshold_;
};

}  // namespace factor_graph_optimization
