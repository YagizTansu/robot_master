#pragma once

// ─── ROS2 ─────────────────────────────────────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ─── GTSAM ────────────────────────────────────────────────────────────────────
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// ─── Local ────────────────────────────────────────────────────────────────────
#include "robot_fgo_localization/scan_matcher.hpp"

// ─── STL ──────────────────────────────────────────────────────────────────────
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace robot_fgo_localization
{

using gtsam::symbol_shorthand::X;  // Poses: X(0), X(1), ...

// ─── Sensor Config Structs ────────────────────────────────────────────────────
// Her sensör için YAML'dan okunan konfigürasyon.
// Yeni sensör eklemek = yeni bir struct + opsiyonel subscriber.

struct OdomConfig {
  bool        enabled{true};
  std::string topic{"/odometry"};
  double      noise_x{0.05};
  double      noise_y{0.05};
  double      noise_yaw{0.02};
};

struct ImuConfig {
  bool        enabled{true};
  std::string topic{"/imu"};
  double      noise_yaw{0.01};
};

struct LidarConfig {
  bool        enabled{true};
  std::string topic{"/scan"};
  double      noise_x{0.10};
  double      noise_y{0.10};
  double      noise_yaw{0.05};
  double      min_fitness{0.30};
};

struct GpsConfig {
  bool        enabled{false};
  std::string topic{"/gps/fix"};
  double      datum_lat{0.0};   // referans enlem → yerel (0, 0)
  double      datum_lon{0.0};   // referans boylam → yerel (0, 0)
  double      noise_x{2.0};     // metre
  double      noise_y{2.0};     // metre
};

/**
 * @brief Factor Graph Optimization localization node.
 *
 * Replaces: robot_localization EKF + nav2_amcl
 *
 * Inputs (configured via fgo_params.yaml — sensors.* block):
 *   sensors.odometry  → nav_msgs/Odometry        (BetweenFactor — backbone)
 *   sensors.imu       → sensor_msgs/Imu           (yaw PriorFactor, optional)
 *   sensors.lidar     → sensor_msgs/LaserScan     (ICP PriorFactor, optional)
 *   /map              → nav_msgs/OccupancyGrid    (map_server)
 *   /initialpose      → geometry_msgs/PoseWithCovarianceStamped
 *
 * Outputs:
 *   /fgo_pose          → geometry_msgs/PoseWithCovarianceStamped
 *   TF: map → odom
 *   TF: odom → base_footprint
 */
class FGOLocalizationNode : public rclcpp::Node
{
public:
  explicit FGOLocalizationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  ~FGOLocalizationNode() = default;

private:
  // ─── Initialization ────────────────────────────────────────────────────────
  void declareParameters();
  void initISAM2();
  void addPriorFactor(const gtsam::Pose2 & initial_pose);

  // ─── Callbacks ─────────────────────────────────────────────────────────────
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  // ─── TF Publishing ─────────────────────────────────────────────────────────
  void publishTFs(const gtsam::Pose2 & map_pose,
                  const gtsam::Pose2 & odom_pose,
                  const rclcpp::Time & stamp);
  void publishFGOPose(const gtsam::Pose2 & pose, const rclcpp::Time & stamp);

  // ─── Helpers ───────────────────────────────────────────────────────────────
  static gtsam::Pose2 odomToPose2(const nav_msgs::msg::Odometry & odom);
  static double quaternionToYaw(const geometry_msgs::msg::Quaternion & q);
  static geometry_msgs::msg::TransformStamped makeTransform(
    const std::string & parent, const std::string & child,
    double x, double y, double yaw, const rclcpp::Time & stamp);

  // ─── Sensor Configs (YAML-driven) ─────────────────────────────────────────
  OdomConfig  odom_cfg_;
  ImuConfig   imu_cfg_;
  LidarConfig lidar_cfg_;
  GpsConfig   gps_cfg_;

  // ─── ROS2 Comms ────────────────────────────────────────────────────────────
  // Subscriber'lar sadece config.enabled == true ise oluşturulur
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr              odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr                imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr          scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr          gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr         map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
                                                                        initial_pose_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr fgo_pose_pub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer>               tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;

  // ─── GTSAM ─────────────────────────────────────────────────────────────────
  gtsam::ISAM2                  isam_;
  gtsam::NonlinearFactorGraph   new_factors_;
  gtsam::Values                 new_values_;
  gtsam::Values                 current_estimate_;
  uint64_t                      pose_key_{0};
  bool                          graph_initialized_{false};

  // ─── State ─────────────────────────────────────────────────────────────────
  std::optional<gtsam::Pose2>   last_odom_pose_;      // previous odom reading (for delta)
  gtsam::Pose2                  last_raw_odom_pose_;   // raw odom pose for odom→base_footprint TF
  gtsam::Pose2                  last_map_pose_;        // last FGO result for map→odom TF

  // IMU pending state
  std::optional<double>         latest_imu_yaw_;
  bool                          imu_yaw_pending_{false};

  // Lidar pending state
  bool                          scan_factor_pending_{false};
  gtsam::Pose2                  pending_scan_pose_;

  // GPS pending state
  bool                          gps_factor_pending_{false};
  gtsam::Pose2                  pending_gps_pose_;  // yaw=0, sadece x/y kısıtı

  std::mutex                    state_mutex_;

  // Timers
  rclcpp::TimerBase::SharedPtr  auto_init_timer_;
  rclcpp::TimerBase::SharedPtr  tf_publish_timer_;

  // ─── Scan Matcher ──────────────────────────────────────────────────────────
  std::shared_ptr<ScanMatcher> scan_matcher_;

  // ─── Frame IDs ─────────────────────────────────────────────────────────────
  std::string map_frame_{"map"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_footprint"};

  // ─── iSAM2 params ──────────────────────────────────────────────────────────
  double isam2_relinearize_threshold_{0.1};
  int    isam2_relinearize_skip_{10};

  // ─── Initial covariance ────────────────────────────────────────────────────
  double initial_cov_x_{0.5};
  double initial_cov_y_{0.5};
  double initial_cov_yaw_{0.2};
};

}  // namespace robot_fgo_localization
