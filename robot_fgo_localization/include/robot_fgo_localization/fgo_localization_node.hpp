#pragma once

// ─── ROS2 ─────────────────────────────────────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
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

namespace robot_fgo_localization
{

using gtsam::symbol_shorthand::X;  // Poses: X(0), X(1), ...

/**
 * @brief Factor Graph Optimization localization node.
 *
 * Replaces: robot_localization EKF + nav2_amcl
 *
 * Inputs:
 *   /odometry          → nav_msgs/Odometry
 *   /imu               → sensor_msgs/Imu
 *   /scan              → sensor_msgs/LaserScan   (merged dual lidar)
 *   /map               → nav_msgs/OccupancyGrid  (from map_server)
 *   /initialpose       → geometry_msgs/PoseWithCovarianceStamped
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
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  // ─── Core FGO Logic ────────────────────────────────────────────────────────
  /**
   * @brief Main optimization step triggered on each odometry message.
   *
   * Steps:
   *  1. Compute delta pose from previous odometry.
   *  2. Add BetweenFactor (odometry).
   *  3. Accumulate IMU yaw constraint (if fresh).
   *  4. Accumulate scan-match factor (if fresh).
   *  5. Call isam_.update().
   *  6. Publish TFs + /fgo_pose.
   */
  void optimize();

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

  // ─── ROS2 Comms ────────────────────────────────────────────────────────────
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr              odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr                imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr          scan_sub_;
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
  std::optional<double>         latest_imu_yaw_;       // most recent IMU yaw (rad)
  bool                          imu_yaw_pending_{false};
  bool                          scan_factor_pending_{false};
  gtsam::Pose2                  pending_scan_pose_;
  std::mutex                    state_mutex_;

  // Auto-init timer: başlangıç pose gelmezse 5 sn sonra origin'den başla
  rclcpp::TimerBase::SharedPtr  auto_init_timer_;
  // 20 Hz TF yayıncısı: robot hareket etmese de map→odom ve odom→base_footprint sürekli yayınlı
  rclcpp::TimerBase::SharedPtr  tf_publish_timer_;

  // ─── Scan Matcher ──────────────────────────────────────────────────────────
  std::shared_ptr<ScanMatcher> scan_matcher_;

  // ─── Parameters ────────────────────────────────────────────────────────────
  std::string map_frame_{"map"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_footprint"};

  // Noise sigmas
  double odom_noise_x_{0.05};
  double odom_noise_y_{0.05};
  double odom_noise_yaw_{0.02};
  double imu_noise_yaw_{0.01};
  double scan_noise_x_{0.1};
  double scan_noise_y_{0.1};
  double scan_noise_yaw_{0.05};
  double scan_min_fitness_{0.85};

  // iSAM2
  double isam2_relinearize_threshold_{0.1};
  int    isam2_relinearize_skip_{10};

  // Initial covariance
  double initial_cov_x_{0.1};
  double initial_cov_y_{0.1};
  double initial_cov_yaw_{0.05};
};

}  // namespace robot_fgo_localization
