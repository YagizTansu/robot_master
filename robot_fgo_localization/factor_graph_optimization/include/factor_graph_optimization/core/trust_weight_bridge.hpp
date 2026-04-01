#pragma once

#include <array>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "factor_graph_optimization/core/trust_weights.hpp"

namespace factor_graph_optimization
{

/// ROS 2 node that bridges external Transformer trust scores into
/// the FGO pipeline's /fgo/trust_weights topic.
///
/// Two operating modes are selected via the `manual_mode` parameter:
///
///   manual_mode = true  (default)
///     Weights come from YAML parameters (manual_w_*).
///     Useful for tuning and integration testing before the Transformer
///     module is available.  The /transformer/trust_scores subscription
///     still exists but its data is discarded in this mode.
///
///   manual_mode = false
///     Weights come from the /transformer/trust_scores topic
///     (std_msgs/Float64MultiArray, 4 elements: [encoder, imu, lidar, gps]).
///     Incoming values are clamped to [EPSILON, 1.0] before publishing.
///
/// In both modes the node publishes the active TrustWeights on
/// /fgo/trust_weights at a fixed 10 Hz rate using a wall timer,
/// and immediately on each subscriber update so FgoNode always has
/// a fresh value.
///
/// A /fgo/set_trust_weights service (std_srvs/SetBool) is exposed for
/// toggling manual_mode at runtime without a node restart.
class TrustWeightBridge : public rclcpp::Node
{
public:
  explicit TrustWeightBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── Callbacks ────────────────────────────────────────────────────────────
  void transformerCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void publishTimer();
  void setModeService(
    const std_srvs::srv::SetBool::Request::SharedPtr  req,
    std_srvs::srv::SetBool::Response::SharedPtr       res);

  // ── Helpers ────────────────────────────────────────────────────────────
  void publishWeights();
  TrustWeights buildManualWeights() const;
  static std_msgs::msg::Float64MultiArray toMsg(const TrustWeights & w);

  // ── State ────────────────────────────────────────────────────────────────
  mutable std::mutex weights_mutex_;
  TrustWeights       active_weights_;   ///< latest clamped weights (protected by mutex)
  bool               manual_mode_;      ///< true = use YAML params, false = use topic

  // ── Manual-mode parameters ───────────────────────────────────────────────
  double param_w_encoder_;
  double param_w_imu_;
  double param_w_lidar_;
  double param_w_gps_;

  // ── ROS interfaces ─────────────────────────────────────────────────────
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_transformer_;
  rclcpp::Publisher   <std_msgs::msg::Float64MultiArray>::SharedPtr pub_weights_;
  rclcpp::TimerBase::SharedPtr                                       publish_timer_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr                 set_mode_srv_;
};

}  // namespace factor_graph_optimization
