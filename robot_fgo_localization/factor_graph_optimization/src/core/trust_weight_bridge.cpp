#include "factor_graph_optimization/core/trust_weight_bridge.hpp"

#include <chrono>

namespace factor_graph_optimization
{

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────

TrustWeightBridge::TrustWeightBridge(const rclcpp::NodeOptions & options)
: rclcpp::Node("trust_weight_bridge", options)
{
  // ── Parameters ───────────────────────────────────────────────────────────
  manual_mode_ = declare_parameter<bool>  ("manual_mode",       true);
  param_w_encoder_ = declare_parameter<double>("manual_w_encoder", 1.0);
  param_w_imu_     = declare_parameter<double>("manual_w_imu",     1.0);
  param_w_lidar_   = declare_parameter<double>("manual_w_lidar",   1.0);
  param_w_gps_     = declare_parameter<double>("manual_w_gps",     1.0);

  const std::string topic_in  = declare_parameter<std::string>(
    "topic_in",  "/transformer/trust_scores");
  const std::string topic_out = declare_parameter<std::string>(
    "topic_out", "/fgo/trust_weights");

  // Initialise active weights from the manual parameters (used in both modes
  // until the first Transformer message arrives when manual_mode = false).
  active_weights_ = buildManualWeights();

  // ── Subscriber ────────────────────────────────────────────────────────────
  // Always subscribe so that the node is ready to switch modes at runtime.
  sub_transformer_ = create_subscription<std_msgs::msg::Float64MultiArray>(
    topic_in, rclcpp::QoS(10),
    std::bind(&TrustWeightBridge::transformerCallback, this, std::placeholders::_1));

  // ── Publisher ─────────────────────────────────────────────────────────────
  pub_weights_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    topic_out, rclcpp::QoS(10));

  // ── 10 Hz republish timer ─────────────────────────────────────────────────
  // FgoNode's 50 Hz timer will always find a valid weight on /fgo/trust_weights.
  publish_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&TrustWeightBridge::publishTimer, this));

  // ── SetBool service for manual_mode toggle ────────────────────────────────
  // Sending request=true → manual mode; request=false → Transformer mode.
  set_mode_srv_ = create_service<std_srvs::srv::SetBool>(
    "/fgo/set_trust_weights",
    std::bind(&TrustWeightBridge::setModeService, this,
              std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(),
    "[TrustWeightBridge] Started. manual_mode=%s  in=%s  out=%s",
    manual_mode_ ? "true" : "false",
    topic_in.c_str(), topic_out.c_str());

  if (manual_mode_) {
    RCLCPP_INFO(get_logger(),
      "[TrustWeightBridge] Manual weights: encoder=%.4f  imu=%.4f  "
      "lidar=%.4f  gps=%.4f",
      param_w_encoder_, param_w_imu_, param_w_lidar_, param_w_gps_);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Subscriber callback — /transformer/trust_scores
// ─────────────────────────────────────────────────────────────────────────────

void TrustWeightBridge::transformerCallback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (manual_mode_) {
    // Silently discard while in manual mode; the node stays subscribed so
    // switching modes at runtime does not require a new subscription.
    return;
  }

  if (msg->data.size() < 4) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000 /*ms*/,
      "[TrustWeightBridge] /transformer/trust_scores has %zu elements; "
      "expected 4 [encoder, imu, lidar, gps]. Message ignored.",
      msg->data.size());
    return;
  }

  TrustWeights w;
  w.w_encoder = msg->data[0];
  w.w_imu     = msg->data[1];
  w.w_lidar   = msg->data[2];
  w.w_gps     = msg->data[3];
  w.clamp();  // enforce [EPSILON, 1.0] before storing

  {
    std::lock_guard<std::mutex> lk(weights_mutex_);
    active_weights_ = w;
  }

  // Publish immediately so FgoNode receives the update as fast as possible
  // rather than waiting for the next 10 Hz timer tick.
  publishWeights();

  RCLCPP_DEBUG(get_logger(),
    "[TrustWeightBridge] Transformer weights: encoder=%.4f  imu=%.4f  "
    "lidar=%.4f  gps=%.4f",
    w.w_encoder, w.w_imu, w.w_lidar, w.w_gps);
}

// ─────────────────────────────────────────────────────────────────────────────
// 10 Hz publish timer
// ─────────────────────────────────────────────────────────────────────────────

void TrustWeightBridge::publishTimer()
{
  // In manual mode, re-read the live parameter values on every tick so that
  // `ros2 param set` changes take effect immediately without restarting the node.
  if (manual_mode_) {
    // Re-read mutable parameters (operator may update them via ros2 param set)
    param_w_encoder_ = get_parameter("manual_w_encoder").as_double();
    param_w_imu_     = get_parameter("manual_w_imu").as_double();
    param_w_lidar_   = get_parameter("manual_w_lidar").as_double();
    param_w_gps_     = get_parameter("manual_w_gps").as_double();

    std::lock_guard<std::mutex> lk(weights_mutex_);
    active_weights_ = buildManualWeights();
  }
  publishWeights();
}

// ─────────────────────────────────────────────────────────────────────────────
// Service — /fgo/set_trust_weights  (toggle manual/Transformer mode)
// ─────────────────────────────────────────────────────────────────────────────

void TrustWeightBridge::setModeService(
  const std_srvs::srv::SetBool::Request::SharedPtr  req,
  std_srvs::srv::SetBool::Response::SharedPtr       res)
{
  const bool new_mode = req->data;  // true = manual, false = Transformer
  {
    std::lock_guard<std::mutex> lk(weights_mutex_);
    manual_mode_ = new_mode;
    if (manual_mode_) {
      active_weights_ = buildManualWeights();
    }
  }
  res->success = true;
  res->message = manual_mode_ ? "Switched to manual mode." : "Switched to Transformer mode.";
  RCLCPP_INFO(get_logger(), "[TrustWeightBridge] %s", res->message.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

void TrustWeightBridge::publishWeights()
{
  TrustWeights w_copy;
  {
    std::lock_guard<std::mutex> lk(weights_mutex_);
    w_copy = active_weights_;
  }
  pub_weights_->publish(toMsg(w_copy));
}

TrustWeights TrustWeightBridge::buildManualWeights() const
{
  TrustWeights w;
  w.w_encoder = param_w_encoder_;
  w.w_imu     = param_w_imu_;
  w.w_lidar   = param_w_lidar_;
  w.w_gps     = param_w_gps_;
  w.clamp();
  return w;
}

std_msgs::msg::Float64MultiArray TrustWeightBridge::toMsg(const TrustWeights & w)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {w.w_encoder, w.w_imu, w.w_lidar, w.w_gps};
  return msg;
}

}  // namespace factor_graph_optimization

// ── Standalone entry point ────────────────────────────────────────────────────
#include <rclcpp/rclcpp.hpp>
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<factor_graph_optimization::TrustWeightBridge>());
  rclcpp::shutdown();
  return 0;
}
