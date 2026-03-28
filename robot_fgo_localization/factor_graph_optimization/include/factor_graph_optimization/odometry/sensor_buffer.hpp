#pragma once

#include <cstddef>
#include <deque>
#include <mutex>
#include <vector>

#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/pose.hpp>

// GTSAM types used by ImuSample
#include <gtsam/base/Vector.h>

namespace factor_graph_optimization
{

// ─────────────────────────────────────────────────────────────────────────────
// Sensor sample types
// ─────────────────────────────────────────────────────────────────────────────

/// Full 6-DOF IMU sample (accelerometer + gyroscope), expressed in base frame.
struct ImuSample
{
  rclcpp::Time   timestamp;
  gtsam::Vector3 accel{0.0, 0.0, 0.0};   ///< linear acceleration (m/s²) — gravity removed by MakeSharedU() in ImuPreintegrator
  gtsam::Vector3 gyro {0.0, 0.0, 0.0};   ///< angular velocity    (rad/s)
};

/// Odometry keyframe stored with its ROS timestamp for IMU alignment.
struct OdomSample
{
  geometry_msgs::msg::Pose pose;
  rclcpp::Time             timestamp;
};

// ─────────────────────────────────────────────────────────────────────────────
// SensorBuffer<T> — thread-safe FIFO, drainable in one swap
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Minimal thread-safe FIFO buffer for sensor samples.
 *
 * The intended usage pattern is:
 * @code
 *   // Producer (callback thread):
 *   buf_.push(sample, max_size);  // max_size 0 = unbounded
 *
 *   // Consumer (timer / optimizer thread):
 *   std::vector<T> local;
 *   buf_.drain(local);   // moves all elements into local in O(N)
 *   for (auto & s : local) { ... }
 * @endcode
 *
 * The internal store is `std::deque<T>` so that the FIFO cap path
 * (drop-oldest-on-overflow) runs in O(1) instead of the O(N) that
 * `std::vector::erase(begin())` would incur.
 *
 * @tparam T  Sample type.  Must be copyable.
 */
template <typename T>
class SensorBuffer
{
public:
  /**
   * @brief Append a sample to the back of the buffer.
   *
   * @param item      Sample to append.
   * @param max_size  If > 0, the oldest entry is dropped when the buffer
   *                  already has @p max_size elements (O(1) deque pop_front).
   *                  Pass 0 to disable the cap (unbounded — use with care).
   */
  void push(const T & item, std::size_t max_size = 0)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (max_size > 0 && buffer_.size() >= max_size) {
      buffer_.pop_front();  // O(1) on deque
    }
    buffer_.push_back(item);
  }

  /**
   * @brief Move all buffered samples into @p out.
   *
   * After the call the internal buffer is empty.
   * @p out is cleared first, so its previous contents are discarded.
   *
   * @param out  Destination vector (will be overwritten).
   */
  void drain(std::vector<T> & out)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    out.clear();
    out.assign(buffer_.begin(), buffer_.end());
    buffer_.clear();
  }

  /// Discard all buffered samples.
  void clear()
  {
    std::lock_guard<std::mutex> lk(mutex_);
    buffer_.clear();
  }

  /// Returns the number of buffered samples (snapshot — may be immediately stale).
  std::size_t size() const
  {
    std::lock_guard<std::mutex> lk(mutex_);
    return buffer_.size();
  }

private:
  std::deque<T>      buffer_;  ///< deque: O(1) push_back AND pop_front
  mutable std::mutex mutex_;
};

}  // namespace factor_graph_optimization
