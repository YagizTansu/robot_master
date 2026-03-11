#pragma once

#include <mutex>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <gtsam/geometry/Point3.h>
#include <gtsam/linear/NoiseModel.h>

#include "factor_graph_optimization/config/fgo_config.hpp"
#include "factor_graph_optimization/odometry/sensor_buffer.hpp"  // SensorBuffer<T>

namespace factor_graph_optimization
{

// ─────────────────────────────────────────────────────────────────────────────
// Data types
// ─────────────────────────────────────────────────────────────────────────────

/// One GPS measurement after UTM projection and datum subtraction.
///
/// Stored in local map-frame coordinates (metres).
/// Raw lat/lon is discarded after projection — only local XY is retained
/// to avoid floating-point cancellation in the GTSAM residual computation
/// (raw UTM easting ~600 000 m; local map residuals are at 3 m scale).
struct GpsSample
{
  rclcpp::Time timestamp;   ///< stamp copied from NavSatFix header
  double x{0.0};            ///< local UTM easting  (metres, datum subtracted)
  double y{0.0};            ///< local UTM northing (metres, datum subtracted)
  double hdop{1.0};         ///< horizontal dilution of precision (dimensionless; ≥ 1.0 after clamping)
  int    fix_type{0};       ///< remapped quality: 0=none, 1=fix, 2=DGPS/SBAS
};

/// Thread-safe FIFO buffer for GPS samples — direct use of SensorBuffer<T>.
/// No specialisation is needed: GPS does not require preintegration between
/// arbitrary timestamps and drain-then-match is the correct pattern.
using GpsBuffer = SensorBuffer<GpsSample>;

// ─────────────────────────────────────────────────────────────────────────────
// UTM projection result
// ─────────────────────────────────────────────────────────────────────────────

/// Intermediate result of one NavSatFix → local-UTM projection.
/// Consumed internally by GpsHandler; not exposed outside the GPS subsystem.
struct UtmProjection
{
  double x{0.0};   ///< local easting  = UTM_easting  − datum_easting  (metres)
  double y{0.0};   ///< local northing = UTM_northing − datum_northing (metres)
  int    zone{0};  ///< UTM zone used for projection  (1–60)
  bool   north{true}; ///< true = northern hemisphere
};

// ─────────────────────────────────────────────────────────────────────────────
// GpsHandler
// ─────────────────────────────────────────────────────────────────────────────

/// Subscribes to NavSatFix, gates/projects each fix, and maintains a
/// thread-safe GpsBuffer for the optimizer to drain each cycle.
///
/// ## NavSatStatus convention mapping
///
/// `sensor_msgs::msg::NavSatStatus` uses a different scale than the
/// `gps_min_fix_type` config parameter:
///
///   NavSatStatus constant   | raw value | internal fix_type
///   STATUS_NO_FIX            |   -1      |   0   (no fix)
///   STATUS_FIX               |    0      |   1   (standard GPS fix)
///   STATUS_SBAS_FIX          |    1      |   2   (SBAS / WAAS corrected)
///   STATUS_GBAS_FIX          |    2      |   2   (ground-based correction)
///
/// `gps_min_fix_type` threshold uses the internal 0/1/2 enumeration.
/// A `gps_min_fix_type = 1` therefore accepts standard GPS fixes and above.
///
/// ## HDOP source
///
/// `NavSatFix.position_covariance_type == COVARIANCE_TYPE_DIAGONAL_KNOWN`
/// → covariance[0] is σ²_E (m²), not HDOP.
///
/// `NavSatFix.position_covariance_type == COVARIANCE_TYPE_APPROXIMATED`
/// → many drivers (ublox, nmea_navsat_driver) store scalar HDOP × HDOP in
///   covariance[0] as a documented convention.
///
/// This class reads `covariance[0]` as the raw HDOP² value and takes its
/// square root.  If the resulting value is ≤ 0 (unknown), it defaults to 1.0.
/// Users should verify the convention used by their GPS driver.
///
/// ## Thread safety
///
/// `onNavSatFix()` runs on the ROS executor thread.
/// `drain()` runs on the optimization timer thread.
/// All shared state (`utm_state_mutex_`, `GpsBuffer` internal mutex) is
/// individually protected.
class GpsHandler
{
public:
  /// @param cfg     Full FgoConfig (GPS fields are read from it).
  /// @param node    ROS2 node used to create the GPS subscription.
  /// @param logger  Logger passed from FgoNode — consistent diagnostic source.
  GpsHandler(const FgoConfig & cfg,
             rclcpp::Node & node,
             const rclcpp::Logger & logger);

  // ── Primary callback ───────────────────────────────────────────────────────

  /// Process one incoming NavSatFix message.
  ///
  /// Runs the full pipeline:
  ///   fix-type gate → HDOP gate → NaN guard → UTM projection →
  ///   outlier rejection → buffer push
  ///
  /// All rejection paths emit RCLCPP_WARN or RCLCPP_WARN_THROTTLE.
  /// Silent drops do not occur.
  void onNavSatFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  // ── Buffer access ──────────────────────────────────────────────────────────

  /// Move all buffered GpsSamples into @p out (clears internal buffer).
  /// Pass @p out directly to GraphManager::step() as @p local_gps.
  void drain(std::vector<GpsSample> & out);

  // ── Query helpers (called from GraphManager::addGpsBatch) ─────────────────

  /// Build a HDOP-scaled Diagonal noise model for a GPSFactor.
  ///
  /// sigma_x = max(hdop, 1.0) * cfg_.noise_gps_sigma_x
  /// sigma_y = max(hdop, 1.0) * cfg_.noise_gps_sigma_y
  /// sigma_z = 999.0   (unconstrained Z — 2D robot convention)
  ///
  /// @param hdop  Raw HDOP from GpsSample (already clamped to ≥ 1.0 in handler).
  gtsam::SharedNoiseModel buildNoiseModel(double hdop) const;

  // ── State queries ──────────────────────────────────────────────────────────

  /// True once the UTM datum is locked AND at least one fix has been accepted.
  /// Must be true before any GPS factor is added to the graph.
  bool hasValidGps() const;

  /// Timestamp of the last fix that passed all gates and was pushed to buffer.
  /// Returns rclcpp::Time(0) if no fix has ever been accepted.
  rclcpp::Time lastAcceptedStamp() const;

  // ── Lifecycle ──────────────────────────────────────────────────────────────

  /// Reset all GPS state (UTM datum, outlier baseline, buffer).
  ///
  /// Must be called from FgoNode::initialPoseCallback() in sync with
  /// graph_mgr_->reinit() to keep the GPS coordinate frame consistent
  /// with the newly set map origin.
  void reset();

private:
  // ── UTM conversion (implementation in .cpp) ───────────────────────────────

  /// Project a NavSatFix to local UTM coordinates.
  ///
  /// Locks utm_locked_zone_ on first call.
  /// All subsequent calls project into the locked zone for consistency.
  ///
  /// @return nullopt if the fix has NaN coordinates or GeographicLib throws.
  std::optional<UtmProjection> projectToLocalUtm(
    const sensor_msgs::msg::NavSatFix & fix);

  // ── Helpers ───────────────────────────────────────────────────────────────

  /// Remap sensor_msgs NavSatStatus raw value to internal 0/1/2 convention.
  static int remapFixType(int8_t raw_status);

  // ── Configuration ────────────────────────────────────────────────────────
  const FgoConfig cfg_;
  rclcpp::Logger  logger_;

  // ── Subscription ─────────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;

  // ── Sample buffer (internally mutex-protected by SensorBuffer) ────────────
  GpsBuffer gps_buf_;

  // ── UTM datum + outlier state (all protected by utm_state_mutex_) ─────────
  mutable std::mutex utm_state_mutex_;

  double utm_datum_x_{0.0};         ///< UTM easting  of map origin (m)
  double utm_datum_y_{0.0};         ///< UTM northing of map origin (m)
  int    utm_locked_zone_{0};       ///< 0 = not yet determined
  bool   utm_datum_locked_{false};  ///< true once set from first valid fix

  double       last_accepted_x_{0.0};
  double       last_accepted_y_{0.0};
  bool         has_accepted_once_{false};
  int          consecutive_rejects_{0};
  rclcpp::Time last_accepted_stamp_{0, 0, RCL_ROS_TIME};
};

}  // namespace factor_graph_optimization
