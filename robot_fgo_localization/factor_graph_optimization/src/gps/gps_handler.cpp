#include "factor_graph_optimization/gps/gps_handler.hpp"

#include <cmath>
#include <stdexcept>

#include <sensor_msgs/msg/nav_sat_status.hpp>

// GeographicLib UTM projection (ROS2 ecosystem standard)
#include <GeographicLib/UTMUPS.hpp>

namespace factor_graph_optimization
{

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────

GpsHandler::GpsHandler(const FgoConfig & cfg,
                       rclcpp::Node & node,
                       const rclcpp::Logger & logger)
: cfg_(cfg), logger_(logger), clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
{
  // Validate GPS noise sigmas at construction time — a zero sigma would produce
  // a singular information matrix and crash iSAM2's Cholesky solver at runtime.
  if (cfg_.noise_gps_sigma_x <= 0.0 || cfg_.noise_gps_sigma_y <= 0.0) {
    throw std::invalid_argument(
      "[GpsHandler] noise.gps.sigma_x and noise.gps.sigma_y must be strictly positive. "
      "Current values: sigma_x=" + std::to_string(cfg_.noise_gps_sigma_x) +
      " sigma_y=" + std::to_string(cfg_.noise_gps_sigma_y));
  }

  // Validate hemisphere string — only "N" or "S" are valid.
  if (cfg_.gps_utm_hemisphere != "N" && cfg_.gps_utm_hemisphere != "S") {
    throw std::invalid_argument(
      "[GpsHandler] gps.utm_hemisphere must be \"N\" or \"S\". "
      "Got: \"" + cfg_.gps_utm_hemisphere + "\"");
  }

  if (!cfg_.enable_gps) {
    // Feature disabled — create no subscription.  Zero runtime overhead:
    // no callbacks fire, buffer stays empty, all callers see empty drain().
    RCLCPP_INFO(logger_, "[GpsHandler] GPS disabled (sensors.enable_gps=false). "
      "No subscription created.");
    return;
  }

  sub_gps_ = node.create_subscription<sensor_msgs::msg::NavSatFix>(
    cfg_.gps_topic,
    rclcpp::QoS(10),
    [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
      this->onNavSatFix(msg);
    });

  RCLCPP_INFO(logger_,
    "[GpsHandler] GPS enabled. topic=%s  min_fix_type=%d  "
    "hdop_threshold=%.2f  outlier_dist=%.2fm  "
    "sigma_x=%.3fm  sigma_y=%.3fm  "
    "utm_zone=%s  offset=(%.3f, %.3f)",
    cfg_.gps_topic.c_str(),
    cfg_.gps_min_fix_type,
    cfg_.gps_hdop_reject_threshold,
    cfg_.gps_outlier_reject_dist_m,
    cfg_.noise_gps_sigma_x, cfg_.noise_gps_sigma_y,
    (cfg_.gps_utm_zone == 0 ? "auto" : std::to_string(cfg_.gps_utm_zone).c_str()),
    cfg_.gps_offset_x, cfg_.gps_offset_y);
}

// ─────────────────────────────────────────────────────────────────────────────
// Primary callback
// ─────────────────────────────────────────────────────────────────────────────

void GpsHandler::onNavSatFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  // ── STEP 1: Fix type gate ─────────────────────────────────────────────────
  const int fix_type = remapFixType(msg->status.status);

  if (fix_type < cfg_.gps_min_fix_type) {
    std::lock_guard<std::mutex> lk(utm_state_mutex_);
    consecutive_rejects_++;
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000 /*ms*/,
      "[GpsHandler] Fix rejected — fix_type=%d < min=%d  "
      "(consecutive_rejects=%d)",
      fix_type, cfg_.gps_min_fix_type, consecutive_rejects_);
    return;
  }

  // ── STEP 2: HDOP extraction and gate ─────────────────────────────────────
  // Many ROS2 GPS drivers (ublox_gps, nmea_navsat_driver) store HDOP² in
  // position_covariance[0] when covariance_type == COVARIANCE_TYPE_APPROXIMATED.
  // We sqrt() it to recover HDOP.  If the result is ≤ 0 (unknown / not set),
  // we fall back to 1.0 rather than rejecting — the fix is still usable with
  // nominal noise.
  double hdop = 1.0;
  if (msg->position_covariance_type !=
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN)
  {
    const double cov_ee = msg->position_covariance[0];  // East σ² or HDOP²
    if (cov_ee > 0.0) {
      hdop = std::sqrt(cov_ee);
    } else {
      RCLCPP_DEBUG(logger_,
        "[GpsHandler] position_covariance[0]=%.6f (non-positive) — using hdop=1.0", cov_ee);
    }
  }

  if (cfg_.gps_hdop_reject_threshold > 0.0 && hdop > cfg_.gps_hdop_reject_threshold) {
    std::lock_guard<std::mutex> lk(utm_state_mutex_);
    consecutive_rejects_++;
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000 /*ms*/,
      "[GpsHandler] Fix rejected — HDOP=%.2f > threshold=%.2f  "
      "(consecutive_rejects=%d)",
      hdop, cfg_.gps_hdop_reject_threshold, consecutive_rejects_);
    return;
  }

  // ── STEP 3: NaN / Inf guard ───────────────────────────────────────────────
  if (!std::isfinite(msg->latitude) || !std::isfinite(msg->longitude)) {
    std::lock_guard<std::mutex> lk(utm_state_mutex_);
    consecutive_rejects_++;
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 10000 /*ms*/,
      "[GpsHandler] Fix rejected — non-finite lat=%.8f lon=%.8f  "
      "(consecutive_rejects=%d)",
      msg->latitude, msg->longitude, consecutive_rejects_);
    return;
  }

  // ── STEP 4 + STEP 5: UTM projection, datum lock, offset ──────────────────
  // All UTM state mutation is under utm_state_mutex_.
  double local_x = 0.0;
  double local_y = 0.0;
  {
    std::lock_guard<std::mutex> lk(utm_state_mutex_);

    const auto proj = projectToLocalUtm(*msg);
    if (!proj.has_value()) {
      consecutive_rejects_++;
      // projectToLocalUtm() already logged the error
      return;
    }

    local_x = proj->x;
    local_y = proj->y;

    // NOTE: GPS antenna → base_link offset is applied inside
    // GraphManager::addGpsBatch() where the current keyframe yaw is available
    // from new_values_.  We store the raw projected position here so the
    // handler stays free of graph-state dependencies.

    // ── STEP 6: Outlier rejection ─────────────────────────────────────────
    // NOTE: consecutive_rejects_ is also incremented by earlier gates (fix-type,
    // HDOP, NaN), but the adaptive baseline reset below only fires when an outlier-path
    // rejection pushes the count over the threshold — earlier paths return before
    // reaching this block. The YAML comment for gps_outlier_strike_limit documents
    // this distinction.
    if (has_accepted_once_ && cfg_.gps_outlier_reject_dist_m > 0.0) {
      const double dx   = local_x - last_accepted_x_;
      const double dy   = local_y - last_accepted_y_;
      const double dist = std::sqrt(dx * dx + dy * dy);

      if (dist > cfg_.gps_outlier_reject_dist_m) {
        consecutive_rejects_++;

        // Adaptive baseline reset: after gps_outlier_strike_limit consecutive
        // rejections the stale baseline is itself the problem (e.g. robot moved
        // while GPS was degraded, or graph was re-initialised without GPS reset).
        // Reset the baseline to the current fix so subsequent fixes can be accepted.
        if (consecutive_rejects_ >= cfg_.gps_outlier_strike_limit) {
          RCLCPP_WARN(logger_,
            "[GpsHandler] %d consecutive GPS outliers — resetting outlier baseline "
            "to current fix (x=%.3f y=%.3f). "
            "Possible causes: map re-initialisation, or GPS lost during motion.",
            consecutive_rejects_, local_x, local_y);
          last_accepted_x_    = local_x;
          last_accepted_y_    = local_y;
          last_accepted_stamp_ = rclcpp::Time(msg->header.stamp);
          consecutive_rejects_ = 0;
          // Fall through to buffer push below — this reset fix is accepted.
        } else {
          RCLCPP_WARN(logger_,
            "[GpsHandler] Outlier rejected — dist=%.2fm > threshold=%.2fm  "
            "last=(%.3f, %.3f) current=(%.3f, %.3f)  "
            "(consecutive_rejects=%d)",
            dist, cfg_.gps_outlier_reject_dist_m,
            last_accepted_x_, last_accepted_y_, local_x, local_y,
            consecutive_rejects_);
          return;
        }
      }
    }

    // ── STEP 7: Accept ────────────────────────────────────────────────────
    last_accepted_x_     = local_x;
    last_accepted_y_     = local_y;
    last_accepted_stamp_ = rclcpp::Time(msg->header.stamp);
    has_accepted_once_   = true;
    consecutive_rejects_ = 0;
  }  // unlock utm_state_mutex_

  // Clamp HDOP to floor of 1.0 before storing.  An over-determined GPS
  // constellation can produce HDOP < 1.0, which would give sigma < sigma_base,
  // trusting the measurement beyond its rated spec.
  const double hdop_clamped = std::max(hdop, 1.0);

  GpsSample sample;
  sample.timestamp = rclcpp::Time(msg->header.stamp);
  sample.x         = local_x;
  sample.y         = local_y;
  sample.hdop      = hdop_clamped;
  sample.fix_type  = fix_type;

  gps_buf_.push(sample, static_cast<std::size_t>(cfg_.max_pending_gps));

  RCLCPP_DEBUG(logger_,
    "[GpsHandler] Fix accepted — x=%.3f y=%.3f hdop=%.2f fix_type=%d",
    local_x, local_y, hdop_clamped, fix_type);
}

// ─────────────────────────────────────────────────────────────────────────────
// Buffer access
// ─────────────────────────────────────────────────────────────────────────────

void GpsHandler::drain(std::vector<GpsSample> & out)
{
  gps_buf_.drain(out);
}

// ─────────────────────────────────────────────────────────────────────────────
// State queries
// ─────────────────────────────────────────────────────────────────────────────

bool GpsHandler::hasValidGps() const
{
  std::lock_guard<std::mutex> lk(utm_state_mutex_);
  return utm_datum_locked_ && has_accepted_once_;
}

rclcpp::Time GpsHandler::lastAcceptedStamp() const
{
  std::lock_guard<std::mutex> lk(utm_state_mutex_);
  return last_accepted_stamp_;
}

// ─────────────────────────────────────────────────────────────────────────────
// Lifecycle
// ─────────────────────────────────────────────────────────────────────────────

void GpsHandler::reset()
{
  gps_buf_.clear();

  std::lock_guard<std::mutex> lk(utm_state_mutex_);
  utm_datum_locked_    = false;
  utm_locked_zone_     = 0;
  utm_datum_x_         = 0.0;
  utm_datum_y_         = 0.0;
  has_accepted_once_   = false;
  consecutive_rejects_ = 0;
  last_accepted_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  RCLCPP_WARN(logger_,
    "[GpsHandler] Reset — UTM datum cleared. "
    "Re-acquiring datum from next valid fix.");
}

// ─────────────────────────────────────────────────────────────────────────────
// UTM projection (private)
// ─────────────────────────────────────────────────────────────────────────────

std::optional<UtmProjection> GpsHandler::projectToLocalUtm(
  const sensor_msgs::msg::NavSatFix & fix)
{
  // Caller holds utm_state_mutex_.

  // Determine which UTM zone to request.
  // cfg_.gps_utm_zone == 0 → auto-detect on first call, then lock.
  // cfg_.gps_utm_zone 1–60 → always use the configured zone.
  int  zone_to_use = cfg_.gps_utm_zone;  // 0 = UTMUPS::STANDARD (auto)
  bool north       = (fix.latitude >= 0.0);

  if (cfg_.gps_utm_zone == 0 && utm_locked_zone_ != 0) {
    // Auto-detect was done on a previous call — enforce the locked zone.
    zone_to_use = utm_locked_zone_;
  }

  double easting  = 0.0;
  double northing = 0.0;
  int    detected_zone = 0;
  bool   northp        = false;

  try {
    // GeographicLib::UTMUPS::Forward with setzone:
    //   setzone = -1 (UTMUPS::STANDARD) → auto select zone from lon
    //   setzone =  0 (UTMUPS::UPS)      → UPS projection (polar regions)
    //   setzone = 1–60                  → force that UTM zone
    //
    // We map our "0 = auto" to GeographicLib's -1 (STANDARD).
    const int geolib_zone = (zone_to_use == 0) ? GeographicLib::UTMUPS::STANDARD
                                                : zone_to_use;

    GeographicLib::UTMUPS::Forward(
      fix.latitude, fix.longitude,
      detected_zone, northp,
      easting, northing,
      geolib_zone);

    north = northp;
  }
  catch (const GeographicLib::GeographicErr & e) {
    RCLCPP_ERROR(logger_,
      "[GpsHandler] GeographicLib UTM projection failed: %s  "
      "(lat=%.8f lon=%.8f zone_hint=%d)",
      e.what(), fix.latitude, fix.longitude, zone_to_use);
    return std::nullopt;
  }

  // ── Lock the zone on first auto-detected fix ─────────────────────────────
  if (utm_locked_zone_ == 0) {
    utm_locked_zone_ = detected_zone;
    RCLCPP_INFO(logger_,
      "[GpsHandler] UTM zone locked: %d%s  "
      "(first valid fix lat=%.6f lon=%.6f  E=%.3f N=%.3f)",
      utm_locked_zone_, (northp ? "N" : "S"),
      fix.latitude, fix.longitude, easting, northing);
  } else if (detected_zone != utm_locked_zone_) {
    // Fix arrived from a different zone than the locked one.
    // GeographicLib already projected it into zone_to_use = utm_locked_zone_,
    // so easting/northing are correct.  Log once for awareness.
    RCLCPP_DEBUG(logger_,
      "[GpsHandler] Fix zone %d != locked zone %d — "
      "projected into locked zone (coordinate accuracy degrades near boundary)",
      detected_zone, utm_locked_zone_);
  }

  // ── Set datum from the very first valid fix ───────────────────────────────
  if (!utm_datum_locked_) {
    utm_datum_x_     = easting;
    utm_datum_y_     = northing;
    utm_datum_locked_ = true;
    RCLCPP_INFO(logger_,
      "[GpsHandler] UTM datum set: E=%.3f N=%.3f (zone %d%s)",
      utm_datum_x_, utm_datum_y_, utm_locked_zone_, (northp ? "N" : "S"));
  }

  UtmProjection result;
  result.x     = easting  - utm_datum_x_;
  result.y     = northing - utm_datum_y_;
  result.zone  = utm_locked_zone_;
  result.north = north;
  return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers (private static)
// ─────────────────────────────────────────────────────────────────────────────

int GpsHandler::remapFixType(int8_t raw_status)
{
  // sensor_msgs::NavSatStatus raw values → internal 0/1/2 convention.
  // See header comment for full table.
  using S = sensor_msgs::msg::NavSatStatus;
  if (raw_status == S::STATUS_NO_FIX) { return 0; }
  if (raw_status == S::STATUS_FIX)    { return 1; }
  // STATUS_SBAS_FIX = 1, STATUS_GBAS_FIX = 2 → both map to internal 2 (DGPS)
  return 2;
}

}  // namespace factor_graph_optimization
