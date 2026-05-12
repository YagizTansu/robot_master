// ============================================================================
// pallet_detection.cpp
// ----------------------------------------------------------------------------
// EUR (EPAL-1) Palet Algılama Node'u — ROS2 Jazzy
//
// Yaklaşım: 2D LiDAR LaserScan üzerinde geometrik şablon eşleştirme.
//   1. Polygon ROI filtresi (station-relative arama bölgesi)
//   2. Adaptive Breakpoint Detector (Borges & Aldon, 2004) ile segmentasyon
//   3. PCA tabanlı doğru fit + cluster karakterizasyonu (width, flatness)
//   4. EUR bacak adayları sınıflandırma (corner ~100mm, center ~145mm)
//   5. 3'lü kombinasyon araması: kollineer + doğru aralık + boşluk temizliği
//   6. Pose çıkarımı (PCA line + LiDAR'a bakan normal)
//   7. Temporal N-of-M tutarlılık filtresi
//
// Sahada hata teşhisi için her aşama ayrı topic'e basılır.
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <robot_interfaces/msg/pallet_station.hpp>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <optional>
#include <vector>

namespace pallet_detection {

// ============================================================================
// EUR (EPAL-1) palet ön yüz geometrisi — derleme zamanı sabitleri
//
//   [köşe 100mm][boşluk 227.5mm][orta 145mm][boşluk 227.5mm][köşe 100mm]
//   |<------------------- 800 mm (ön yüz genişliği) ------------------>|
//
//   Bacak merkezleri (sol kenardan): 50, 400, 750 mm
//   Köşe → orta merkez mesafesi:     350 mm
//   Köşe → köşe merkez mesafesi:     700 mm
// ============================================================================
namespace eur {
constexpr double FACE_WIDTH                 = 0.800;
constexpr double CORNER_BLOCK_WIDTH         = 0.100;
constexpr double CENTER_BLOCK_WIDTH         = 0.145;
constexpr double CORNER_TO_CENTER_DISTANCE  = 0.350;
constexpr double CORNER_TO_CORNER_DISTANCE  = 0.700;
}  // namespace eur

// ============================================================================
// Yardımcı tipler
// ============================================================================

/// 2D LiDAR'dan gelen tek bir geçerli nokta (LiDAR frame'inde)
struct ScanPoint
{
    double  x = 0.0;
    double  y = 0.0;
    double  range = 0.0;
    double  angle = 0.0;
    size_t  scan_index = 0;  // orijinal LaserScan içindeki index — gap testi için
};

/// Bir tarama segmenti (komşu noktalar). Doğru fit edilmiş ve karakterize edilmiş.
struct Cluster
{
    std::vector<ScanPoint> points;

    Eigen::Vector2d centroid     = Eigen::Vector2d::Zero();
    Eigen::Vector2d line_dir     = Eigen::Vector2d::UnitX();  // birim, doğru boyunca
    Eigen::Vector2d line_normal  = Eigen::Vector2d::UnitY();  // birim, doğruya dik

    double width    = 0.0;   // doğru yönündeki projeksiyon uzunluğu
    double flatness = 0.0;   // doğru normaline maksimum sapma (mutlak)
    double range_to_centroid = 0.0;

    // Sınıflandırma
    bool is_corner_candidate = false;
    bool is_center_candidate = false;

    // İlgili scan indeksleri (boşluk testi için)
    size_t scan_index_min = 0;
    size_t scan_index_max = 0;
};

/// Onaylanmış bir palet algılaması (LiDAR frame'inde)
struct PalletDetection
{
    Eigen::Vector2d position      = Eigen::Vector2d::Zero();
    double          yaw           = 0.0;   // palet ön yüz normalinin açısı (LiDAR'a bakan)
    double          score         = 0.0;   // 0..1, eşleşme kalitesi
    rclcpp::Time    stamp;
};

// ============================================================================
// Node sınıfı
// ============================================================================

class PalletDetectorNode : public rclcpp::Node
{
public:
    PalletDetectorNode()
    : Node("pallet_detection")
    {
        declareParameters();

        // TF
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Aboneler — orijinal node ile aynı topic'ler
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/lidar_fork_1/scan", rclcpp::SensorDataQoS(),
            std::bind(&PalletDetectorNode::scanCallback, this, std::placeholders::_1));

        station_sub_ = this->create_subscription<robot_interfaces::msg::PalletStation>(
            "pallet_station_pose", 1,
            std::bind(&PalletDetectorNode::stationCallback, this, std::placeholders::_1));

        // Ana yayıncılar (orijinal arayüzle uyumlu)
        pose_array_pub_     = this->create_publisher<geometry_msgs::msg::PoseArray>(
                                    "/pallet_poses", 1);
        arrow_marker_pub_   = this->create_publisher<visualization_msgs::msg::Marker>(
                                    "convex_hull_marker", 1);
        station_area_pub_   = this->create_publisher<visualization_msgs::msg::Marker>(
                                    "filtered_lidar_area_marker", 1);

        // Diagnostic yayıncıları (yeni — saha debug için kritik)
        clusters_pub_       = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                                    "/pallet_detection/clusters", 1);
        leg_candidates_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                                    "/pallet_detection/leg_candidates", 1);
        accepted_pattern_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                                    "/pallet_detection/accepted_pattern", 1);

        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&PalletDetectorNode::parametersCallback, this, std::placeholders::_1));

        // Varsayılan arama polygon'u (station mesajı gelene kadar)
        search_polygon_ = {
            makePoint( 4.45, -25.3),
            makePoint( 4.45, -27.3),
            makePoint( 2.45, -27.3),
            makePoint( 2.45, -25.3)
        };

        RCLCPP_INFO(this->get_logger(),
            "EUR Pallet Detector aktif — geometrik şablon eşleştirme modu.");
    }

private:
    // ------------------------------------------------------------------------
    // Parametreler
    // ------------------------------------------------------------------------
    struct Params {
        // Frames
        std::string lidar_frame      = "lidar_fork_1";
        std::string reference_frame  = "map";

        // ROI
        double  min_range            = 0.30;
        double  max_range            = 3.50;
        double  fov_min              = -M_PI_2;   // -90°
        double  fov_max              =  M_PI_2;   //  90°

        // Station-relative bounding box (PalletStation pose etrafında)
        double  station_box_x_min    = -0.60;
        double  station_box_x_max    =  0.75;
        double  station_box_y_min    = -0.65;
        double  station_box_y_max    =  0.65;

        // Adaptive Breakpoint Detector
        double  abd_lambda           = 10.0 * M_PI / 180.0;  // worst-case grazing 10°
        double  abd_sigma_r          = 0.012;                // S2 mesafe gürültüsü ~12mm
        int     min_cluster_points   = 3;
        int     max_cluster_points   = 60;

        // Bacak sınıflandırma
        double  leg_width_tolerance  = 0.030;   // ±30mm
        double  leg_flatness_max     = 0.015;   // bacak yüzü düz olmalı (≤15mm)
        int     min_leg_points       = 4;
        double  leg_width_absolute_min = 0.060;
        double  leg_width_absolute_max = 0.200;

        // Pattern eşleştirme
        double  pattern_distance_tolerance = 0.040;  // ±40mm aralık toleransı
        double  pattern_collinearity_max   = 0.025;  // bacak merkezleri kollineer (≤25mm)
        bool    require_gap_clear          = true;
        double  gap_clear_margin           = 0.15;   // bacaktan en az 15cm daha uzakta

        // Temporal filter
        int     temporal_window      = 7;
        int     temporal_required    = 5;
        double  temporal_xy_jitter   = 0.025;   // 25mm
        double  temporal_yaw_jitter  = 0.05;    // ~3°
        double  temporal_timeout_sec = 1.0;     // bu süre içinde yenilenmezse historiyi temizle

        // Diagnostic
        bool    publish_diagnostics  = true;
    } p_;

    void declareParameters()
    {
        p_.lidar_frame     = this->declare_parameter("lidar_frame", p_.lidar_frame);
        p_.reference_frame = this->declare_parameter("reference_frame", p_.reference_frame);

        p_.min_range = this->declare_parameter("min_range", p_.min_range);
        p_.max_range = this->declare_parameter("max_range", p_.max_range);
        p_.fov_min   = this->declare_parameter("fov_min",   p_.fov_min);
        p_.fov_max   = this->declare_parameter("fov_max",   p_.fov_max);

        p_.station_box_x_min = this->declare_parameter("station_box_x_min", p_.station_box_x_min);
        p_.station_box_x_max = this->declare_parameter("station_box_x_max", p_.station_box_x_max);
        p_.station_box_y_min = this->declare_parameter("station_box_y_min", p_.station_box_y_min);
        p_.station_box_y_max = this->declare_parameter("station_box_y_max", p_.station_box_y_max);

        p_.abd_lambda         = this->declare_parameter("abd_lambda",         p_.abd_lambda);
        p_.abd_sigma_r        = this->declare_parameter("abd_sigma_r",        p_.abd_sigma_r);
        p_.min_cluster_points = this->declare_parameter("min_cluster_points", p_.min_cluster_points);
        p_.max_cluster_points = this->declare_parameter("max_cluster_points", p_.max_cluster_points);

        p_.leg_width_tolerance     = this->declare_parameter("leg_width_tolerance",     p_.leg_width_tolerance);
        p_.leg_flatness_max        = this->declare_parameter("leg_flatness_max",        p_.leg_flatness_max);
        p_.min_leg_points          = this->declare_parameter("min_leg_points",          p_.min_leg_points);
        p_.leg_width_absolute_min  = this->declare_parameter("leg_width_absolute_min",  p_.leg_width_absolute_min);
        p_.leg_width_absolute_max  = this->declare_parameter("leg_width_absolute_max",  p_.leg_width_absolute_max);

        p_.pattern_distance_tolerance = this->declare_parameter("pattern_distance_tolerance", p_.pattern_distance_tolerance);
        p_.pattern_collinearity_max   = this->declare_parameter("pattern_collinearity_max",   p_.pattern_collinearity_max);
        p_.require_gap_clear          = this->declare_parameter("require_gap_clear",          p_.require_gap_clear);
        p_.gap_clear_margin           = this->declare_parameter("gap_clear_margin",           p_.gap_clear_margin);

        p_.temporal_window      = this->declare_parameter("temporal_window",      p_.temporal_window);
        p_.temporal_required    = this->declare_parameter("temporal_required",    p_.temporal_required);
        p_.temporal_xy_jitter   = this->declare_parameter("temporal_xy_jitter",   p_.temporal_xy_jitter);
        p_.temporal_yaw_jitter  = this->declare_parameter("temporal_yaw_jitter",  p_.temporal_yaw_jitter);
        p_.temporal_timeout_sec = this->declare_parameter("temporal_timeout_sec", p_.temporal_timeout_sec);

        p_.publish_diagnostics = this->declare_parameter("publish_diagnostics", p_.publish_diagnostics);
    }

    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> & params)
    {
        for (const auto & pr : params) {
            const auto & n = pr.get_name();
            if      (n == "min_range")                 p_.min_range = pr.as_double();
            else if (n == "max_range")                 p_.max_range = pr.as_double();
            else if (n == "fov_min")                   p_.fov_min   = pr.as_double();
            else if (n == "fov_max")                   p_.fov_max   = pr.as_double();
            else if (n == "abd_lambda")                p_.abd_lambda = pr.as_double();
            else if (n == "abd_sigma_r")               p_.abd_sigma_r = pr.as_double();
            else if (n == "min_cluster_points")        p_.min_cluster_points = pr.as_int();
            else if (n == "max_cluster_points")        p_.max_cluster_points = pr.as_int();
            else if (n == "leg_width_tolerance")       p_.leg_width_tolerance = pr.as_double();
            else if (n == "leg_flatness_max")          p_.leg_flatness_max    = pr.as_double();
            else if (n == "min_leg_points")            p_.min_leg_points      = pr.as_int();
            else if (n == "pattern_distance_tolerance") p_.pattern_distance_tolerance = pr.as_double();
            else if (n == "pattern_collinearity_max")  p_.pattern_collinearity_max  = pr.as_double();
            else if (n == "require_gap_clear")         p_.require_gap_clear   = pr.as_bool();
            else if (n == "gap_clear_margin")          p_.gap_clear_margin    = pr.as_double();
            else if (n == "temporal_window")           p_.temporal_window     = pr.as_int();
            else if (n == "temporal_required")         p_.temporal_required   = pr.as_int();
            else if (n == "temporal_xy_jitter")        p_.temporal_xy_jitter  = pr.as_double();
            else if (n == "temporal_yaw_jitter")       p_.temporal_yaw_jitter = pr.as_double();
            else if (n == "publish_diagnostics")       p_.publish_diagnostics = pr.as_bool();
        }
        rcl_interfaces::msg::SetParametersResult r;
        r.successful = true;
        return r;
    }

    // ------------------------------------------------------------------------
    // ROS arayüzleri
    // ------------------------------------------------------------------------
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<robot_interfaces::msg::PalletStation>::SharedPtr station_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr        pose_array_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr      arrow_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr      station_area_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clusters_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr leg_candidates_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr accepted_pattern_pub_;

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // State
    std::vector<geometry_msgs::msg::Point> search_polygon_;  // map frame
    std::deque<PalletDetection>            history_;

    // ------------------------------------------------------------------------
    // Yardımcılar
    // ------------------------------------------------------------------------
    static geometry_msgs::msg::Point makePoint(double x, double y, double z = 0.0)
    {
        geometry_msgs::msg::Point p;
        p.x = x; p.y = y; p.z = z;
        return p;
    }

    static bool pointInPolygon(double x, double y,
                               const std::vector<geometry_msgs::msg::Point> & poly)
    {
        int crossings = 0;
        const size_t n = poly.size();
        for (size_t i = 0; i < n; ++i) {
            const auto & a = poly[i];
            const auto & b = poly[(i + 1) % n];
            if (((a.y > y) != (b.y > y)) &&
                (x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x)) {
                ++crossings;
            }
        }
        return (crossings & 1);
    }

    // ------------------------------------------------------------------------
    // Station pose callback — arama polygon'unu günceller
    // ------------------------------------------------------------------------
    void stationCallback(const robot_interfaces::msg::PalletStation::ConstSharedPtr & msg)
    {
        tf2::Quaternion q(
            msg->station_coor.pose.orientation.x,
            msg->station_coor.pose.orientation.y,
            msg->station_coor.pose.orientation.z,
            msg->station_coor.pose.orientation.w);
        tf2::Matrix3x3 R(q);

        const tf2::Vector3 t(
            msg->station_coor.pose.position.x,
            msg->station_coor.pose.position.y,
            0.0);

        const std::array<tf2::Vector3, 4> local = {
            tf2::Vector3(p_.station_box_x_max, p_.station_box_y_max, 0.0),
            tf2::Vector3(p_.station_box_x_max, p_.station_box_y_min, 0.0),
            tf2::Vector3(p_.station_box_x_min, p_.station_box_y_min, 0.0),
            tf2::Vector3(p_.station_box_x_min, p_.station_box_y_max, 0.0)
        };

        std::vector<geometry_msgs::msg::Point> poly;
        poly.reserve(4);
        for (const auto & v : local) {
            tf2::Vector3 w = R * v + t;
            poly.push_back(makePoint(w.x(), w.y()));
        }
        search_polygon_ = std::move(poly);
    }

    // ------------------------------------------------------------------------
    // Ana pipeline
    // ------------------------------------------------------------------------
    void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
    {
        publishStationArea();

        // TF lookup
        geometry_msgs::msg::TransformStamped tf_lidar_to_map;
        try {
            tf_lidar_to_map = tf_buffer_->lookupTransform(
                p_.reference_frame, p_.lidar_frame, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "TF lookup failed (%s → %s): %s",
                p_.lidar_frame.c_str(), p_.reference_frame.c_str(), ex.what());
            return;
        }

        // 1) ROI'deki noktaları çıkar (LiDAR frame'inde, scan ordering korunmuş)
        auto roi_points = extractROIPoints(*scan, tf_lidar_to_map);
        if (roi_points.size() < static_cast<size_t>(p_.min_leg_points * 3)) {
            RCLCPP_DEBUG(this->get_logger(),
                "ROI'de yetersiz nokta (%zu)", roi_points.size());
            clearHistoryIfTimedOut();
            return;
        }

        // 2) Adaptive Breakpoint Detector ile segmentasyon
        auto clusters = clusterAdaptiveBreakpoint(roi_points, scan->angle_increment);
        if (p_.publish_diagnostics) publishClusters(clusters);

        // 3) Her cluster için doğru fit + width/flatness karakterizasyonu
        for (auto & c : clusters) characterizeCluster(c);

        // 4) Bacak adayları (corner ~100mm, center ~145mm)
        auto leg_candidates = filterLegCandidates(clusters);
        if (p_.publish_diagnostics) publishLegCandidates(leg_candidates);

        if (leg_candidates.size() < 3) {
            RCLCPP_DEBUG(this->get_logger(),
                "Yetersiz bacak adayı (%zu)", leg_candidates.size());
            clearHistoryIfTimedOut();
            return;
        }

        // 5) EUR pattern eşleştirme (3 kollineer bacak + doğru aralık + boşluk temiz)
        auto detections = matchEURPattern(leg_candidates, roi_points);
        if (detections.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "EUR pattern eşleşmedi");
            clearHistoryIfTimedOut();
            return;
        }

        // En yüksek skorlu detection'ı al (genelde tek tane olur)
        auto best = *std::max_element(detections.begin(), detections.end(),
            [](const PalletDetection & a, const PalletDetection & b){
                return a.score < b.score;
            });
        best.stamp = this->now();

        // 6) Temporal tutarlılık filtresi
        auto confirmed = temporalFilter(best);
        if (!confirmed.has_value()) {
            RCLCPP_DEBUG(this->get_logger(),
                "Detection aday ama temporal tutarsız (history size=%zu)",
                history_.size());
            return;
        }

        // 7) Yayınla
        publishDetection(*confirmed);
    }

    // ------------------------------------------------------------------------
    // 1) ROI extraction
    // ------------------------------------------------------------------------
    std::vector<ScanPoint> extractROIPoints(
        const sensor_msgs::msg::LaserScan & scan,
        const geometry_msgs::msg::TransformStamped & tf_lidar_to_map)
    {
        std::vector<ScanPoint> out;
        out.reserve(scan.ranges.size() / 4);

        size_t total_valid = 0;
        size_t total_in_roi = 0;
        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            const float r = scan.ranges[i];

            // Range / geçerlilik
            if (!std::isfinite(r)) continue;
            if (r < std::max<double>(scan.range_min, p_.min_range)) continue;
            if (r > std::min<double>(scan.range_max, p_.max_range)) continue;

            // FOV
            const double a = scan.angle_min + i * scan.angle_increment;
            if (a < p_.fov_min || a > p_.fov_max) continue;

            // LiDAR frame'inde (x,y)
            ScanPoint sp;
            sp.range = r;
            sp.angle = a;
            sp.x = r * std::cos(a);
            sp.y = r * std::sin(a);
            sp.scan_index = i;

            // map frame'ine taşı, polygon kontrolü
            geometry_msgs::msg::PointStamped p_lidar, p_map;
            p_lidar.header.frame_id = p_.lidar_frame;
            p_lidar.point.x = sp.x;
            p_lidar.point.y = sp.y;
            p_lidar.point.z = 0.0;
            try {
                tf2::doTransform(p_lidar, p_map, tf_lidar_to_map);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN_ONCE(this->get_logger(), "extractROIPoints: TF dönüşüm hatası: %s", ex.what());
                continue;
            }

            ++total_valid;

            // DEBUG: Polygon testi devre dışı, tüm noktalar ROI'ye alınacak
            // if (!pointInPolygon(p_map.point.x, p_map.point.y, search_polygon_)) continue;

            ++total_in_roi;
            out.push_back(sp);
        }
        RCLCPP_DEBUG(this->get_logger(),
            "extractROIPoints: toplam nokta=%zu, geçerli=%zu, ROI'de=%zu",
            scan.ranges.size(), total_valid, total_in_roi);
        return out;
    }

    // ------------------------------------------------------------------------
    // 2) Adaptive Breakpoint Detector (Borges & Aldon 2004)
    //    Komşu noktalar arası beklenen maks mesafe:
    //      d_max(r, Δφ) = r * sin(Δφ) / sin(λ - Δφ) + 3·σ_r
    //    Bu eşiği aşan komşulukta breakpoint var → yeni cluster.
    // ------------------------------------------------------------------------
    std::vector<Cluster> clusterAdaptiveBreakpoint(
        const std::vector<ScanPoint> & pts, double angle_increment)
    {
        std::vector<Cluster> clusters;
        if (pts.empty()) return clusters;

        const double dphi = std::abs(angle_increment);
        const double lam  = p_.abd_lambda;
        const double sin_dphi = std::sin(dphi);
        const double sin_lam_minus_dphi = std::sin(lam - dphi);

        Cluster cur;
        cur.points.push_back(pts.front());

        for (size_t i = 1; i < pts.size(); ++i) {
            const auto & a = pts[i - 1];
            const auto & b = pts[i];

            // Açısal komşuluk: scan index'leri ardışık olmalı
            const bool angularly_adjacent =
                (b.scan_index - a.scan_index) <= 2;  // 1-2 boşluğa müsamaha

            const double dx = b.x - a.x;
            const double dy = b.y - a.y;
            const double dist = std::sqrt(dx * dx + dy * dy);

            const double r_min = std::min(a.range, b.range);
            const double d_max = (sin_lam_minus_dphi > 1e-6)
                                 ? r_min * sin_dphi / sin_lam_minus_dphi + 3.0 * p_.abd_sigma_r
                                 : 0.05;

            if (!angularly_adjacent || dist > d_max) {
                // Cluster'ı kapat
                if (cur.points.size() >= static_cast<size_t>(p_.min_cluster_points) &&
                    cur.points.size() <= static_cast<size_t>(p_.max_cluster_points)) {
                    clusters.push_back(std::move(cur));
                }
                cur = Cluster{};
            }
            cur.points.push_back(b);
        }
        if (cur.points.size() >= static_cast<size_t>(p_.min_cluster_points) &&
            cur.points.size() <= static_cast<size_t>(p_.max_cluster_points)) {
            clusters.push_back(std::move(cur));
        }
        return clusters;
    }

    // ------------------------------------------------------------------------
    // 3) Cluster karakterizasyonu — PCA ile doğru, width, flatness
    // ------------------------------------------------------------------------
    void characterizeCluster(Cluster & c)
    {
        const size_t N = c.points.size();
        if (N < 2) return;

        // Centroid
        Eigen::Vector2d mean = Eigen::Vector2d::Zero();
        for (const auto & p : c.points) mean += Eigen::Vector2d(p.x, p.y);
        mean /= static_cast<double>(N);
        c.centroid = mean;
        c.range_to_centroid = mean.norm();

        // Kovaryans
        Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
        for (const auto & p : c.points) {
            const Eigen::Vector2d d(p.x - mean.x(), p.y - mean.y());
            cov += d * d.transpose();
        }
        cov /= static_cast<double>(N);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
        // Büyük özdeğer → doğru yönü; küçük → normal
        c.line_dir    = es.eigenvectors().col(1).normalized();
        c.line_normal = es.eigenvectors().col(0).normalized();

        // Width = doğru yönündeki projeksiyon menzili
        double pmin =  std::numeric_limits<double>::infinity();
        double pmax = -std::numeric_limits<double>::infinity();
        double max_perp = 0.0;
        for (const auto & p : c.points) {
            const Eigen::Vector2d d(p.x - mean.x(), p.y - mean.y());
            const double t = d.dot(c.line_dir);
            const double n = std::abs(d.dot(c.line_normal));
            pmin = std::min(pmin, t);
            pmax = std::max(pmax, t);
            max_perp = std::max(max_perp, n);
        }
        c.width    = pmax - pmin;
        c.flatness = max_perp;

        // Scan index aralığı
        c.scan_index_min = c.points.front().scan_index;
        c.scan_index_max = c.points.back().scan_index;
        for (const auto & p : c.points) {
            c.scan_index_min = std::min(c.scan_index_min, p.scan_index);
            c.scan_index_max = std::max(c.scan_index_max, p.scan_index);
        }
    }

    // ------------------------------------------------------------------------
    // 4) Bacak aday filtresi
    // ------------------------------------------------------------------------
    std::vector<Cluster> filterLegCandidates(const std::vector<Cluster> & clusters)
    {
        std::vector<Cluster> out;
        out.reserve(clusters.size());

        for (auto c : clusters) {
            bool rejected = false;
            std::string reason;
            if (static_cast<int>(c.points.size()) < p_.min_leg_points) {
                rejected = true;
                reason += "min_leg_points ";
            }
            if (c.width < p_.leg_width_absolute_min) {
                rejected = true;
                reason += "width_min ";
            }
            if (c.width > p_.leg_width_absolute_max) {
                rejected = true;
                reason += "width_max ";
            }
            if (c.flatness > p_.leg_flatness_max) {
                rejected = true;
                reason += "flatness ";
            }

            const double tol = p_.leg_width_tolerance;
            c.is_corner_candidate =
                std::abs(c.width - eur::CORNER_BLOCK_WIDTH) <= tol;
            c.is_center_candidate =
                std::abs(c.width - eur::CENTER_BLOCK_WIDTH) <= tol;

            if (c.is_corner_candidate || c.is_center_candidate) {
                if (c.points.size() > 0) {
                    if (!rejected) {
                        out.push_back(std::move(c));
                        RCLCPP_DEBUG(rclcpp::get_logger("pallet_detection"),
                            "Bacak adayı: width=%.3f flatness=%.3f points=%zu [OK]",
                            c.width, c.flatness, c.points.size());
                    } else {
                        RCLCPP_DEBUG(rclcpp::get_logger("pallet_detection"),
                            "Bacak adayı: width=%.3f flatness=%.3f points=%zu [REJECTED: %s]",
                            c.width, c.flatness, c.points.size(), reason.c_str());
                    }
                } else {
                    RCLCPP_DEBUG(rclcpp::get_logger("pallet_detection"),
                        "Cluster width=%.3f flatness=%.3f points=%zu [NOT LEG]",
                        c.width, c.flatness, c.points.size());
                }
            } else {
                RCLCPP_DEBUG(rclcpp::get_logger("pallet_detection"),
                    "Cluster width=%.3f flatness=%.3f points=%zu [NOT LEG]",
                    c.width, c.flatness, c.points.size());
            }
        }
        return out;
    }

    // ------------------------------------------------------------------------
    // 5) EUR pattern eşleştirme — 3 kollineer bacak araması
    // ------------------------------------------------------------------------
    std::vector<PalletDetection> matchEURPattern(
        const std::vector<Cluster> & legs,
        const std::vector<ScanPoint> & all_points)
    {
        std::vector<PalletDetection> detections;
        const size_t M = legs.size();
        if (M < 3) return detections;

        const double D_nominal = eur::CORNER_TO_CENTER_DISTANCE;
        const double D_tol     = p_.pattern_distance_tolerance;

        // Tüm 3'lü kombinasyonlar
        for (size_t i = 0; i < M; ++i) {
            if (!legs[i].is_corner_candidate) continue;
            for (size_t k = i + 1; k < M; ++k) {
                if (!legs[k].is_corner_candidate) continue;

                // Köşe-köşe mesafesi ≈ 700 mm
                const double d_ik = (legs[i].centroid - legs[k].centroid).norm();
                if (std::abs(d_ik - eur::CORNER_TO_CORNER_DISTANCE) > 2.0 * D_tol)
                    continue;

                for (size_t j = 0; j < M; ++j) {
                    if (j == i || j == k) continue;
                    if (!legs[j].is_center_candidate) continue;

                    const double d_ij = (legs[i].centroid - legs[j].centroid).norm();
                    const double d_jk = (legs[j].centroid - legs[k].centroid).norm();

                    if (std::abs(d_ij - D_nominal) > D_tol) continue;
                    if (std::abs(d_jk - D_nominal) > D_tol) continue;

                    // Kollineerlik: j'nin (i,k) doğrusuna dik mesafesi
                    const Eigen::Vector2d ik = legs[k].centroid - legs[i].centroid;
                    const Eigen::Vector2d ij = legs[j].centroid - legs[i].centroid;
                    const double ik_len = ik.norm();
                    if (ik_len < 1e-6) continue;
                    const Eigen::Vector2d ik_hat = ik / ik_len;
                    const Eigen::Vector2d ik_perp(-ik_hat.y(), ik_hat.x());
                    const double perp = std::abs(ij.dot(ik_perp));
                    if (perp > p_.pattern_collinearity_max) continue;

                    // Boşluk temizliği
                    if (p_.require_gap_clear) {
                        if (!isGapClear(legs[i], legs[j], all_points)) continue;
                        if (!isGapClear(legs[j], legs[k], all_points)) continue;
                    }

                    // Skor: birden fazla kritere göre 0..1
                    double score = 1.0;
                    score *= 1.0 - std::abs(d_ij - D_nominal) / D_tol * 0.25;
                    score *= 1.0 - std::abs(d_jk - D_nominal) / D_tol * 0.25;
                    score *= 1.0 - perp / p_.pattern_collinearity_max * 0.25;
                    score = std::clamp(score, 0.0, 1.0);

                    // Pose çıkarımı
                    PalletDetection det = extractPose(legs[i], legs[j], legs[k]);
                    det.score = score;
                    detections.push_back(det);

                    if (p_.publish_diagnostics) {
                        publishAcceptedPattern({legs[i], legs[j], legs[k]}, det);
                    }
                }
            }
        }
        return detections;
    }

    // ------------------------------------------------------------------------
    // Boşluk temizlik kontrolü
    //   İki bacak arası scan açılarında, bacak mesafesine yakın (engelleyici)
    //   bir geri dönüş VAR mı? Varsa: gap dolu → reddet.
    // ------------------------------------------------------------------------
    bool isGapClear(const Cluster & a, const Cluster & b,
                    const std::vector<ScanPoint> & all_points)
    {
        // İki bacağın scan_index aralığı arasındaki noktaları al
        const size_t lo = std::min(a.scan_index_max, b.scan_index_max);
        const size_t hi = std::max(a.scan_index_min, b.scan_index_min);
        // Aslında "boşluk", iki bacağın index'lerinin arasındaki bölge:
        const size_t gap_lo = std::min(a.scan_index_max, b.scan_index_max);
        const size_t gap_hi = std::max(a.scan_index_min, b.scan_index_min);

        (void)lo; (void)hi;

        if (gap_hi <= gap_lo + 1) return true;  // bacaklar dokunuyor, boşluk yok sayılabilir

        const double leg_range = 0.5 * (a.range_to_centroid + b.range_to_centroid);
        const double max_allowed_range = leg_range - p_.gap_clear_margin;
        // Yani gap'te, leg_range'den >= gap_clear_margin daha YAKIN bir nokta varsa,
        // o nokta bacak yüzünün önünde — gap dolu demek. (palet arkasındaki dönüşler
        // daha uzakta olur, sorun değil)

        int blocking = 0;
        for (const auto & p : all_points) {
            if (p.scan_index <= gap_lo) continue;
            if (p.scan_index >= gap_hi) continue;
            if (p.range < max_allowed_range) {
                ++blocking;
                if (blocking > 2) return false;  // 1-2 outlier tolere et
            }
        }
        return true;
    }

    // ------------------------------------------------------------------------
    // Pose çıkarımı — 3 bacak merkezi üzerinden least-squares
    // ------------------------------------------------------------------------
    PalletDetection extractPose(const Cluster & a, const Cluster & b, const Cluster & c)
    {
        PalletDetection det;

        // Merkez: orta bacak konumu (b) tek başına en az gürültülü tahmin,
        //         ama 3 merkezin ortalaması yaw için daha kararlı.
        // Pratikte: yaw 3 merkezden line fit, position orta bacaktan.
        Eigen::Matrix<double, 3, 2> P;
        P << a.centroid.transpose(),
             b.centroid.transpose(),
             c.centroid.transpose();
        const Eigen::Vector2d mean = P.colwise().mean();

        Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
        for (int r = 0; r < 3; ++r) {
            const Eigen::Vector2d d(P(r, 0) - mean.x(), P(r, 1) - mean.y());
            cov += d * d.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
        const Eigen::Vector2d line_dir = es.eigenvectors().col(1).normalized();

        // Palet ön yüz normali: line_dir'e dik, LiDAR (origin) yönüne bakan
        Eigen::Vector2d normal(-line_dir.y(), line_dir.x());
        // Origin yönü = -mean (LiDAR origin'de, palet mean'de)
        if (normal.dot(-mean) < 0.0) normal = -normal;

        det.position = mean;
        det.yaw = std::atan2(normal.y(), normal.x());
        return det;
    }

    // ------------------------------------------------------------------------
    // 6) Temporal tutarlılık filtresi (N-of-M)
    // ------------------------------------------------------------------------
    std::optional<PalletDetection> temporalFilter(const PalletDetection & d)
    {
        const auto now = this->now();
        // Çok eski geçmişi at
        while (!history_.empty() &&
               (now - history_.front().stamp).seconds() > p_.temporal_timeout_sec) {
            history_.pop_front();
        }
        history_.push_back(d);
        while (static_cast<int>(history_.size()) > p_.temporal_window) {
            history_.pop_front();
        }

        // Mevcut detection ile uyumlu olan kaç geçmiş detection var?
        Eigen::Vector2d sum_pos = Eigen::Vector2d::Zero();
        double sum_sin = 0.0, sum_cos = 0.0;
        int consistent = 0;

        for (const auto & h : history_) {
            const double dxy = (h.position - d.position).norm();
            const double dyaw = std::atan2(
                std::sin(h.yaw - d.yaw), std::cos(h.yaw - d.yaw));
            if (dxy <= p_.temporal_xy_jitter &&
                std::abs(dyaw) <= p_.temporal_yaw_jitter) {
                sum_pos += h.position;
                sum_sin += std::sin(h.yaw);
                sum_cos += std::cos(h.yaw);
                ++consistent;
            }
        }

        if (consistent < p_.temporal_required) return std::nullopt;

        // Smoothed pose
        PalletDetection out = d;
        out.position = sum_pos / static_cast<double>(consistent);
        out.yaw      = std::atan2(sum_sin, sum_cos);
        out.score    = d.score *
            (static_cast<double>(consistent) /
             static_cast<double>(p_.temporal_window));
        return out;
    }

    void clearHistoryIfTimedOut()
    {
        if (history_.empty()) return;
        const auto now = this->now();
        if ((now - history_.back().stamp).seconds() > p_.temporal_timeout_sec) {
            history_.clear();
        }
    }

    // ------------------------------------------------------------------------
    // 7) Yayınlama — orijinal arayüze sadık
    // ------------------------------------------------------------------------
    void publishDetection(const PalletDetection & d)
    {
        // LiDAR frame'inde PoseStamped hazırla
        tf2::Quaternion q;
        q.setRPY(0, 0, d.yaw);
        geometry_msgs::msg::PoseStamped in_lidar;
        in_lidar.header.frame_id = p_.lidar_frame;
        in_lidar.header.stamp = rclcpp::Time(0);
        in_lidar.pose.position.x = d.position.x();
        in_lidar.pose.position.y = d.position.y();
        in_lidar.pose.position.z = 0.0;
        in_lidar.pose.orientation.x = q.x();
        in_lidar.pose.orientation.y = q.y();
        in_lidar.pose.orientation.z = q.z();
        in_lidar.pose.orientation.w = q.w();

        // map frame'ine taşı
        geometry_msgs::msg::PoseStamped in_map;
        try {
            tf_buffer_->transform(in_lidar, in_map, p_.reference_frame,
                                  tf2::durationFromSec(0.2));
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(),
                "Pose'u map'e taşıyamadım: %s", ex.what());
            return;
        }

        // /pallet_poses
        geometry_msgs::msg::PoseArray pa;
        pa.header.frame_id = p_.reference_frame;
        pa.header.stamp = this->now();
        pa.poses.push_back(in_map.pose);
        pose_array_pub_->publish(pa);

        // convex_hull_marker (arrow) — LiDAR frame'inde
        publishArrowMarker(d);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Palet onaylandı: pos=(%.3f, %.3f) yaw=%.1f° score=%.2f",
            d.position.x(), d.position.y(), d.yaw * 180.0 / M_PI, d.score);
    }

    void publishArrowMarker(const PalletDetection & d)
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = p_.lidar_frame;
        m.header.stamp = rclcpp::Time(0);
        m.ns = "pallet_pose";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::ARROW;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.05; m.scale.y = 0.10; m.scale.z = 0.15;
        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 1.0;
        m.pose.orientation.w = 1.0;

        geometry_msgs::msg::Point s, e;
        s.x = d.position.x(); s.y = d.position.y(); s.z = 0.0;
        e.x = s.x + 0.5 * std::cos(d.yaw);
        e.y = s.y + 0.5 * std::sin(d.yaw);
        e.z = 0.0;
        m.points.push_back(s);
        m.points.push_back(e);
        arrow_marker_pub_->publish(m);
    }

    void publishStationArea()
    {
        if (search_polygon_.size() < 3) return;
        visualization_msgs::msg::Marker m;
        m.header.frame_id = p_.reference_frame;
        m.header.stamp = this->now();
        m.ns = "search_area";
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.05;
        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0;
        m.pose.orientation.w = 1.0;
        for (const auto & p : search_polygon_) m.points.push_back(p);
        m.points.push_back(search_polygon_.front());
        station_area_pub_->publish(m);
    }

    // ------------------------------------------------------------------------
    // Diagnostic yayıncılar — saha debug için
    // ------------------------------------------------------------------------
    void publishClusters(const std::vector<Cluster> & clusters)
    {
        visualization_msgs::msg::MarkerArray arr;
        int id = 0;
        // Önce eski marker'ları sil
        visualization_msgs::msg::Marker del;
        del.header.frame_id = p_.lidar_frame;
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.push_back(del);

        for (const auto & c : clusters) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = p_.lidar_frame;
            m.header.stamp = rclcpp::Time(0);
            m.ns = "clusters";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::POINTS;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.scale.x = 0.03; m.scale.y = 0.03;
            m.color.r = 0.6; m.color.g = 0.6; m.color.b = 1.0; m.color.a = 1.0;
            m.pose.orientation.w = 1.0;
            for (const auto & p : c.points) {
                geometry_msgs::msg::Point gp;
                gp.x = p.x; gp.y = p.y; gp.z = 0.0;
                m.points.push_back(gp);
            }
            arr.markers.push_back(m);
        }
        clusters_pub_->publish(arr);
    }

    void publishLegCandidates(const std::vector<Cluster> & legs)
    {
        visualization_msgs::msg::MarkerArray arr;
        int id = 0;
        visualization_msgs::msg::Marker del;
        del.header.frame_id = p_.lidar_frame;
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.push_back(del);

        for (const auto & c : legs) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = p_.lidar_frame;
            m.header.stamp = rclcpp::Time(0);
            m.ns = "leg_candidates";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = c.centroid.x();
            m.pose.position.y = c.centroid.y();
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = m.scale.z = 0.08;
            // Yeşil: corner, Sarı: center, Turuncu: her ikisi de
            if (c.is_corner_candidate && c.is_center_candidate) {
                m.color.r = 1.0; m.color.g = 0.6; m.color.b = 0.0;
            } else if (c.is_corner_candidate) {
                m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0;
            } else {
                m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0;
            }
            m.color.a = 1.0;
            arr.markers.push_back(m);
        }
        leg_candidates_pub_->publish(arr);
    }

    void publishAcceptedPattern(const std::array<Cluster, 3> & legs,
                                const PalletDetection & d)
    {
        visualization_msgs::msg::MarkerArray arr;
        int id = 0;
        visualization_msgs::msg::Marker del;
        del.header.frame_id = p_.lidar_frame;
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.push_back(del);

        // 3 bacak merkezi
        for (const auto & c : legs) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = p_.lidar_frame;
            m.header.stamp = rclcpp::Time(0);
            m.ns = "accepted_legs";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::CYLINDER;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = c.centroid.x();
            m.pose.position.y = c.centroid.y();
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = std::max(0.05, c.width);
            m.scale.z = 0.05;
            m.color.r = 1.0; m.color.g = 0.0; m.color.b = 1.0; m.color.a = 0.7;
            arr.markers.push_back(m);
        }

        // Ön yüz çizgisi (3 bacağı birleştiren)
        visualization_msgs::msg::Marker line;
        line.header.frame_id = p_.lidar_frame;
        line.header.stamp = rclcpp::Time(0);
        line.ns = "accepted_face";
        line.id = id++;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.02;
        line.color.r = 1.0; line.color.g = 0.0; line.color.b = 1.0; line.color.a = 1.0;
        line.pose.orientation.w = 1.0;
        for (const auto & c : legs) {
            geometry_msgs::msg::Point p;
            p.x = c.centroid.x(); p.y = c.centroid.y(); p.z = 0.0;
            line.points.push_back(p);
        }
        arr.markers.push_back(line);

        (void)d;  // şu an için kullanılmıyor, ileride score text ekleyebiliriz
        accepted_pattern_pub_->publish(arr);
    }
};

}  // namespace pallet_detection

// ============================================================================
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pallet_detection::PalletDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}