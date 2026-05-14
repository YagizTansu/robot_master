#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <visualization_msgs/msg/marker.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/time.h>
#include <cmath>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <robot_interfaces/msg/pallet_station.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <pallet_detection_and_docking/Quaternion.h>
#include <pcl/filters/statistical_outlier_removal.h>

/**
 * @brief PalletDetection class for pallet detection using LIDAR data
 *
 * This class processes LIDAR scan data to detect pallets using DBSCAN clustering.
 * It filters points within a specified area and identifies potential pallet shapes
 * using convex hull and minimum area rectangle detection.
 */

class PalletDetection : public rclcpp::Node
{
public:
    PalletDetection()
    : Node("pallet_detection"),
      cluster_tolerance(0.25),
      min_pallet_cluster_size(10),
      max_pallet_cluster_size(2000),
      mean_k(50),
      stddev_mul_thresh(0.1),
      min_long_side_length(0.6),
      max_long_side_length(1.0),
      min_short_side_length(0.65),
      max_short_side_length(1.0)
    {
        // TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscribers
        lidar_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/lidar_fork_1/scan", 1,
            std::bind(&PalletDetection::scanCallback, this, std::placeholders::_1));  // Forklift LIDAR topic

        pallet_station_subscriber = this->create_subscription<robot_interfaces::msg::PalletStation>(
            "pallet_station_pose", 1,
            std::bind(&PalletDetection::handlePalletStationPoseMessage, this, std::placeholders::_1));  // Pallet station pose topic

        // Publishers
        pallet_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("convex_hull_marker", 1);                      // Convex hull marker
        pallet_station_area_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("filtered_lidar_area_marker", 1); // Filtered LIDAR area marker
        pallet_pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/pallet_poses", 1);                        // Detected pallet poses


        // Declare parameters (replaces dynamic_reconfigure)
        this->declare_parameter("cluster_tolerance", 0.25);
        this->declare_parameter("min_pallet_cluster_size", 10);
        this->declare_parameter("max_pallet_cluster_size", 2000);
        this->declare_parameter("mean_k", 50);
        this->declare_parameter("stddev_mul_thresh", 0.1);
        this->declare_parameter("min_long_side_length", 0.6);
        this->declare_parameter("max_long_side_length", 1.0);
        this->declare_parameter("min_short_side_length", 0.65);
        this->declare_parameter("max_short_side_length", 1.0);

        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&PalletDetection::parametersCallback, this, std::placeholders::_1));

        // Define static polygon points in the map frame
        pallet_station_area = {
            createPoint( 0.05, 7.8, 0.0),
            createPoint( 0.05, 9.4, 0.0),
            createPoint(-1.45, 9.4, 0.0),
            createPoint(-1.45, 7.8, 0.0)};

        RCLCPP_INFO(this->get_logger(), "Pallet Detection Node Started...");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_scan_sub;                    // LIDAR scan subscriber
    rclcpp::Subscription<robot_interfaces::msg::PalletStation>::SharedPtr pallet_station_subscriber;    // Pallet station subscriber

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pallet_marker_pub;               // Convex hull marker publisher
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pallet_station_area_marker_pub;  // Filtered LIDAR area marker publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pallet_pose_array_pub_;            // Detected pallet poses publisher

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;      // Parameter callback handle


    // Parameters (replaces dynamic_reconfigure) - adjust if pallet dimensions change via ros2 param set
    float cluster_tolerance;       // DBSCAN clustering tolerance
    int min_pallet_cluster_size;   // Minimum pallet cluster size
    int max_pallet_cluster_size;   // Maximum pallet cluster size
    int mean_k;                   // SOR mean_k
    double stddev_mul_thresh;     // SOR stddev_mul_thresh
    double min_long_side_length;  // Rectangle min long side
    double max_long_side_length;  // Rectangle max long side
    double min_short_side_length; // Rectangle min short side
    double max_short_side_length; // Rectangle max short side

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                              // TF2 buffer
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;                 // TF2 listener
    std::vector<geometry_msgs::msg::Point> pallet_station_area;               // Pallet station area polygon

    /**
     * @brief Callback function for parameter updates (replaces dynamic_reconfigure)
     * @param parameters Updated parameters
     * @return SetParametersResult
     */
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        for (const auto &param : parameters)
        {
            if (param.get_name() == "cluster_tolerance")
            {
                this->cluster_tolerance = static_cast<float>(param.as_double());
            }
            else if (param.get_name() == "min_pallet_cluster_size")
            {
                this->min_pallet_cluster_size = static_cast<int>(param.as_int());
            }
            else if (param.get_name() == "max_pallet_cluster_size")
            {
                this->max_pallet_cluster_size = static_cast<int>(param.as_int());
            }
            else if (param.get_name() == "mean_k")
            {
                this->mean_k = static_cast<int>(param.as_int());
            }
            else if (param.get_name() == "stddev_mul_thresh")
            {
                this->stddev_mul_thresh = param.as_double();
            }
            else if (param.get_name() == "min_long_side_length")
            {
                this->min_long_side_length = param.as_double();
            }
            else if (param.get_name() == "max_long_side_length")
            {
                this->max_long_side_length = param.as_double();
            }
            else if (param.get_name() == "min_short_side_length")
            {
                this->min_short_side_length = param.as_double();
            }
            else if (param.get_name() == "max_short_side_length")
            {
                this->max_short_side_length = param.as_double();
            }
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    /**
     * @brief Handles pallet station pose message and updates the search area polygon
     * @param msg Incoming PalletStation message
     */
    void handlePalletStationPoseMessage(const robot_interfaces::msg::PalletStation::ConstSharedPtr &msg)
    {
        // Convert quaternion to rotation matrix
        tf2::Quaternion quat(msg->station_coor.pose.orientation.x, msg->station_coor.pose.orientation.y,
                             msg->station_coor.pose.orientation.z, msg->station_coor.pose.orientation.w);
        tf2::Matrix3x3 rotation_matrix(quat);

        // Define the local rectangle corners (centered at the origin)
        std::vector<tf2::Vector3> local_corners = {
            tf2::Vector3( 0.75,  0.65, 0.0),  // Top-right
            tf2::Vector3( 0.75, -0.65, 0.0),  // Bottom-right
            tf2::Vector3(-0.6,  -0.65, 0.0),  // Bottom-left
            tf2::Vector3(-0.6,   0.65, 0.0)   // Top-left
        };

        // Transform local corners to global frame
        std::vector<tf2::Vector3> pallet_station_area_corners;
        for (const auto &corner : local_corners)
        {
            tf2::Vector3 transformed_corner = rotation_matrix * corner;
            transformed_corner += tf2::Vector3(msg->station_coor.pose.position.x, msg->station_coor.pose.position.y, 0.0);
            pallet_station_area_corners.push_back(transformed_corner);
        }

        // Update the pallet station area polygon
        this->pallet_station_area = {
            createPoint(pallet_station_area_corners[0].x(), pallet_station_area_corners[0].y(), 0.0),
            createPoint(pallet_station_area_corners[1].x(), pallet_station_area_corners[1].y(), 0.0),
            createPoint(pallet_station_area_corners[2].x(), pallet_station_area_corners[2].y(), 0.0),
            createPoint(pallet_station_area_corners[3].x(), pallet_station_area_corners[3].y(), 0.0)};
    }

    /**
     * @brief Creates a geometry_msgs::msg::Point object
     * @param x X coordinate
     * @param y Y coordinate
     * @param z Z coordinate
     * @return geometry_msgs::msg::Point object
     */
    geometry_msgs::msg::Point createPoint(double x, double y, double z)
    {
        geometry_msgs::msg::Point point;
        point.x = x;
        point.y = y;
        point.z = z;
        return point;
    }

    /**
     * @brief Checks if a point is inside a polygon (ray casting algorithm)
     * @param point Point to check
     * @param polygon Polygon points
     * @return True if the point is inside the polygon, false otherwise
     */
    bool controlScanPointinStationArea(const geometry_msgs::msg::Point &point, const std::vector<geometry_msgs::msg::Point> &polygon)
    {
        int n = polygon.size();
        int crossings = 0;

        for (int i = 0; i < n; ++i)
        {
            const geometry_msgs::msg::Point &p1 = polygon[i];
            const geometry_msgs::msg::Point &p2 = polygon[(i + 1) % n];

            if (((p1.y > point.y) != (p2.y > point.y)) &&
                (point.x < (p2.x - p1.x) * (point.y - p1.y) / (p2.y - p1.y) + p1.x))
            {
                crossings++;
            }
        }

        return (crossings % 2 == 1);
    }

    /**
     * @brief Processes incoming LIDAR scan data to detect pallets
     * @param scan_msg Incoming LaserScan message
     */
    void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg)
    {
        publishPalletStationArea(pallet_station_area[0], pallet_station_area[1], pallet_station_area[2], pallet_station_area[3]);
        try
        {
            // Get the transform from lidar_fork_1 to map
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("map", "lidar_fork_1", tf2::TimePointZero);

            // Prepare a new LaserScan message for filtered data
            auto filtered_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*scan_msg);
            filtered_scan->ranges.clear();

            // Iterate through LaserScan ranges
            for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
            {
                double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
                double range = scan_msg->ranges[i];

                // Skip invalid range values
                if (range < scan_msg->range_min || range > scan_msg->range_max)
                {
                    filtered_scan->ranges.push_back(std::numeric_limits<float>::infinity());
                    continue;
                }

                // Calculate point in lidar_fork_1 frame
                geometry_msgs::msg::PointStamped point_in_lidar;
                point_in_lidar.header.frame_id = "lidar_fork_1";
                point_in_lidar.point.x = range * cos(angle);
                point_in_lidar.point.y = range * sin(angle);
                point_in_lidar.point.z = 0.0;

                // Transform point to map frame
                geometry_msgs::msg::PointStamped point_in_map;
                tf2::doTransform(point_in_lidar, point_in_map, transform);

                // Check if the point is within the polygon
                if (controlScanPointinStationArea(point_in_map.point, pallet_station_area))
                {
                    filtered_scan->ranges.push_back(range);
                }
                else
                {
                    filtered_scan->ranges.push_back(std::numeric_limits<float>::infinity());
                }
            }

            if (filtered_scan->ranges.empty())
            {
                return;
            }

            auto cloud = convertLaserScanToPointCloud(filtered_scan);

            if (cloud->points.empty())
            {
                return;
            }

            auto pallet_indicies = performDBSCAN(cloud);
            auto pallet_cloud = createClusteredPointCloud(cloud, pallet_indicies);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        }
    }

    /**
     * @brief Converts LaserScan data to PointCloud data
     * @param scan_msg Incoming LaserScan message
     * @return Point cloud data
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertLaserScanToPointCloud(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
        {
            if (std::isfinite(scan_msg->ranges[i]))
            {
                float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
                pcl::PointXYZ point;
                point.x = scan_msg->ranges[i] * cos(angle);
                point.y = scan_msg->ranges[i] * sin(angle);
                point.z = 0.0;
                cloud->points.push_back(point);
            }
        }
        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = true;

        // Apply Statistical Outlier Removal filter
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        if (!cloud->points.empty())
        {
            sor.setInputCloud(cloud);
            sor.setMeanK(mean_k);             // Number of neighbors to analyze
            sor.setStddevMulThresh(stddev_mul_thresh);  // Standard deviation threshold
            sor.filter(*cloud_filtered);

            return cloud_filtered;
        }
        else
        {
            return cloud;
        }
    }

    /**
     * @brief Performs DBSCAN clustering on point cloud data
     * @param cloud Input point cloud
     * @return Vector of point indices representing potential pallet clusters
     */
    std::vector<pcl::PointIndices> performDBSCAN(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance);
        ec.setMinClusterSize(min_pallet_cluster_size);
        ec.setMaxClusterSize(max_pallet_cluster_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        ec.extract(cluster_indices);

        int cluster_id = 0;
        for (const auto &indices : cluster_indices)
        {
            findPallet(cloud, indices, cluster_id++);
        }

        return cluster_indices;
    }

    /**
     * @brief Creates a point cloud from the cluster indices
     * @param cloud Input point cloud
     * @param cluster_indices Vector of point indices representing potential pallet clusters
     * @return Clustered point cloud
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr createClusteredPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::vector<pcl::PointIndices> &cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &indices : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud2(new pcl::PointCloud<pcl::PointXYZ>);

            for (const auto &idx : indices.indices)
            {
                pcl::PointXYZ point;
                point.x = cloud->points[idx].x;
                point.y = cloud->points[idx].y;
                point.z = cloud->points[idx].z;
                clustered_cloud->points.push_back(point);
                clustered_cloud2->points.push_back(point);
            }
        }

        clustered_cloud->width = clustered_cloud->points.size();
        clustered_cloud->height = 1;
        clustered_cloud->is_dense = true;

        return clustered_cloud;
    }

    /**
     * @brief Extracts the points of a single cluster
     * @param cloud Input point cloud
     * @param cluster_indices Point indices of the cluster
     * @return Point cloud of the cluster
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractClusterPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointIndices &cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_points(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &idx : cluster_indices.indices)
        {
            cluster_points->points.push_back(cloud->points[idx]);
        }
        return cluster_points;
    }

    /**
     * @brief Computes the convex hull of the cluster points
     * @param cluster_points Input cluster point cloud
     * @return Convex hull cloud
     */
    pcl::PointCloud<pcl::PointXYZ> computeConvexHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cluster_points)
    {
        pcl::ConvexHull<pcl::PointXYZ> chull;
        pcl::PointCloud<pcl::PointXYZ> hull_cloud;
        chull.setInputCloud(cluster_points);
        chull.reconstruct(hull_cloud);
        return hull_cloud;
    }

    /**
     * @brief Computes the minimum area rectangle of the cluster points
     * @param hull_cloud Convex hull cloud
     * @param rect_points Array of four rectangle corner points (output)
     */
    void computeMinAreaRectangle(const pcl::PointCloud<pcl::PointXYZ> &hull_cloud, cv::Point2f rect_points[4])
    {
        std::vector<cv::Point2f> points;
        for (const auto &point : hull_cloud.points)
        {
            points.emplace_back(point.x, point.y);
        }

        cv::RotatedRect minRect = cv::minAreaRect(points);
        minRect.points(rect_points);
    }

    /**
     * @brief Calculates yaw angle between two points
     * @param p1 First point
     * @param p2 Second point
     * @return Yaw angle in radians
     */
    double calculateYaw(const cv::Point2f &p1, const cv::Point2f &p2)
    {
        // Calculate the differences in the x and y coordinates
        float dx = p2.x - p1.x;
        float dy = p2.y - p1.y;

        // Use atan2 to compute the angle in radians
        double yaw = std::atan2(dy, dx) + (M_PI / 2);
        if (yaw < (M_PI / 2))
        {
            yaw += M_PI;
        }
        return yaw;  // Return the angle in radians
    }

    /**
     * @brief Converts euler angles to quaternion
     * @param roll Roll angle in radians
     * @param pitch Pitch angle in radians
     * @param yaw Yaw angle in radians
     * @return Quaternion
     */
    Quaternion eulerToQuaternion(double roll, double pitch, double yaw)
    {
        // Calculate half angles
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        Quaternion q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;

        return q;
    }

    /**
     * @brief Computes the center points of the rectangle sides and filters by distance range
     * @param rect_points Array of rectangle corner points
     * @return Vector of valid center poses
     */
    std::vector<geometry_msgs::msg::Pose> computeRectangleCenters(const cv::Point2f rect_points[4])
    {
        std::vector<geometry_msgs::msg::Pose> center_points;

        // Calculate the main center point (midpoint of the long side)
        geometry_msgs::msg::Pose main_center;
        main_center.position.x = (rect_points[3].x + rect_points[0].x) / 2;
        main_center.position.y = (rect_points[3].y + rect_points[0].y) / 2;
        main_center.position.z = 0.0;

        double main_yaw = calculateYaw(rect_points[0], rect_points[3]);
        Quaternion main_quaternion = eulerToQuaternion(0, 0, main_yaw);

        main_center.orientation.x = main_quaternion.x;
        main_center.orientation.y = main_quaternion.y;
        main_center.orientation.z = main_quaternion.z;
        main_center.orientation.w = main_quaternion.w;

        double dx = rect_points[3].x - rect_points[0].x;
        double dy = rect_points[3].y - rect_points[0].y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // Filter by distance range (long side)
        if (distance > min_long_side_length && distance < max_long_side_length)
        {
            center_points.push_back(main_center);
        }

        // Calculate the midpoints of other sides and filter by distance (short sides)
        for (size_t i = 0; i < 3; ++i)
        {
            geometry_msgs::msg::Pose side_center;

            double dx = rect_points[i + 1].x - rect_points[i].x;
            double dy = rect_points[i + 1].y - rect_points[i].y;
            double distance = std::sqrt(dx * dx + dy * dy);

            side_center.position.x = (rect_points[i].x + rect_points[i + 1].x) / 2;
            side_center.position.y = (rect_points[i].y + rect_points[i + 1].y) / 2;
            side_center.position.z = 0.0;

            double side_yaw = calculateYaw(rect_points[i], rect_points[i + 1]);
            Quaternion side_quaternion = eulerToQuaternion(0, 0, side_yaw);

            side_center.orientation.x = side_quaternion.x;
            side_center.orientation.y = side_quaternion.y;
            side_center.orientation.z = side_quaternion.z;
            side_center.orientation.w = side_quaternion.w;

            // Filter by distance range (short sides)
            if (distance > min_short_side_length && distance < max_short_side_length)
            {
                center_points.push_back(side_center);
            }
        }

        return center_points;
    }

    /**
     * @brief Finds the closest point to the origin
     * @param center_points Vector of center points
     * @return Closest pose to the origin
     */
    geometry_msgs::msg::Pose findClosestPoint(const std::vector<geometry_msgs::msg::Pose> &center_points)
    {
        geometry_msgs::msg::Pose closest_pose;
        double min_distance = std::numeric_limits<double>::infinity();

        for (const auto &pose : center_points)
        {
            double distance = std::sqrt(std::pow(pose.position.x, 2) + std::pow(pose.position.y, 2));
            if (distance < min_distance)
            {
                min_distance = distance;
                closest_pose = pose;
            }
        }
        return closest_pose;
    }

    /**
     * @brief Finds the pallet front point and publishes it
     * @param cloud Input point cloud
     * @param pallet_cluster_indices Point indices of the cluster
     * @param id Cluster ID
     */
    void findPallet(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointIndices &pallet_cluster_indices, int id)
    {
        auto cluster_points = extractClusterPoints(cloud, pallet_cluster_indices);
        auto hull_cloud = computeConvexHull(cluster_points);

        cv::Point2f rect_points[4];
        computeMinAreaRectangle(hull_cloud, rect_points);

        auto center_points = computeRectangleCenters(rect_points);
        auto closest_pose = findClosestPoint(center_points);

        if (center_points.size() > 0)
        {
            publishPalletFront(closest_pose, id);
            publishRotatedRectangle(rect_points, id);
            publishPalletPoseArray(closest_pose);
        }
    }

    /**
     * @brief Transforms pose to map frame and publishes as PoseArray
     * @param pose Pallet pose in lidar_fork_1 frame
     */
    void publishPalletPoseArray(geometry_msgs::msg::Pose pose)
    {
        try
        {
            // Transform pose to map frame
            geometry_msgs::msg::PoseStamped input_pose_stamped;
            input_pose_stamped.header.frame_id = "lidar_fork_1";
            input_pose_stamped.header.stamp = rclcpp::Time(0);
            input_pose_stamped.pose = pose;

            geometry_msgs::msg::PoseStamped output_pose_stamped;
            tf_buffer_->transform(input_pose_stamped, output_pose_stamped, "map", tf2::durationFromSec(1.0));

            // Build and publish PoseArray
            geometry_msgs::msg::PoseArray pallet_poses;
            pallet_poses.header.frame_id = "map";
            pallet_poses.header.stamp = this->now();
            pallet_poses.poses.push_back(output_pose_stamped.pose);

            pallet_pose_array_pub_->publish(pallet_poses);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform pose to map frame: %s", ex.what());
        }
    }

    /**
     * @brief Publishes the pallet front point as an arrow marker in the lidar_fork_1 frame
     * @param start_pose Start pose of the arrow
     * @param id Marker ID
     */
    void publishPalletFront(geometry_msgs::msg::Pose start_pose, int id)
    {
        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.header.frame_id = "lidar_fork_1";
        arrow_marker.header.stamp = rclcpp::Time(0);
        arrow_marker.ns = "arrow_marker";
        arrow_marker.id = id * 10;
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;
        arrow_marker.pose.orientation.w = 1.0;

        // Define arrow scale: shaft diameter and head diameter/length
        arrow_marker.scale.x = 0.05;  // Shaft diameter
        arrow_marker.scale.y = 0.1;   // Head diameter
        arrow_marker.scale.z = 0.15;  // Head length

        // Define arrow color
        arrow_marker.color.r = 0.0;
        arrow_marker.color.g = 1.0;
        arrow_marker.color.b = 1.0;
        arrow_marker.color.a = 1.0;

        // Define the start and end points of the arrow
        geometry_msgs::msg::Point start_point;
        start_point.x = start_pose.position.x;
        start_point.y = start_pose.position.y;
        start_point.z = 0.0;

        geometry_msgs::msg::Point end_point;
        tf2::Quaternion quat(start_pose.orientation.x, start_pose.orientation.y,
                             start_pose.orientation.z, start_pose.orientation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        // Yön (yaw) doğrultusunda 0.5 metre ilerlet
        end_point.x = start_point.x + 0.5 * cos(yaw);
        end_point.y = start_point.y + 0.5 * sin(yaw);
        end_point.z = 0.0;

        arrow_marker.points.push_back(start_point);
        arrow_marker.points.push_back(end_point);

        pallet_marker_pub->publish(arrow_marker);
    }

    /**
     * @brief Publishes the minimum area rectangle as a line strip marker in the lidar_fork_1 frame
     * @param rect_points Array of rectangle corner points
     * @param id Marker ID
     */
    void publishRotatedRectangle(const cv::Point2f *rect_points, int id)
    {
        visualization_msgs::msg::Marker rect_marker;
        rect_marker.header.frame_id = "lidar_fork_1";
        rect_marker.header.stamp = rclcpp::Time(0);
        rect_marker.ns = "min_rotated_rectangle";
        rect_marker.id = id;
        rect_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        rect_marker.action = visualization_msgs::msg::Marker::ADD;
        rect_marker.pose.orientation.w = 1.0;
        rect_marker.scale.x = 0.02;
        rect_marker.color.r = 0.0;
        rect_marker.color.g = 1.0;
        rect_marker.color.b = 1.0;
        rect_marker.color.a = 1.0;

        for (int i = 0; i < 4; i++)
        {
            geometry_msgs::msg::Point p;
            p.x = rect_points[i].x;
            p.y = rect_points[i].y;
            p.z = 0.0;
            rect_marker.points.push_back(p);
        }
        rect_marker.points.push_back(rect_marker.points.front());

        pallet_marker_pub->publish(rect_marker);
    }

    /**
     * @brief Publishes the pallet station area as a line strip marker in the map frame
     * @param p1 First point of the area
     * @param p2 Second point of the area
     * @param p3 Third point of the area
     * @param p4 Fourth point of the area
     */
    void publishPalletStationArea(
        const geometry_msgs::msg::Point &p1,
        const geometry_msgs::msg::Point &p2,
        const geometry_msgs::msg::Point &p3,
        const geometry_msgs::msg::Point &p4)
    {
        visualization_msgs::msg::Marker rectangle_marker;
        rectangle_marker.header.frame_id = "map";
        rectangle_marker.header.stamp = this->now();
        rectangle_marker.ns = "rectangle";
        rectangle_marker.id = 0;
        rectangle_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        rectangle_marker.action = visualization_msgs::msg::Marker::ADD;

        rectangle_marker.scale.x = 0.05;  // Çizgi kalınlığı
        rectangle_marker.color.r = 0.0;
        rectangle_marker.color.g = 1.0;
        rectangle_marker.color.b = 0.0;
        rectangle_marker.color.a = 1.0;

        rectangle_marker.points.push_back(p1);
        rectangle_marker.points.push_back(p2);
        rectangle_marker.points.push_back(p3);
        rectangle_marker.points.push_back(p4);
        rectangle_marker.points.push_back(p1);  // Dikdörtgeni kapatmak için ilk noktaya geri dön

        pallet_station_area_marker_pub->publish(rectangle_marker);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PalletDetection>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
