#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class ScanRelay : public rclcpp::Node
{
public:
    ScanRelay()
    : Node("scan_relay")
    {
        sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/slamware_ros_sdk_server_node/scan",
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                msg->header.frame_id = "lidar_top";
                msg->header.stamp    = now();
                pub_->publish(*msg);
            });

        pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
            "/lidar_top/scan", rclcpp::SensorDataQoS());
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr    pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanRelay>());
    rclcpp::shutdown();
    return 0;
}
