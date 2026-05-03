#include <rclcpp/rclcpp.hpp>
#include <pallet_detection_and_docking/utilities.h>
#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/msg/goal_status.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <cmath>
#include <algorithm>
#include <limits>

/**
 * @brief Class for following a path using Stanley controller.
 */
class PathFollower : public rclcpp::Node
{
public:
    PathFollower() : Node("path_follower")
    {
        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        status_pub_  = this->create_publisher<action_msgs::msg::GoalStatusArray>("path_follower/status", 10);

        // Subscribers
        path_sub_   = this->create_subscription<nav_msgs::msg::Path>(
            "pallet_docking_path", 10,
            std::bind(&PathFollower::pathCallback, this, std::placeholders::_1));

        pose_sub_   = this->create_subscription<geometry_msgs::msg::Pose>(
            "/tf_pose", 10,
            std::bind(&PathFollower::poseCallback, this, std::placeholders::_1));

        pause_sub_  = this->create_subscription<std_msgs::msg::Bool>(
            "path_follower/pause", 10,
            std::bind(&PathFollower::pauseCallback, this, std::placeholders::_1));

        cancel_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "path_follower/cancel", 10,
            std::bind(&PathFollower::cancelCallback, this, std::placeholders::_1));

        // Parameters (replaces dynamic_reconfigure DockingConfig)
        this->declare_parameter("linear_speed",       0.2);
        this->declare_parameter("angular_speed",      0.5);
        this->declare_parameter("yaw_tolerance",      0.05);
        this->declare_parameter("distance_tolerance", 0.05);

        linear_speed_       = this->get_parameter("linear_speed").as_double();
        angular_speed_      = this->get_parameter("angular_speed").as_double();
        yaw_tolerance_      = this->get_parameter("yaw_tolerance").as_double();
        distance_tolerance_ = this->get_parameter("distance_tolerance").as_double();

        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&PathFollower::parametersCallback, this, std::placeholders::_1));

        // Main control loop at 150 Hz (replaces spin() while loop)
        loop_timer_ = this->create_wall_timer(
            std::chrono::microseconds(6667),  // ~150 Hz
            std::bind(&PathFollower::loopCallback, this));
    }

private:
    // Publishers / Subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<action_msgs::msg::GoalStatusArray>::SharedPtr status_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_sub_;

    rclcpp::TimerBase::SharedPtr loop_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Pose and path
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Pose target_pose_;
    nav_msgs::msg::Path      path_;

    bool path_received_ = false;
    bool pose_received_ = false;
    bool goal_reached_  = false;   // SUCCEEDED sonrası aynı path gelirse yoksay
    bool final_yaw_mode_ = false;  // Hedefe ulaşıldı, artık sadece yaw düzelt

    // Control flags
    bool pause_  = false;
    bool cancel_ = false;

    // Control parameters
    double linear_speed_       = 0.2;
    double angular_speed_      = 0.5;
    double yaw_tolerance_      = 0.05;
    double distance_tolerance_ = 0.05;

    /**
     * @brief Parameter update callback (replaces dynamic_reconfigure).
     */
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        for (const auto &param : parameters)
        {
            if      (param.get_name() == "linear_speed")       linear_speed_       = param.as_double();
            else if (param.get_name() == "angular_speed")      angular_speed_      = param.as_double();
            else if (param.get_name() == "yaw_tolerance")      yaw_tolerance_      = param.as_double();
            else if (param.get_name() == "distance_tolerance") distance_tolerance_ = param.as_double();
        }

        RCLCPP_INFO(this->get_logger(),
            "Reconfigured params: linear_speed=%.2f, angular_speed=%.2f, yaw_tolerance=%.2f, distance_tolerance=%.2f",
            linear_speed_, angular_speed_, yaw_tolerance_, distance_tolerance_);

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    /**
     * @brief Callback function for pose messages.
     */
    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        if (!msg)
        {
            RCLCPP_WARN(this->get_logger(), "Received null pose message!");
            return;
        }
        current_pose_  = *msg;
        pose_received_ = true;
    }

    /**
     * @brief Callback function for path messages.
     *        Eğer önceki hedefle aynı path tekrar geliyorsa ve görev bittiyse yoksayar.
     */
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (!msg || msg->poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received an empty or null path!");
            path_received_ = false;
            stop();
            return;
        }

        // Görev tamamlandıktan sonra path planner aynı path'i tekrar yayınlarsa yoksay.
        if (goal_reached_)
        {
            geometry_msgs::msg::Pose new_target = msg->poses.back().pose;
            double dist = Utilities::calculateDistance(new_target, target_pose_);
            if (dist < 0.01)
            {
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Goal already reached, ignoring duplicate path.");
                return;
            }
            // Farklı bir hedef geldi — yeni görev başlıyor
            goal_reached_ = false;
        }

        path_          = *msg;
        target_pose_   = msg->poses.back().pose;
        path_received_ = true;
        final_yaw_mode_ = false;

        RCLCPP_INFO(this->get_logger(),
            "New path received with %zu poses, target: (%.2f, %.2f, %.2f)",
            path_.poses.size(),
            target_pose_.position.x, target_pose_.position.y,
            Utilities::calculateYaw(target_pose_));
    }

    /**
     * @brief Stop the robot.
     */
    void stop()
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x  = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
    }

    /**
     * @brief Callback for pause messages.
     */
    void pauseCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        pause_ = msg->data;
        if (pause_)
        {
            stop();
        }
    }

    /**
     * @brief Callback for cancel messages.
     */
    void cancelCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            cancel_         = true;
            path_received_  = false;
            goal_reached_   = false;
            final_yaw_mode_ = false;
            stop();
        }
    }

    /**
     * @brief Main control loop callback at 150 Hz (replaces spin() while loop).
     */
    void loopCallback()
    {
        action_msgs::msg::GoalStatusArray status_array;
        action_msgs::msg::GoalStatus status;

        if (cancel_)
        {
            status.status = action_msgs::msg::GoalStatus::STATUS_CANCELED;
            status_array.status_list.push_back(status);
            status_pub_->publish(status_array);
            cancel_ = false;
        }
        else if (pause_)
        {
            return;
        }
        else if (path_received_ && pose_received_)
        {
            followPath();
        }
        else if (path_received_ && !pose_received_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Waiting for pose data...");
            status.status = action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
            status_array.status_list.push_back(status);
            status_pub_->publish(status_array);
        }
        else
        {
            status.status = action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
            status_array.status_list.push_back(status);
            status_pub_->publish(status_array);
        }
    }

    void followPath()
    {

    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
