#include <rclcpp/rclcpp.hpp>
#include <pallet_detection_and_docking/utilities.h>
#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/msg/goal_status.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
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

        // TF2 buffer & listener for map -> base_footprint
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscribers
        path_sub_   = this->create_subscription<nav_msgs::msg::Path>(
            "pallet_docking_path", 10,
            std::bind(&PathFollower::pathCallback, this, std::placeholders::_1));

        pose_sub_   = this->create_subscription<std_msgs::msg::Bool>(  // unused slot kept for ABI
            "path_follower/pose_unused", 1,
            [](const std_msgs::msg::Bool::SharedPtr){});  // TF2 kullanıyoruz

        pause_sub_  = this->create_subscription<std_msgs::msg::Bool>(
            "path_follower/pause", 10,
            std::bind(&PathFollower::pauseCallback, this, std::placeholders::_1));

        cancel_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "path_follower/cancel", 10,
            std::bind(&PathFollower::cancelCallback, this, std::placeholders::_1));

        // Parameters (replaces dynamic_reconfigure DockingConfig)
        this->declare_parameter("linear_speed",       0.10);
        this->declare_parameter("angular_speed",      0.1);
        this->declare_parameter("yaw_tolerance",      0.2);   // 0.5 deg
        this->declare_parameter("distance_tolerance", 0.2);      // 2 cm
        this->declare_parameter("k_stanley",          1.5);
        this->declare_parameter("orient_threshold",   0.2);    // 5 deg
        this->declare_parameter("wheel_base",         1.08);

        linear_speed_       = this->get_parameter("linear_speed").as_double();
        angular_speed_      = this->get_parameter("angular_speed").as_double();
        yaw_tolerance_      = this->get_parameter("yaw_tolerance").as_double();
        distance_tolerance_ = this->get_parameter("distance_tolerance").as_double();
        k_stanley_          = this->get_parameter("k_stanley").as_double();
        orient_threshold_   = this->get_parameter("orient_threshold").as_double();
        wheel_base_         = this->get_parameter("wheel_base").as_double();

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
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pose_sub_;  // dummy — TF2 kullanıyoruz
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_sub_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

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
    double linear_speed_       = 0.10;
    double angular_speed_      = 0.10;
    double yaw_tolerance_      = 0.2;  // 0.5 deg
    double distance_tolerance_ = 0.2;     // 4 cm
    double k_stanley_          = 1.5;      // Stanley cross-track gain
    double orient_threshold_   = 0.2;   // 5 deg — orient-in-place threshold
    double wheel_base_         = 1.08;     // m — Ackermann wheel base

    // Phase tracking
    enum class Phase { ORIENT, STANLEY, FINAL_YAW };
    Phase  current_phase_ = Phase::ORIENT;
    size_t nearest_idx_   = 0;

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
            else if (param.get_name() == "k_stanley")          k_stanley_          = param.as_double();
            else if (param.get_name() == "orient_threshold")   orient_threshold_   = param.as_double();
            else if (param.get_name() == "wheel_base")         wheel_base_         = param.as_double();
        }

        RCLCPP_INFO(this->get_logger(),
            "Reconfigured params: linear_speed=%.2f, angular_speed=%.2f, yaw_tolerance=%.4f, "
            "distance_tolerance=%.3f, k_stanley=%.2f, orient_threshold=%.4f",
            linear_speed_, angular_speed_, yaw_tolerance_,
            distance_tolerance_, k_stanley_, orient_threshold_);

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
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
        nearest_idx_    = 0;
        current_phase_  = Phase::ORIENT;

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
     * @brief Main control loop callback at 150 Hz.
     *        Her tick'te TF2'den anlık map->base_footprint transform'u alır.
     */
    void loopCallback()
    {
        // ── TF2'den pose güncelle ───────────────────────────────────────────
        try
        {
            auto tf = tf_buffer_->lookupTransform(
                "map", "base_footprint", tf2::TimePointZero);
            current_pose_.position.x    = tf.transform.translation.x;
            current_pose_.position.y    = tf.transform.translation.y;
            current_pose_.position.z    = tf.transform.translation.z;
            current_pose_.orientation   = tf.transform.rotation;
            pose_received_ = true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "TF2 map->base_footprint bekleniyor: %s", ex.what());
            pose_received_ = false;
        }

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
        // ── Current robot state ──────────────────────────────────────────────
        const double robot_yaw   = Utilities::calculateYaw(current_pose_);
        const double target_yaw  = Utilities::calculateYaw(target_pose_);
        const double dist_to_goal = Utilities::calculateDistance(current_pose_, target_pose_);

        // ── Status publish helper ────────────────────────────────────────────
        auto publishStatus = [this](uint8_t s)
        {
            action_msgs::msg::GoalStatusArray arr;
            action_msgs::msg::GoalStatus st;
            st.status = s;
            arr.status_list.push_back(st);
            status_pub_->publish(arr);
        };

        // ════════════════════════════════════════════════════════════════════
        // FAZ 3 — FINAL YAW
        // Robot hedefe yeterince yakın: dur, sadece yaw hizala
        // ════════════════════════════════════════════════════════════════════
        if (dist_to_goal < distance_tolerance_)
        {
            current_phase_ = Phase::FINAL_YAW;
            const double yaw_err = Utilities::calculateAngleDifference(robot_yaw, target_yaw);

            if (std::abs(yaw_err) < yaw_tolerance_)
            {
                RCLCPP_INFO(this->get_logger(),
                    "[FINAL_YAW] Goal reached. dist=%.4f m  yaw_err=%.5f rad (%.3f deg)",
                    dist_to_goal, yaw_err, yaw_err * 180.0 / M_PI);
                stop();
                path_received_ = false;
                goal_reached_  = true;
                publishStatus(action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
                return;
            }

            // Yerinde yaw düzelt — orantılı hız, minimum threshold ile
            const double yaw_speed = std::clamp(
                1.5 * yaw_err,
                -angular_speed_,
                angular_speed_);

            geometry_msgs::msg::Twist cmd;
            cmd.linear.x  = 0.0;
            cmd.angular.z = yaw_speed;
            cmd_vel_pub_->publish(cmd);

            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                "[FINAL_YAW] yaw_err=%.5f rad (%.3f deg)  angular=%.4f",
                yaw_err, yaw_err * 180.0 / M_PI, yaw_speed);

            publishStatus(action_msgs::msg::GoalStatus::STATUS_EXECUTING);
            return;
        }

        // ── En yakın path noktasını bul (geri dönmeden ilerle) ───────────────
        {
            double best_dist = std::numeric_limits<double>::max();
            for (size_t i = nearest_idx_; i < path_.poses.size(); ++i)
            {
                double d = Utilities::calculateDistance(current_pose_, path_.poses[i].pose);
                if (d < best_dist)
                {
                    best_dist    = d;
                    nearest_idx_ = i;
                }
                else if (d > best_dist + 0.05)
                {
                    // Mesafe artmaya başladı — daha öteye bakma
                    break;
                }
            }
        }

        const double path_yaw = Utilities::calculateYaw(path_.poses[nearest_idx_].pose);
        const double px       = path_.poses[nearest_idx_].pose.position.x;
        const double py       = path_.poses[nearest_idx_].pose.position.y;
        const double rx       = current_pose_.position.x;
        const double ry       = current_pose_.position.y;

        // Heading error: pozitif → robot sola (CCW) dönmeli
        const double heading_err = Utilities::calculateAngleDifference(robot_yaw, path_yaw);

        // CTE: pozitif → robot path'in sağında
        // Formül: dot(robot_to_path_point, path_right_normal)
        // path_right_normal = (sin(yaw), -cos(yaw))
        const double cte = std::sin(path_yaw) * (rx - px) - std::cos(path_yaw) * (ry - py);

        // ════════════════════════════════════════════════════════════════════
        // FAZ 1 — ORIENT
        // Heading hatası çok büyük: yerinde dön, hizalan
        // ════════════════════════════════════════════════════════════════════
        if (std::abs(heading_err) > orient_threshold_)
        {
            current_phase_ = Phase::ORIENT;

            // Orantılı angular, angular_speed_ ile sınırlandırılmış
            const double angular = std::clamp(
                2.0 * heading_err,
                -angular_speed_,
                angular_speed_);

            geometry_msgs::msg::Twist cmd;
            cmd.linear.x  = 0.0;
            cmd.angular.z = angular;
            cmd_vel_pub_->publish(cmd);

            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                "[ORIENT] heading_err=%.4f rad (%.2f deg)  angular=%.3f",
                heading_err, heading_err * 180.0 / M_PI, angular);

            publishStatus(action_msgs::msg::GoalStatus::STATUS_EXECUTING);
            return;
        }

        // ════════════════════════════════════════════════════════════════════
        // FAZ 2 — STANLEY TRACKING (reverse — çatal yönünde gidiş)
        //
        // Reverse Stanley formülü:
        //   δ = heading_err - arctan(k * cte / v)
        //   (CTE terimi ters — geri gidişte düzeltme yönü tersine döner)
        //
        // cmd_vel.linear.x < 0  → kinco_bridge geri götürür
        // cmd_vel.angular.z     → işaret korunur (kinco_bridge handle eder)
        // ════════════════════════════════════════════════════════════════════
        current_phase_ = Phase::STANLEY;

        const double min_speed   = 0.05;  // sıfıra bölünmeyi önle
        const double stanley_cmd = heading_err
                                   - std::atan2(k_stanley_ * cte,
                                                std::max(linear_speed_, min_speed));

        const double max_angular = M_PI / 2.0;

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = -linear_speed_;  // geri
        cmd.angular.z = std::clamp(stanley_cmd, -max_angular, max_angular);
        cmd_vel_pub_->publish(cmd);

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 100,
            "[STANLEY] idx=%zu/%zu  dist_goal=%.3f m  cte=%.4f m  "
            "heading_err=%.4f rad (%.2f deg)  angular=%.4f",
            nearest_idx_, path_.poses.size() - 1,
            dist_to_goal, cte, heading_err,
            heading_err * 180.0 / M_PI, cmd.angular.z);

        publishStatus(action_msgs::msg::GoalStatus::STATUS_EXECUTING);
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
