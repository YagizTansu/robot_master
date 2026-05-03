#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <robot_interfaces/msg/service.hpp>
#include <robot_interfaces/msg/pallet_station.hpp>
#include <robot_interfaces/msg/docking_status.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include "pallet_detection_and_docking/utilities.h"
#include "pallet_detection_and_docking/PathCreateUtilities.h"
#include <vector>
#include <thread>
#include <chrono>
#include <stdexcept>
#include <mutex>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <angles/angles.h>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>

/**
 * @brief Class for pallet docking operations and path tracking status management.
 *
 * NOTE: This node relies on long blocking sequences (move, wait, repeat) initiated from
 * subscriber callbacks. Run with MultiThreadedExecutor + ReentrantCallbackGroup so that
 * spin_some() calls inside loops can still process incoming messages.
 */
class PalletDocking : public rclcpp::Node
{
public:
    explicit PalletDocking()
    : Node("pallet_docking")
    {
        // All callbacks must be reentrant to allow spin_some() inside blocking loops
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        initializeSubscribersAndPublishers();
        ResetDockingPathProcess();

        RCLCPP_INFO(this->get_logger(), "Pallet Docking Node initialized. Waiting for operations...");
    }

private:
    rclcpp::CallbackGroup::SharedPtr cb_group_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<robot_interfaces::msg::Service>::SharedPtr linear_motor_service_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr docking_process_pause_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr docking_process_cancel_publisher_;
    rclcpp::Publisher<robot_interfaces::msg::DockingStatus>::SharedPtr docking_process_status_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_arrow_;

    // Subscribers
    rclcpp::Subscription<robot_interfaces::msg::Service>::SharedPtr advobot_services_subscriber_;
    rclcpp::Subscription<robot_interfaces::msg::PalletStation>::SharedPtr pallet_station_subscriber_;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr path_status_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pallet_poses_sub_;

    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::PoseArray::SharedPtr latest_pallet_poses_;

    // Path tracking status
    PathStatus path_status_ = PathStatus::IDLE;

    // Timeout for pallet finding operation
    double timeout_ = 10.0;
    double start_time_ = 0.0;
    double total_pick_exit_time_ = 0.0;

    // Flags for pausing and canceling
    bool pause_ = false;
    bool cancel_ = false;

    // Mutex for pose
    std::mutex pose_mutex_;
    std::mutex pallet_poses_mutex_;

    /**
     * @brief Initializes ROS2 subscribers and publishers.
     */
    void initializeSubscribersAndPublishers()
    {
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = cb_group_;

        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/pallet_docking_path", 1);
        linear_motor_service_publisher_ = this->create_publisher<robot_interfaces::msg::Service>("services", 1);
        docking_process_status_publisher_ = this->create_publisher<robot_interfaces::msg::DockingStatus>("/docking_process_status", 1);
        docking_process_pause_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/path_follower/pause", 1);
        docking_process_cancel_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/path_follower/cancel", 1);
        pub_arrow_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

        // Subscribers
        advobot_services_subscriber_ = this->create_subscription<robot_interfaces::msg::Service>(
            "/services", 1,
            std::bind(&PalletDocking::handleAdvobotServiceMessage, this, std::placeholders::_1), sub_opts);

        pallet_station_subscriber_ = this->create_subscription<robot_interfaces::msg::PalletStation>(
            "pallet_station_pose", 1,
            std::bind(&PalletDocking::handlePalletStationPoseMessage, this, std::placeholders::_1), sub_opts);

        path_status_subscriber_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
            "/path_follower/status", 10,
            std::bind(&PalletDocking::pathStatusCallback, this, std::placeholders::_1), sub_opts);

        pause_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "pallet_docking/pause", 10,
            std::bind(&PalletDocking::pauseCallback, this, std::placeholders::_1), sub_opts);

        cancel_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "pallet_docking/cancel", 10,
            std::bind(&PalletDocking::cancelCallback, this, std::placeholders::_1), sub_opts);

        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/tf_pose", 10,
            std::bind(&PalletDocking::poseCallback, this, std::placeholders::_1), sub_opts);

        pallet_poses_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/pallet_poses", 10,
            std::bind(&PalletDocking::palletPosesCallback, this, std::placeholders::_1), sub_opts);
    }

    /**
     * @brief Callback to store the latest pallet poses.
     */
    void palletPosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(pallet_poses_mutex_);
        latest_pallet_poses_ = msg;
    }

    /**
     * @brief Callback for obstacle messages.
     */
    void obstacleCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            PauseDockingProcess();
        }
        else
        {
            UnPauseDockingProcess();
        }
    }

    /**
     * @brief Callback for pause messages.
     */
    void pauseCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            PauseDockingProcess();
        }
        else
        {
            UnPauseDockingProcess();
        }
    }

    /**
     * @brief Callback for cancel messages.
     */
    void cancelCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            cancel_ = true;
            CancelDockingProcess();
        }
    }

    /**
     * @brief Callback function for pose messages.
     */
    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_pose_ = *msg;
    }

    /**
     * @brief Move the robot forward by a specified distance.
     */
    bool robotMoveForward(double speed, double distance)
    {
        pose_mutex_.lock();
        geometry_msgs::msg::Pose initial_pose = current_pose_;
        pose_mutex_.unlock();

        double distance_traveled = 0.0;
        bool moving = true;

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = speed;

        rclcpp::Rate cmd_loop_rate(10);

        while (rclcpp::ok() && moving && !cancel_)
        {
            rclcpp::spin_some(this->shared_from_this());

            pose_mutex_.lock();
            distance_traveled = Utilities::calculateDistance(initial_pose, current_pose_);
            pose_mutex_.unlock();

            if (!pause_)
            {
                if (distance_traveled >= distance)
                {
                    cmd.linear.x = 0.0;
                    moving = false;
                }
            }
            else
            {
                cmd.linear.x = 0.0;
            }

            cmd_vel_pub_->publish(cmd);
            cmd_loop_rate.sleep();
        }

        // Stop the robot
        cmd.linear.x = 0.0;
        cmd_vel_pub_->publish(cmd);

        return !moving;
    }

    /**
     * @brief Move the robot backward by a specified distance.
     */
    bool robotMoveBackward(double speed, double distance)
    {
        pose_mutex_.lock();
        geometry_msgs::msg::Pose initial_pose = current_pose_;
        pose_mutex_.unlock();

        double distance_traveled = 0.0;
        bool moving = true;

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = -speed;

        rclcpp::Rate cmd_loop_rate(10);

        while (rclcpp::ok() && moving && !cancel_)
        {
            rclcpp::spin_some(this->shared_from_this());

            pose_mutex_.lock();
            distance_traveled = Utilities::calculateDistance(initial_pose, current_pose_);
            pose_mutex_.unlock();

            if (!pause_)
            {
                if (distance_traveled >= distance)
                {
                    cmd.linear.x = 0.0;
                    moving = false;
                }
            }
            else
            {
                cmd.linear.x = 0.0;
            }

            cmd_vel_pub_->publish(cmd);
            cmd_loop_rate.sleep();
        }

        // Stop the robot
        cmd.linear.x = 0.0;
        cmd_vel_pub_->publish(cmd);

        return !moving;
    }

    /**
     * @brief Publishes the docking path.
     */
    void PublishDockingPathProcess(nav_msgs::msg::Path &path)
    {
        path_publisher_->publish(path);
    }

    /**
     * @brief Resets the path tracking status to IDLE.
     */
    void ResetDockingPathProcess()
    {
        path_status_ = PathStatus::IDLE;
    }

    /**
     * @brief Publishes a pause command.
     */
    void PauseDockingProcess()
    {
        pause_ = true;

        std_msgs::msg::Bool msg;
        msg.data = true;
        docking_process_pause_publisher_->publish(msg);
        RCLCPP_WARN(this->get_logger(), "Pause docking pallet process!");
    }

    /**
     * @brief Publishes an unpause command.
     */
    void UnPauseDockingProcess()
    {
        pause_ = false;

        std_msgs::msg::Bool msg;
        msg.data = false;
        docking_process_pause_publisher_->publish(msg);
        RCLCPP_WARN(this->get_logger(), "Unpause docking pallet process!");
    }

    /**
     * @brief Publishes a cancel command.
     */
    void CancelDockingProcess()
    {
        std_msgs::msg::Bool msg;
        msg.data = true;
        docking_process_cancel_publisher_->publish(msg);

        robot_interfaces::msg::DockingStatus status;
        status.phase_type = "pick";
        status.phase_status = 2;
        status.success = false;
        docking_process_status_publisher_->publish(status);

        RCLCPP_WARN(this->get_logger(), "Cancel docking pallet process!");
    }

    /**
     * @brief Waits for the path status to update to ARRIVED.
     */
    void WaitRobotFinishedToDockingProcess()
    {
        rclcpp::Rate loop_rate(100);
        while (path_status_ != PathStatus::ARRIVED && rclcpp::ok())
        {
            rclcpp::spin_some(this->shared_from_this());
            loop_rate.sleep();
        }
        ResetDockingPathProcess();
    }

    /**
     * @brief Publishes a command to the linear motor and waits.
     */
    void LinearMotorServicePublish(bool status, int seconds = 15)
    {
        robot_interfaces::msg::Service service;
        service.service_name = "service_linear_direction";
        service.request = status;
        linear_motor_service_publisher_->publish(service);

        if (status)
        {
            RCLCPP_INFO(this->get_logger(), "Linear motor activated.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Linear motor deactivated.");
        }

        rclcpp::sleep_for(std::chrono::seconds(seconds));
    }

    /**
     * @brief Callback for path status updates.
     */
    void pathStatusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
    {
        for (const auto &status : msg->status_list)
        {
            if (status.status == PathStatus::ARRIVED)
            {
                path_status_ = PathStatus::ARRIVED;
            }
        }
    }

    /**
     * @brief Handles service requests from /services topic.
     */
    void handleAdvobotServiceMessage(const robot_interfaces::msg::Service::SharedPtr msg)
    {
        std::vector<std::string> services = Utilities::split(msg->service_name, ',');
        if (!services.empty() && services[0] == "service_pallet_start" && msg->request)
        {
            RCLCPP_INFO(this->get_logger(), "Received 'service_pallet_start' request. Executing pick pallet operation.");
            bool pick_status = executePickPallet();
            if (pick_status)
            {
                executeExitPallet();
            }
        }
    }

    /**
     * @brief Processes pallet station pose messages.
     */
    void handlePalletStationPoseMessage(const robot_interfaces::msg::PalletStation::SharedPtr msg)
    {
        if (msg->station_type == "pick")
        {
            robot_interfaces::msg::DockingStatus status;
            status.phase_type = "pick";
            status.phase_status = 1;

            start_time_ = this->now().seconds();

            bool pick_status = executePickPallet();
            if (pick_status)
            {
                bool exit_status = executeExitPallet();
                if (exit_status)
                {
                    status.phase_type = "pick";
                    status.phase_status = 2;
                    status.success = true;
                    docking_process_status_publisher_->publish(status);
                    total_pick_exit_time_ = this->now().seconds() - start_time_;
                    RCLCPP_INFO(this->get_logger(), "Pick and exit operations completed successfully.");
                    RCLCPP_INFO(this->get_logger(), "Total pick and exit time: %f seconds", total_pick_exit_time_);
                }
                else
                {
                    status.phase_type = "pick";
                    status.phase_status = 2;
                    status.success = false;
                    docking_process_status_publisher_->publish(status);
                    RCLCPP_ERROR(this->get_logger(), "Error during exit operation.");
                }
            }
            else
            {
                status.phase_type = "pick";
                status.phase_status = 2;
                status.success = false;
                docking_process_status_publisher_->publish(status);
                RCLCPP_ERROR(this->get_logger(), "Error during pick operation.");
            }
        }
        else if (msg->station_type == "drop")
        {
            robot_interfaces::msg::DockingStatus status;
            status.phase_type = "drop";
            status.phase_status = 1;
            docking_process_status_publisher_->publish(status);

            RCLCPP_INFO(this->get_logger(), "Station type is 'drop'. Initiating drop operation.");
            geometry_msgs::msg::Pose dropping_station_pose = msg->station_coor.pose;
            bool drop_status = executeDropPallet(dropping_station_pose);
            if (drop_status)
            {
                bool exit_status = executeExitPallet();
                if (exit_status)
                {
                    status.phase_type = "drop";
                    status.phase_status = 2;
                    status.success = true;
                    docking_process_status_publisher_->publish(status);
                    RCLCPP_INFO(this->get_logger(), "Drop and exit operations completed successfully.");
                }
                else
                {
                    status.phase_type = "drop";
                    status.phase_status = 2;
                    status.success = false;
                    docking_process_status_publisher_->publish(status);
                    RCLCPP_ERROR(this->get_logger(), "Error during exit operation.");
                }
            }
            else
            {
                status.phase_type = "drop";
                status.phase_status = 2;
                status.success = false;
                docking_process_status_publisher_->publish(status);
                RCLCPP_ERROR(this->get_logger(), "Error during drop operation.");
            }
        }
    }

    /**
     * @brief Returns the current robot pose. Throws if no pose received yet.
     */
    geometry_msgs::msg::Pose getRobotPose()
    {
        // Wait up to 1 second for a valid pose
        auto start = this->now();
        rclcpp::Rate rate(10);
        while (rclcpp::ok() && (this->now() - start).seconds() < 1.0)
        {
            {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                if (current_pose_.orientation.w != 0.0)
                {
                    return current_pose_;
                }
            }
            rclcpp::spin_some(this->shared_from_this());
            rate.sleep();
        }
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (current_pose_.orientation.w == 0.0)
        {
            throw std::runtime_error("Robot pose data unavailable.");
        }
        return current_pose_;
    }

    /**
     * @brief Waits for pallet poses and returns them. Throws on timeout.
     */
    geometry_msgs::msg::PoseArray getPalletPoses()
    {
        auto start = this->now();
        rclcpp::Rate rate(10);

        while (rclcpp::ok() && (this->now() - start).seconds() < timeout_)
        {
            {
                std::lock_guard<std::mutex> lock(pallet_poses_mutex_);
                if (latest_pallet_poses_ && !latest_pallet_poses_->poses.empty())
                {
                    return *latest_pallet_poses_;
                }
            }
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Pallet poses data unavailable. Searching for pallets... timeout remaining: %.1f s",
                timeout_ - (this->now() - start).seconds());
            rclcpp::spin_some(this->shared_from_this());
            rate.sleep();
        }

        throw std::runtime_error("Pallet poses data unavailable.");
    }

    /**
     * @brief Sets a double parameter on a remote node via SetParameters service.
     */
    void setDynamicParamPathFollower(const std::string &param_name, double value)
    {
        const std::string service_name = "/path_follower/set_parameters";
        auto client = this->create_client<rcl_interfaces::srv::SetParameters>(service_name);

        if (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(this->get_logger(), "Service %s not available", service_name.c_str());
            return;
        }

        auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        rcl_interfaces::msg::Parameter param;
        param.name = param_name;
        param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        param.value.double_value = value;
        request->parameters.push_back(param);

        auto future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Successfully set %s to %f", param_name.c_str(), value);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set parameter %s", param_name.c_str());
        }
    }

    /**
     * @brief Sets an integer parameter on the lidar fork node via SetParameters service.
     */
    void setDynamicParamLidarFork(const std::string &param_name, double value)
    {
        const std::string service_name = "/rplidarNode_fork/set_parameters";
        auto client = this->create_client<rcl_interfaces::srv::SetParameters>(service_name);

        if (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(this->get_logger(), "Service %s not available", service_name.c_str());
            return;
        }

        auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        rcl_interfaces::msg::Parameter param;
        param.name = param_name;
        param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        param.value.integer_value = static_cast<int64_t>(value);
        request->parameters.push_back(param);

        auto future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Successfully set %s to %f", param_name.c_str(), value);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set parameter %s", param_name.c_str());
        }
    }

    void setForkLidarParams(double min_deg_s2, double max_deg_s2)
    {
        setDynamicParamLidarFork("min_deg_s2", min_deg_s2);
        setDynamicParamLidarFork("max_deg_s2", max_deg_s2);
    }

    double getYawFromPose(const geometry_msgs::msg::Pose &pose)
    {
        tf2::Quaternion q(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        return yaw;
    }

    double calculateAngleDifference(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2)
    {
        double yaw1 = getYawFromPose(pose1);
        double yaw2 = getYawFromPose(pose2);

        double diff = yaw1 - yaw2;
        while (diff >  M_PI) diff -= 2 * M_PI;
        while (diff < -M_PI) diff += 2 * M_PI;
        return std::abs(diff);
    }

    double getAverageCurrentYaw()
    {
        double sum = 0.0;
        for (int i = 0; i < 10; i++)
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            sum += getYawFromPose(current_pose_);
        }
        return sum / 10.0;
    }

    /**
     * @brief Rotates the robot in place to reach a target yaw angle.
     */
    bool rotateInPlace(double target_yaw, double angle_tolerance = 0.01)
    {
        geometry_msgs::msg::Twist cmd;
        rclcpp::Rate cmd_loop_rate(20);
        bool rotating = true;

        RCLCPP_INFO(this->get_logger(), "Rotating Robot orientation to pallet orientation Begin");

        rclcpp::sleep_for(std::chrono::seconds(1));

        while (rclcpp::ok() && rotating && !cancel_)
        {
            rclcpp::spin_some(this->shared_from_this());

            double current_yaw = getAverageCurrentYaw();
            RCLCPP_INFO(this->get_logger(), "Rotating Robot orientation to pallet orientation Start");

            double angle_diff = target_yaw - current_yaw;
            while (angle_diff >  M_PI) angle_diff -= 2 * M_PI;
            while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

            RCLCPP_INFO(this->get_logger(), "Target yaw: %.2f, Current yaw: %.2f, Angle diff: %.2f",
                        target_yaw, current_yaw, angle_diff);

            if (std::abs(angle_diff) <= angle_tolerance)
            {
                cmd.angular.z = 0.0;
                cmd_vel_pub_->publish(cmd);
                rotating = false;
                RCLCPP_INFO(this->get_logger(), "Target rotation reached!");
            }
            else if (!pause_)
            {
                double Kp = 0.5;
                cmd.angular.z = Kp * angle_diff;

                double max_speed = 0.2;
                if (cmd.angular.z >  max_speed) cmd.angular.z =  max_speed;
                if (cmd.angular.z < -max_speed) cmd.angular.z = -max_speed;
            }
            else
            {
                cmd.angular.z = 0.0;
            }

            cmd_vel_pub_->publish(cmd);
            cmd_loop_rate.sleep();
        }

        cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);

        rclcpp::sleep_for(std::chrono::seconds(2));

        return !rotating;
    }

    double averageYaw(const std::vector<double> &yaws)
    {
        double x = 0.0;
        double y = 0.0;
        for (double yaw : yaws)
        {
            x += cos(yaw);
            y += sin(yaw);
        }
        return atan2(y, x);
    }

    double getAverageTargetYaw()
    {
        std::vector<double> yaws;
        for (int i = 0; i < 10; ++i)
        {
            geometry_msgs::msg::PoseArray pallet_poses = getPalletPoses();
            double yaw = getYawFromPose(pallet_poses.poses[0]);
            double target_yaw = angles::normalize_angle(yaw + M_PI);
            yaws.push_back(target_yaw);
        }
        return averageYaw(yaws);
    }

    void CheckDirection()
    {
        double target_yaw = getAverageTargetYaw();
        if (!rotateInPlace(target_yaw, 0.01))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to align with pallet orientation, but continuing with approach");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Successfully aligned with pallet orientation");
        }
    }

    void CheckDistance()
    {
        bool position_correct = false;
        int max_correction_attempts = 3;
        int attempt = 0;
        double target_distance = 0.75;
        double distance_tolerance = 0.06;

        while (!position_correct && attempt < max_correction_attempts && rclcpp::ok())
        {
            geometry_msgs::msg::Pose robot_pose = getAveragePose();
            geometry_msgs::msg::PoseArray pallet_poses = getPalletPoses();
            geometry_msgs::msg::Pose nearest_pallet_pose = pallet_poses.poses[0];

            double current_distance = Utilities::calculateDistance(nearest_pallet_pose, robot_pose);
            RCLCPP_INFO(this->get_logger(), "Current distance to pallet: %.3f m (target: %.3f m)",
                        current_distance, target_distance);

            if (std::abs(current_distance - target_distance) <= distance_tolerance)
            {
                position_correct = true;
                RCLCPP_INFO(this->get_logger(), "Robot is at correct distance from pallet");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Robot is not at correct distance. Moving and recalculating...");

                robotMoveForward(0.15, 0.05);

                robot_pose = getRobotPose();
                pallet_poses = getPalletPoses();
                nearest_pallet_pose = pallet_poses.poses[0];

                nav_msgs::msg::Path path = PathCreateUtilities::calculatePathCurrentPoseToBelowPalletPose(
                    robot_pose, nearest_pallet_pose, target_distance);
                this->PublishDockingPathProcess(path);
                this->WaitRobotFinishedToDockingProcess();
            }

            attempt++;
            if (!position_correct && attempt >= max_correction_attempts)
            {
                RCLCPP_WARN(this->get_logger(),
                    "Could not achieve exact position after %d attempts. Proceeding with best position.",
                    max_correction_attempts);
            }
        }
    }

    /**
     * @brief Returns the average of 10 consecutive pose readings.
     */
    geometry_msgs::msg::Pose getAveragePose()
    {
        int n = 10;
        double sum_x = 0, sum_y = 0, sum_z = 0;
        double sum_qx = 0, sum_qy = 0, sum_qz = 0, sum_qw = 0;

        for (int i = 0; i < n; ++i)
        {
            auto start = this->now();
            bool got_pose = false;
            rclcpp::Rate rate(10);
            while (rclcpp::ok() && (this->now() - start).seconds() < 1.0)
            {
                rclcpp::spin_some(this->shared_from_this());
                std::lock_guard<std::mutex> lock(pose_mutex_);
                if (current_pose_.orientation.w != 0.0)
                {
                    sum_x  += current_pose_.position.x;
                    sum_y  += current_pose_.position.y;
                    sum_z  += current_pose_.position.z;
                    sum_qx += current_pose_.orientation.x;
                    sum_qy += current_pose_.orientation.y;
                    sum_qz += current_pose_.orientation.z;
                    sum_qw += current_pose_.orientation.w;
                    got_pose = true;
                    break;
                }
                rate.sleep();
            }
            if (!got_pose)
            {
                throw std::runtime_error("Robot pose data unavailable.");
            }
        }

        geometry_msgs::msg::Pose avg_pose;
        avg_pose.position.x = sum_x  / n;
        avg_pose.position.y = sum_y  / n;
        avg_pose.position.z = sum_z  / n;
        avg_pose.orientation.x = sum_qx / n;
        avg_pose.orientation.y = sum_qy / n;
        avg_pose.orientation.z = sum_qz / n;
        avg_pose.orientation.w = sum_qw / n;
        return avg_pose;
    }

    /**
     * @brief Executes pallet picking operation.
     */
    bool executePickPallet()
    {
        RCLCPP_INFO(this->get_logger(), "Starting pallet picking operation...");
        try
        {
            setDynamicParamPathFollower("yaw_tolerance", 0.04);
            setDynamicParamPathFollower("distance_tolerance", 0.04);

            geometry_msgs::msg::Pose robot_pose = getAveragePose();
            geometry_msgs::msg::PoseArray pallet_poses = getPalletPoses();
            geometry_msgs::msg::Pose nearest_pallet_pose = pallet_poses.poses[0];

            nav_msgs::msg::Path path = PathCreateUtilities::calculatePathCurrentPoseToBelowPalletPose(
                robot_pose, nearest_pallet_pose, 0.90);
            this->PublishDockingPathProcess(path);
            this->WaitRobotFinishedToDockingProcess();

            RCLCPP_INFO(this->get_logger(), "Path following done, waiting 10 seconds...");
            rclcpp::sleep_for(std::chrono::seconds(10));

            bool exit_pallet_operations = robotMoveBackward(0.16, 0.95);
            if (!exit_pallet_operations)
            {
                RCLCPP_ERROR(this->get_logger(), "Error during pallet picking operation.");
                return false;
            }

            this->LinearMotorServicePublish(true, 5);

            RCLCPP_INFO(this->get_logger(), "Finished pallet picking operation...");
            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error during pallet picking operation: %s", e.what());
            return false;
        }
    }

    /**
     * @brief Executes pallet exit operation.
     */
    bool executeExitPallet()
    {
        RCLCPP_INFO(this->get_logger(), "Starting pallet exit operation...");
        bool result = robotMoveForward(0.18, 1.25);

        if (result)
        {
            RCLCPP_INFO(this->get_logger(), "Finished pallet exit operation.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Error during pallet exit operation.");
        }
        return result;
    }

    /**
     * @brief Executes pallet dropping operation.
     */
    bool executeDropPallet(geometry_msgs::msg::Pose dropping_station_pose)
    {
        RCLCPP_INFO(this->get_logger(), "Starting pallet dropping operation...");
        try
        {
            geometry_msgs::msg::Pose robot_pose = getRobotPose();
            double station_yaw = getYawFromPose(dropping_station_pose);

            nav_msgs::msg::Path path = PathCreateUtilities::calculatePathCurrentPoseToBelowPalletPose(
                robot_pose, dropping_station_pose, -0.7);
            this->PublishDockingPathProcess(path);
            this->WaitRobotFinishedToDockingProcess();

            if (!rotateInPlace(station_yaw, 0.01))
            {
                RCLCPP_WARN(this->get_logger(), "Failed to align with station orientation, but continuing");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Successfully aligned with station orientation");
            }

            bool drop_result = robotMoveBackward(0.20, 1.2);
            if (!drop_result)
            {
                RCLCPP_ERROR(this->get_logger(), "Error during Drop operation.");
                return false;
            }

            this->LinearMotorServicePublish(false, 5);

            RCLCPP_INFO(this->get_logger(), "Finished pallet dropping operation...");
            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error during pallet dropping operation: %s", e.what());
            return false;
        }
    }

    void publishArrow(double x, double y, double yaw_rad,
                      float r = 1.0, float g = 0.0, float b = 0.0, float a = 1.0, int /*id*/ = 0)
    {
        static int marker_id = 0;

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_rad);

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "arrow";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.orientation = tf2::toMsg(q);

        marker.scale.x = 0.5;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = a;

        pub_arrow_->publish(marker);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // MultiThreadedExecutor required: blocking loops inside callbacks use spin_some()
    auto node = std::make_shared<PalletDocking>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
