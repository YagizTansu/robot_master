#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/path.hpp>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>

// Enum representing the different states of path tracking
enum PathStatus
{
    IDLE    = 0,
    DOCKING = 1,
    ARRIVED = 3,
    FAILED  = 4
};

class PathCreateUtilities
{
public:
    /**
     * @brief Converts a quaternion to Euler angles.
     * @param pose Input pose with quaternion orientation.
     * @return Roll, pitch, yaw angles as a vector.
     */
    static std::vector<double> quaternionToEuler(const geometry_msgs::msg::Pose &pose)
    {
        tf2::Quaternion quaternion;
        quaternion.setX(pose.orientation.x);
        quaternion.setY(pose.orientation.y);
        quaternion.setZ(pose.orientation.z);
        quaternion.setW(pose.orientation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

        return {roll, pitch, yaw};
    }

    /**
     * @brief Creates a nav_msgs::msg::Path from X and Y coordinates.
     */
    static nav_msgs::msg::Path createPath(const std::vector<double> &x_coords,
                                          const std::vector<double> &y_coords)
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

        if (x_coords.size() != y_coords.size())
        {
            RCLCPP_ERROR(rclcpp::get_logger("PathCreateUtilities"),
                         "X and Y coordinates size mismatch. Path generation failed.");
            return path;
        }

        for (size_t i = 0; i < x_coords.size(); ++i)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
            pose.pose.position.x = x_coords[i];
            pose.pose.position.y = y_coords[i];
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }
        return path;
    }

    static std::pair<double, double> calculateBackwardPosition(double x, double y, double yaw, double distance)
    {
        double new_x = x - distance * std::cos(yaw);
        double new_y = y - distance * std::sin(yaw);
        return std::make_pair(new_x, new_y);
    }

    /**
     * @brief Generates a path to the target pallet position.
     * @param current_pose Current robot position.
     * @param target_pose Target pallet position.
     * @param front_pallet_distance Distance offset from pallet front.
     * @param rotate_degree Optional yaw rotation applied to the final pose.
     */
    static nav_msgs::msg::Path calculatePathCurrentPoseToBelowPalletPose(
        const geometry_msgs::msg::Pose &current_pose,
        const geometry_msgs::msg::Pose &target_pose,
        double front_pallet_distance = 0.7,
        double rotate_degree = 0.0)
    {
        constexpr double step = 0.1;

        std::vector<double> x_coords{current_pose.position.x};
        std::vector<double> y_coords{current_pose.position.y};
        std::vector<geometry_msgs::msg::Quaternion> yaw_coords{current_pose.orientation};

        auto euler_angles = PathCreateUtilities::quaternionToEuler(target_pose);
        auto inside_pallet = PathCreateUtilities::calculateBackwardPosition(
            target_pose.position.x, target_pose.position.y, euler_angles[2], front_pallet_distance);

        for (double dist = 0.0; dist >= step; dist -= step)
        {
            auto backward_point = PathCreateUtilities::calculateBackwardPosition(
                inside_pallet.first, inside_pallet.second, euler_angles[2], dist);
            x_coords.push_back(backward_point.first);
            y_coords.push_back(backward_point.second);
            yaw_coords.push_back(current_pose.orientation);
        }

        x_coords.push_back(inside_pallet.first);
        y_coords.push_back(inside_pallet.second);

        // Make a mutable copy to apply optional rotation
        geometry_msgs::msg::Pose mutable_target = target_pose;
        if (rotate_degree != 0.0)
        {
            RCLCPP_INFO(rclcpp::get_logger("PathCreateUtilities"), "Rotate Inplace...");
            double angle_radians = rotate_degree * M_PI / 180.0;

            tf2::Quaternion rotation;
            rotation.setRPY(0, 0, angle_radians);

            tf2::Quaternion current_orientation;
            tf2::fromMsg(mutable_target.orientation, current_orientation);

            tf2::Quaternion new_orientation = rotation * current_orientation;
            new_orientation.normalize();

            mutable_target.orientation = tf2::toMsg(new_orientation);
        }

        yaw_coords.push_back(mutable_target.orientation);

        return PathCreateUtilities::createPathFromCoords(x_coords, y_coords, yaw_coords);
    }

    static nav_msgs::msg::Path calculatePathCurrentPoseToInsidePalletPose(
        const geometry_msgs::msg::Pose &current_pose,
        const geometry_msgs::msg::Pose &target_pose,
        double inside_pallet_distance = -0.5)
    {
        constexpr double step = 0.1;

        std::vector<double> x_coords{current_pose.position.x};
        std::vector<double> y_coords{current_pose.position.y};
        std::vector<geometry_msgs::msg::Quaternion> yaw_coords{current_pose.orientation};

        auto euler_angles = PathCreateUtilities::quaternionToEuler(target_pose);
        auto inside_pallet = PathCreateUtilities::calculateBackwardPosition(
            target_pose.position.x, target_pose.position.y, euler_angles[2], inside_pallet_distance);

        for (double dist = 0.0; dist >= step; dist -= step)
        {
            auto backward_point = PathCreateUtilities::calculateBackwardPosition(
                inside_pallet.first, inside_pallet.second, euler_angles[2], dist);
            x_coords.push_back(backward_point.first);
            y_coords.push_back(backward_point.second);
            yaw_coords.push_back(current_pose.orientation);
        }

        x_coords.push_back(inside_pallet.first);
        y_coords.push_back(inside_pallet.second);
        yaw_coords.push_back(target_pose.orientation);

        return PathCreateUtilities::createPathFromCoords(x_coords, y_coords, yaw_coords);
    }

    static nav_msgs::msg::Path createPathFromCoords(
        const std::vector<double> &x_coords,
        const std::vector<double> &y_coords,
        const std::vector<geometry_msgs::msg::Quaternion> &quat_coords)
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";

        if (x_coords.size() != y_coords.size() || x_coords.size() != quat_coords.size())
        {
            RCLCPP_ERROR(rclcpp::get_logger("PathCreateUtilities"),
                         "Input vectors have inconsistent sizes.");
            return path;
        }

        for (size_t i = 0; i < x_coords.size(); ++i)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
            pose.pose.position.x = x_coords[i];
            pose.pose.position.y = y_coords[i];
            pose.pose.position.z = 0.0;
            pose.pose.orientation = quat_coords[i];
            path.poses.push_back(pose);
        }

        return path;
    }

    // Function to calculate cubic spline coefficients
    static std::vector<Eigen::Vector4d> calculateSplineCoefficients(
        const std::vector<double> &x,
        const std::vector<double> &y)
    {
        int n = x.size();
        std::vector<Eigen::Vector4d> coefficients(n - 1);

        Eigen::MatrixXd A(n - 2, n);
        A.setZero();

        for (int i = 1; i < n - 1; ++i)
        {
            A(i - 1, i - 1) = 2 * (x[i] - x[i - 1]);
            A(i - 1, i)     = 1 * (x[i + 1] - x[i - 1]);
        }

        Eigen::MatrixXd b(n - 2, 1);
        for (int i = 1; i < n - 1; ++i)
        {
            b(i - 1) = 3 * ((y[i + 1] - y[i]) / (x[i + 1] - x[i]) -
                            (y[i] - y[i - 1]) / (x[i] - x[i - 1]));
        }

        Eigen::VectorXd c = A.colPivHouseholderQr().solve(b);

        for (int i = 0; i < n - 1; ++i)
        {
            coefficients[i](0) = y[i];
            coefficients[i](1) = (y[i + 1] - y[i]) / (x[i + 1] - x[i]) -
                                  (x[i + 1] - x[i]) * (c(i) + c(i + 1)) / 3.0;
            coefficients[i](2) = c(i);
            coefficients[i](3) = (c(i + 1) - c(i)) / (3.0 * (x[i + 1] - x[i]));
        }

        return coefficients;
    }

    // Function to evaluate spline at a given point
    static double evaluateSpline(const std::vector<Eigen::Vector4d> &coefficients, double t, double x)
    {
        int segment = 0;
        while (segment < static_cast<int>(coefficients.size()) - 1 &&
               x > coefficients[segment + 1](0))
        {
            segment++;
        }

        double dx = x - coefficients[segment](0);
        return coefficients[segment](0) +
               coefficients[segment](1) * dx +
               coefficients[segment](2) * dx * dx +
               coefficients[segment](3) * dx * dx * dx;
    }
};
