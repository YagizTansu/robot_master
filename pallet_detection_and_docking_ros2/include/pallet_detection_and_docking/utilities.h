#pragma once

#include <iostream>
#include <tuple>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class Utilities
{
public:
    static std::vector<std::string> split(const std::string &str, char delimiter)
    {
        std::vector<std::string> tokens;
        std::stringstream ss(str);
        std::string token;
        while (std::getline(ss, token, delimiter))
        {
            tokens.push_back(token);
        }
        return tokens;
    }

    static double getDistance(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2)
    {
        double dx = pose1.position.x - pose2.position.x;
        double dy = pose1.position.y - pose2.position.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    static geometry_msgs::msg::Pose getNearestPose(const geometry_msgs::msg::PoseArray &poseArray,
                                                    const geometry_msgs::msg::Pose &targetPoint)
    {
        geometry_msgs::msg::Pose nearestPose;
        double minDistance = std::numeric_limits<double>::infinity();

        for (const auto &pose : poseArray.poses)
        {
            double distance = getDistance(pose, targetPoint);
            if (distance < minDistance)
            {
                minDistance = distance;
                nearestPose = pose;
            }
        }
        return nearestPose;
    }

    static double calculateDistance(const geometry_msgs::msg::Pose &current_pose,
                                    const geometry_msgs::msg::Pose &target_pose)
    {
        double dx = target_pose.position.x - current_pose.position.x;
        double dy = target_pose.position.y - current_pose.position.y;
        return sqrt(dx * dx + dy * dy);
    }

    static double calculateYaw(const geometry_msgs::msg::Pose &pose)
    {
        tf2::Quaternion q(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    static double calculateAngleDifference(double current_yaw, double target_yaw)
    {
        double diff = target_yaw - current_yaw;
        while (diff >  M_PI) diff -= 2 * M_PI;
        while (diff < -M_PI) diff += 2 * M_PI;
        return diff;
    }
};
