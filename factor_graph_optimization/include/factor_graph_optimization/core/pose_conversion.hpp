#pragma once

#include <geometry_msgs/msg/pose.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

namespace factor_graph_optimization
{

/// Convert a ROS Pose message into a GTSAM Pose3.
/// Orientation is decoded via RPY to avoid gimbal-lock issues with
/// raw quaternion construction.
gtsam::Pose3 msgToGtsam(const geometry_msgs::msg::Pose & pose);

/// Convert a GTSAM Pose3 back into a ROS Pose message.
geometry_msgs::msg::Pose gtsamToMsg(const gtsam::Pose3 & pose);

}  // namespace factor_graph_optimization
