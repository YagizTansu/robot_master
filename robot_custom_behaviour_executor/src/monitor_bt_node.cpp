#include "robot_custom_behaviour_executor/monitor_bt_node.hpp"
#include "robot_custom_behaviour_tree_nodes/battery_check_action.hpp"
#include "robot_custom_behaviour_tree_nodes/emergency_stop_action.hpp"
#include "robot_custom_behaviour_tree_nodes/get_charge_station_pose_action.hpp"
#include "robot_custom_behaviour_tree_nodes/navigate_to_charge_station_action.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "nav2_behavior_tree/plugins/action/navigate_to_pose_action.hpp"

namespace robot_custom_behaviour_executor
{

    MonitorBTNode::MonitorBTNode()
        : Node("monitor_bt_node")
    {
        // Get package share directory
        std::string package_share_directory =
            ament_index_cpp::get_package_share_directory("robot_custom_behaviour_executor");

        // BT XML file path
        std::string bt_xml_path = package_share_directory + "/behavior_trees/my_idle_monitor_bt.xml";

        RCLCPP_INFO(this->get_logger(), "Loading BT from: %s", bt_xml_path.c_str());

        // Register custom BT nodes manually
        factory_.registerNodeType<robot_custom_behaviour_tree_nodes::BatteryCheckAction>("BatteryCheck");
        factory_.registerNodeType<robot_custom_behaviour_tree_nodes::EmergencyStopAction>("EmergencyStop");
        factory_.registerNodeType<robot_custom_behaviour_tree_nodes::GetChargeStationPoseAction>("GetChargeStationPose");
        factory_.registerNodeType<robot_custom_behaviour_tree_nodes::NavigateToChargeStationAction>("NavigateToChargeStation");

        // Create tree from file
        try
        {
            tree_ = factory_.createTreeFromFile(bt_xml_path);
            RCLCPP_INFO(this->get_logger(), "BT loaded successfully!");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load BT: %s", e.what());
            throw;
        }

        // Create timer to tick the tree at 10 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10 Hz tick
            std::bind(&MonitorBTNode::tickTree, this));

        RCLCPP_INFO(this->get_logger(), "Monitor BT Node started, ticking at 10 Hz");
    }

    void MonitorBTNode::tickTree()
    {
        BT::NodeStatus status = tree_.tickOnce();

        // Log status changes for debugging
        static BT::NodeStatus last_status = BT::NodeStatus::IDLE;
        if (status != last_status)
        {
            RCLCPP_INFO(this->get_logger(), "BT Status: %s",
                        status == BT::NodeStatus::SUCCESS ? "SUCCESS" : status == BT::NodeStatus::FAILURE ? "FAILURE"
                                                                    : status == BT::NodeStatus::RUNNING   ? "RUNNING"
                                                                                                          : "IDLE");
            last_status = status;
        }
    }

} // namespace robot_custom_behaviour_executor

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robot_custom_behaviour_executor::MonitorBTNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
