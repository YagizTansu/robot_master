#ifndef ROBOT_CUSTOM_LAYERS__PROHIBITED_LAYER_HPP_
#define ROBOT_CUSTOM_LAYERS__PROHIBITED_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "robot_interfaces/srv/add_prohibited_zone.hpp"
#include "robot_interfaces/srv/remove_prohibited_zone.hpp"
#include "robot_interfaces/srv/list_prohibited_zones.hpp"
#include "robot_interfaces/srv/save_prohibited_zone.hpp"
#include "robot_interfaces/srv/delete_prohibited_zone.hpp"
#include "robot_interfaces/srv/load_prohibited_zones.hpp"

namespace robot_custom_layers
{

struct ProhibitedZone
{
  enum Type { POLYGON };
  Type type;
  std::vector<std::pair<double, double>> polygon_points;  // For polygon zones (must be 4 points)
  std::string name;
};

class ProhibitedLayer : public nav2_costmap_2d::Layer
{
public:
  ProhibitedLayer();
  virtual ~ProhibitedLayer();

  virtual void onInitialize() override;
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  virtual void reset() override
  {
    return;
  }

  virtual void onFootprintChanged() override;

  virtual bool isClearable() override
  {
    return false;
  }

private:
  void loadProhibitedZones();
  void loadFromDatabase();
  void saveToDatabase(const ProhibitedZone& zone);
  void deleteFromDatabase(const std::string& zone_name);
  bool isPointInPolygon(double x, double y, const std::vector<std::pair<double, double>>& polygon);
  
  // Service callbacks
  void handleAddZone(
    const std::shared_ptr<robot_interfaces::srv::AddProhibitedZone::Request> request,
    std::shared_ptr<robot_interfaces::srv::AddProhibitedZone::Response> response);
  void handleRemoveZone(
    const std::shared_ptr<robot_interfaces::srv::RemoveProhibitedZone::Request> request,
    std::shared_ptr<robot_interfaces::srv::RemoveProhibitedZone::Response> response);
  void handleListZones(
    const std::shared_ptr<robot_interfaces::srv::ListProhibitedZones::Request> request,
    std::shared_ptr<robot_interfaces::srv::ListProhibitedZones::Response> response);
  
  std::vector<ProhibitedZone> prohibited_zones_;
  std::string prohibited_zones_file_;
  std::string robot_name_;
  unsigned char cost_value_;
  bool enabled_;
  bool use_database_;
  
  // ROS2 Services
  rclcpp::Service<robot_interfaces::srv::AddProhibitedZone>::SharedPtr add_zone_service_;
  rclcpp::Service<robot_interfaces::srv::RemoveProhibitedZone>::SharedPtr remove_zone_service_;
  rclcpp::Service<robot_interfaces::srv::ListProhibitedZones>::SharedPtr list_zones_service_;
  
  // Database service clients
  rclcpp::Client<robot_interfaces::srv::SaveProhibitedZone>::SharedPtr save_zone_client_;
  rclcpp::Client<robot_interfaces::srv::DeleteProhibitedZone>::SharedPtr delete_zone_client_;
  rclcpp::Client<robot_interfaces::srv::LoadProhibitedZones>::SharedPtr load_zones_client_;
  
  rclcpp::Logger logger_{rclcpp::get_logger("ProhibitedLayer")};
};

}  // namespace robot_custom_layers

#endif  // ROBOT_CUSTOM_LAYERS__PROHIBITED_LAYER_HPP_
