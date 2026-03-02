#include "robot_custom_layers/prohibited_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "nav2_util/node_utils.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace robot_custom_layers
{

ProhibitedLayer::ProhibitedLayer()
: cost_value_(LETHAL_OBSTACLE),
  enabled_(true)
{
}

ProhibitedLayer::~ProhibitedLayer()
{
}

void ProhibitedLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  declareParameter("cost_value", rclcpp::ParameterValue(254));
  int cost_temp = 254;
  node->get_parameter(name_ + "." + "cost_value", cost_temp);
  cost_value_ = static_cast<unsigned char>(cost_temp);

  declareParameter("prohibited_zones_file", rclcpp::ParameterValue(""));
  node->get_parameter(name_ + "." + "prohibited_zones_file", prohibited_zones_file_);

  declareParameter("use_database", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "use_database", use_database_);

  declareParameter("robot_name", rclcpp::ParameterValue("default_robot"));
  node->get_parameter(name_ + "." + "robot_name", robot_name_);

  current_ = true;
  
  // Create database clients if enabled
  if (use_database_) {
    auto node_ptr = node.get();
    save_zone_client_ = node_ptr->create_client<robot_interfaces::srv::SaveProhibitedZone>(
      "/save_prohibited_zone");
    delete_zone_client_ = node_ptr->create_client<robot_interfaces::srv::DeleteProhibitedZone>(
      "/delete_prohibited_zone");
    load_zones_client_ = node_ptr->create_client<robot_interfaces::srv::LoadProhibitedZones>(
      "/load_prohibited_zones");
    
    RCLCPP_INFO(logger_, "Database integration enabled for robot: %s", robot_name_.c_str());
    
    // Load from database first
    loadFromDatabase();
  }
  
  // Then load from parameters (can override database)
  loadProhibitedZones();

  // Create services for dynamic zone management
  auto node_ptr = node.get();
  add_zone_service_ = node_ptr->create_service<robot_interfaces::srv::AddProhibitedZone>(
    "~/add_prohibited_zone",
    std::bind(&ProhibitedLayer::handleAddZone, this, std::placeholders::_1, std::placeholders::_2));
  
  remove_zone_service_ = node_ptr->create_service<robot_interfaces::srv::RemoveProhibitedZone>(
    "~/remove_prohibited_zone",
    std::bind(&ProhibitedLayer::handleRemoveZone, this, std::placeholders::_1, std::placeholders::_2));
  
  list_zones_service_ = node_ptr->create_service<robot_interfaces::srv::ListProhibitedZones>(
    "~/list_prohibited_zones",
    std::bind(&ProhibitedLayer::handleListZones, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
    logger_,
    "ProhibitedLayer initialized with %zu zones, cost_value: %d",
    prohibited_zones_.size(), cost_value_);
}

void ProhibitedLayer::loadProhibitedZones()
{
  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(logger_, "Unable to lock node!");
    return;
  }

  prohibited_zones_.clear();

  // Load from parameters (example: polygon zones)
  try {
    // Declare polygon_zones parameter
    declareParameter("polygon_zones", rclcpp::ParameterValue(std::vector<std::string>()));
    
    // Try to get polygon zones
    std::vector<std::string> zone_names;
    if (node->get_parameter(name_ + ".polygon_zones", zone_names) && !zone_names.empty()) {

      for (const auto& zone_name : zone_names) {
        std::string param_prefix = name_ + "." + zone_name;
        
        // Declare points parameter for this zone
        declareParameter(zone_name + ".points", rclcpp::ParameterValue(std::vector<double>()));
        
        std::vector<double> points;
        if (node->get_parameter(param_prefix + ".points", points)) {

          if (points.size() >= 6 && points.size() % 2 == 0) {  // At least 3 points (x,y pairs)
            ProhibitedZone zone;
            zone.type = ProhibitedZone::POLYGON;
            zone.name = zone_name;

            for (size_t i = 0; i < points.size(); i += 2) {
              zone.polygon_points.push_back({points[i], points[i + 1]});
            }

            prohibited_zones_.push_back(zone);
            RCLCPP_INFO(logger_, "Loaded polygon zone '%s' with %zu points",
                        zone_name.c_str(), zone.polygon_points.size());
          }
        }
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(logger_, "Error loading prohibited zones: %s", e.what());
  }

  if (prohibited_zones_.empty()) {
    RCLCPP_WARN(logger_, "No prohibited zones loaded! Layer will have no effect.");
  }
}

void ProhibitedLayer::loadFromDatabase()
{
  if (!use_database_ || !load_zones_client_) {
    return;
  }

  // Wait for service
  if (!load_zones_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(logger_, "Database service not available, skipping database load");
    return;
  }

  auto request = std::make_shared<robot_interfaces::srv::LoadProhibitedZones::Request>();
  request->robot_name = robot_name_;

  auto future = load_zones_client_->async_send_request(request);
  
  // Wait for response
  if (rclcpp::spin_until_future_complete(node_.lock(), future, std::chrono::seconds(5)) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(logger_, "Loaded zones from database: %s", response->message.c_str());
      // TODO: Parse JSON and populate prohibited_zones_
      // For now, zones will be loaded from parameters
    } else {
      RCLCPP_WARN(logger_, "Failed to load from database: %s", response->message.c_str());
    }
  } else {
    RCLCPP_WARN(logger_, "Database load timeout");
  }
}

void ProhibitedLayer::saveToDatabase(const ProhibitedZone& zone)
{
  if (!use_database_ || !save_zone_client_) {
    return;
  }

  auto request = std::make_shared<robot_interfaces::srv::SaveProhibitedZone::Request>();
  request->robot_name = robot_name_;
  request->zone_name = zone.name;
  request->zone_type = "polygon";

  for (const auto& point : zone.polygon_points) {
    request->polygon_points.push_back(point.first);
    request->polygon_points.push_back(point.second);
  }

  save_zone_client_->async_send_request(request);
  RCLCPP_INFO(logger_, "Saved zone '%s' to database", zone.name.c_str());
}

void ProhibitedLayer::deleteFromDatabase(const std::string& zone_name)
{
  if (!use_database_ || !delete_zone_client_) {
    return;
  }

  auto request = std::make_shared<robot_interfaces::srv::DeleteProhibitedZone::Request>();
  request->robot_name = robot_name_;
  request->zone_name = zone_name;

  delete_zone_client_->async_send_request(request);
  RCLCPP_INFO(logger_, "Deleted zone '%s' from database", zone_name.c_str());
}

void ProhibitedLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  // Expand bounds to cover all prohibited zones
  for (const auto& zone : prohibited_zones_) {
    for (const auto& point : zone.polygon_points) {
      *min_x = std::min(*min_x, point.first);
      *min_y = std::min(*min_y, point.second);
      *max_x = std::max(*max_x, point.first);
      *max_y = std::max(*max_y, point.second);
    }
  }
}

void ProhibitedLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  // Iterate through the costmap cells
  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      // Convert grid coordinates to world coordinates
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);

      // Check if point is in any prohibited zone
      for (const auto& zone : prohibited_zones_) {
        bool in_zone = isPointInPolygon(wx, wy, zone.polygon_points);

        if (in_zone) {
          master_grid.setCost(i, j, cost_value_);
          break;  // No need to check other zones
        }
      }
    }
  }
}

bool ProhibitedLayer::isPointInPolygon(
  double x, double y,
  const std::vector<std::pair<double, double>>& polygon)
{
  // Ray casting algorithm
  int n = polygon.size();
  bool inside = false;

  for (int i = 0, j = n - 1; i < n; j = i++) {
    double xi = polygon[i].first;
    double yi = polygon[i].second;
    double xj = polygon[j].first;
    double yj = polygon[j].second;

    bool intersect = ((yi > y) != (yj > y)) &&
                     (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
    if (intersect) {
      inside = !inside;
    }
  }

  return inside;
}

void ProhibitedLayer::onFootprintChanged()
{
  // Nothing to do here for prohibited zones
}

void ProhibitedLayer::handleAddZone(
  const std::shared_ptr<robot_interfaces::srv::AddProhibitedZone::Request> request,
  std::shared_ptr<robot_interfaces::srv::AddProhibitedZone::Response> response)
{
  try {
    ProhibitedZone zone;
    zone.name = request->zone_name;

    if (request->zone_type == "polygon") {
      if (request->polygon_points.size() != 8) {
        response->success = false;
        response->message = "Polygon must have exactly 4 points (8 values: x1,y1,x2,y2,x3,y3,x4,y4)";
        return;
      }

      zone.type = ProhibitedZone::POLYGON;
      for (size_t i = 0; i < request->polygon_points.size(); i += 2) {
        zone.polygon_points.push_back({request->polygon_points[i], request->polygon_points[i + 1]});
      }
    } else {
      response->success = false;
      response->message = "Invalid zone_type. Use 'polygon' only";
      return;
    }

    // Check if zone already exists
    auto it = std::find_if(prohibited_zones_.begin(), prohibited_zones_.end(),
                           [&](const ProhibitedZone& z) { return z.name == zone.name; });
    
    if (it != prohibited_zones_.end()) {
      // Update existing zone
      *it = zone;
      response->message = "Zone '" + zone.name + "' updated";
    } else {
      // Add new zone
      prohibited_zones_.push_back(zone);
      response->message = "Zone '" + zone.name + "' added";
    }

    // Save to database
    saveToDatabase(zone);

    response->success = true;
    current_ = false;  // Mark costmap as needing update
    
    RCLCPP_INFO(logger_, "%s", response->message.c_str());
  } catch (const std::exception& e) {
    response->success = false;
    response->message = std::string("Error adding zone: ") + e.what();
    RCLCPP_ERROR(logger_, "%s", response->message.c_str());
  }
}

void ProhibitedLayer::handleRemoveZone(
  const std::shared_ptr<robot_interfaces::srv::RemoveProhibitedZone::Request> request,
  std::shared_ptr<robot_interfaces::srv::RemoveProhibitedZone::Response> response)
{
  auto it = std::find_if(prohibited_zones_.begin(), prohibited_zones_.end(),
                         [&](const ProhibitedZone& z) { return z.name == request->zone_name; });
  
  if (it != prohibited_zones_.end()) {
    prohibited_zones_.erase(it);
    
    // Delete from database
    deleteFromDatabase(request->zone_name);
    
    response->success = true;
    response->message = "Zone '" + request->zone_name + "' removed";
    current_ = false;  // Mark costmap as needing update
    RCLCPP_INFO(logger_, "%s", response->message.c_str());
  } else {
    response->success = false;
    response->message = "Zone '" + request->zone_name + "' not found";
    RCLCPP_WARN(logger_, "%s", response->message.c_str());
  }
}

void ProhibitedLayer::handleListZones(
  const std::shared_ptr<robot_interfaces::srv::ListProhibitedZones::Request> /*request*/,
  std::shared_ptr<robot_interfaces::srv::ListProhibitedZones::Response> response)
{
  std::string json = "{\"zones\": [";
  
  for (size_t i = 0; i < prohibited_zones_.size(); ++i) {
    const auto& zone = prohibited_zones_[i];
    response->zone_names.push_back(zone.name);
    response->zone_types.push_back("polygon");

    if (i > 0) json += ", ";
    json += "{\"name\": \"" + zone.name + "\", ";
    json += "\"type\": \"polygon\"";

    json += ", \"points\": [";
    for (size_t j = 0; j < zone.polygon_points.size(); ++j) {
      if (j > 0) json += ", ";
      json += std::to_string(zone.polygon_points[j].first) + ", " + 
              std::to_string(zone.polygon_points[j].second);
    }
    json += "]";
    json += "}";
  }
  
  json += "]}";
  response->zones_json = json;
}

}  // namespace robot_custom_layers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robot_custom_layers::ProhibitedLayer, nav2_costmap_2d::Layer)
