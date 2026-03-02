#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/robot_info.hpp>
#include <robot_interfaces/srv/save_prohibited_zone.hpp>
#include <robot_interfaces/srv/delete_prohibited_zone.hpp>
#include <robot_interfaces/srv/load_prohibited_zones.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/qos.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>
#include <mongocxx/options/replace.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/builder/stream/array.hpp>
#include <bsoncxx/json.hpp>

class RobotDatabaseNode : public rclcpp::Node {
public:
    RobotDatabaseNode() : Node("robot_database_node"), instance_{} {
        // Declare parameters - will read from robot_bringup namespace in robot_params.yaml
        this->declare_parameter<std::string>("robot_name", "default_robot");
        
        // Try to read robot_name from parameters
        if (!this->get_parameter("robot_name", robot_name_)) {
            robot_name_ = "default_robot";
            RCLCPP_WARN(this->get_logger(), "Could not read robot_name parameter, using default");
        }
        
        // Initialize MongoDB
        mongocxx::uri uri("mongodb://localhost:27017");
        client_ = mongocxx::client(uri);
        db_ = client_["robot_database"];
        collection_ = db_["robots_info"];
        collection_pose_ = db_["robots_pose"];
        collection_prohibited_zones_ = db_["prohibited_zones"];

        // Configure QoS for amcl_pose to match AMCL publisher
        rclcpp::QoS amcl_qos(10);
        amcl_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        amcl_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
        
        subscription_amcl_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose", amcl_qos, std::bind(&RobotDatabaseNode::amcl_pose_callback, this, std::placeholders::_1));

        // Create services for prohibited zones
        service_save_zone_ = this->create_service<robot_interfaces::srv::SaveProhibitedZone>(
            "save_prohibited_zone",
            std::bind(&RobotDatabaseNode::save_prohibited_zone_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        service_delete_zone_ = this->create_service<robot_interfaces::srv::DeleteProhibitedZone>(
            "delete_prohibited_zone",
            std::bind(&RobotDatabaseNode::delete_prohibited_zone_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        service_load_zones_ = this->create_service<robot_interfaces::srv::LoadProhibitedZones>(
            "load_prohibited_zones",
            std::bind(&RobotDatabaseNode::load_prohibited_zones_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Robot Database Node has been started");
        RCLCPP_INFO(this->get_logger(), "Robot name: %s", robot_name_.c_str());
    }

    ~RobotDatabaseNode() {
    }

private:
    // Callback for amcl_pose topic
    void amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        // Extract pose information from amcl_pose message
        auto doc = bsoncxx::builder::stream::document{}
            << "robot_name" << robot_name_
            << "header" << bsoncxx::builder::stream::open_document
                << "stamp" << bsoncxx::builder::stream::open_document
                    << "sec" << static_cast<int32_t>(msg->header.stamp.sec)
                    << "nanosec" << static_cast<int32_t>(msg->header.stamp.nanosec)
                << bsoncxx::builder::stream::close_document
                << "frame_id" << msg->header.frame_id
            << bsoncxx::builder::stream::close_document
            << "pose" << bsoncxx::builder::stream::open_document
                << "pose" << bsoncxx::builder::stream::open_document
                    << "position" << bsoncxx::builder::stream::open_document
                        << "x" << msg->pose.pose.position.x
                        << "y" << msg->pose.pose.position.y
                        << "z" << msg->pose.pose.position.z
                    << bsoncxx::builder::stream::close_document
                    << "orientation" << bsoncxx::builder::stream::open_document
                        << "x" << msg->pose.pose.orientation.x
                        << "y" << msg->pose.pose.orientation.y
                        << "z" << msg->pose.pose.orientation.z
                        << "w" << msg->pose.pose.orientation.w
                    << bsoncxx::builder::stream::close_document
                << bsoncxx::builder::stream::close_document
            << bsoncxx::builder::stream::close_document
            << "timestamp" << static_cast<int64_t>(this->get_clock()->now().nanoseconds())
            << bsoncxx::builder::stream::finalize;

        // Use robot_name as filter for upsert operation
        bsoncxx::builder::stream::document filter_builder;
        filter_builder << "robot_name" << robot_name_ << bsoncxx::builder::stream::finalize;

        mongocxx::options::replace opts;
        opts.upsert(true);

        auto result = collection_pose_.replace_one(filter_builder.view(), doc.view(), opts);
    }

    // Service callbacks for prohibited zones
    void save_prohibited_zone_callback(
        const std::shared_ptr<robot_interfaces::srv::SaveProhibitedZone::Request> request,
        std::shared_ptr<robot_interfaces::srv::SaveProhibitedZone::Response> response)
    {
        try {
            auto doc_builder = bsoncxx::builder::stream::document{};
            doc_builder << "robot_name" << request->robot_name
                       << "zone_name" << request->zone_name
                       << "zone_type" << request->zone_type
                       << "timestamp" << static_cast<int64_t>(this->get_clock()->now().nanoseconds());

            if (request->zone_type == "polygon") {
                bsoncxx::builder::stream::array points_array;
                for (const auto& point : request->polygon_points) {
                    points_array << point;
                }
                doc_builder << "polygon_points" << points_array;
            } else {
                // Invalid zone type
                response->success = false;
                response->message = "Invalid zone_type. Use 'polygon' only";
                RCLCPP_ERROR(this->get_logger(), "Invalid zone_type '%s' for zone '%s'", 
                           request->zone_type.c_str(), request->zone_name.c_str());
                return;
            }

            auto doc = doc_builder << bsoncxx::builder::stream::finalize;

            // Upsert: update if exists, insert if not
            bsoncxx::builder::stream::document filter;
            filter << "robot_name" << request->robot_name
                   << "zone_name" << request->zone_name
                   << bsoncxx::builder::stream::finalize;

            mongocxx::options::replace opts;
            opts.upsert(true);

            auto result = collection_prohibited_zones_.replace_one(filter.view(), doc.view(), opts);
            
            if (result) {
                response->success = true;
                if (result->upserted_id()) {
                    response->zone_id = result->upserted_id()->get_oid().value.to_string();
                    response->message = "Zone saved (new)";
                    RCLCPP_INFO(this->get_logger(), "Inserted new prohibited zone '%s' for robot '%s'",
                               request->zone_name.c_str(), request->robot_name.c_str());
                } else {
                    response->zone_id = "";
                    response->message = "Zone updated";
                    RCLCPP_INFO(this->get_logger(), "Updated prohibited zone '%s' for robot '%s'",
                               request->zone_name.c_str(), request->robot_name.c_str());
                }
            } else {
                response->success = false;
                response->message = "Database operation failed";
            }
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Error: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "Failed to save prohibited zone: %s", e.what());
        }
    }

    void delete_prohibited_zone_callback(
        const std::shared_ptr<robot_interfaces::srv::DeleteProhibitedZone::Request> request,
        std::shared_ptr<robot_interfaces::srv::DeleteProhibitedZone::Response> response)
    {
        try {
            bsoncxx::builder::stream::document filter;
            filter << "robot_name" << request->robot_name
                   << "zone_name" << request->zone_name
                   << bsoncxx::builder::stream::finalize;

            auto result = collection_prohibited_zones_.delete_one(filter.view());
            
            if (result && result->deleted_count() > 0) {
                response->success = true;
                response->message = "Zone deleted";
                RCLCPP_INFO(this->get_logger(), "Deleted prohibited zone '%s' for robot '%s'",
                           request->zone_name.c_str(), request->robot_name.c_str());
            } else {
                response->success = false;
                response->message = "Zone not found";
                RCLCPP_WARN(this->get_logger(), "Zone '%s' not found for robot '%s'",
                           request->zone_name.c_str(), request->robot_name.c_str());
            }
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Error: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "Failed to delete prohibited zone: %s", e.what());
        }
    }

    void load_prohibited_zones_callback(
        const std::shared_ptr<robot_interfaces::srv::LoadProhibitedZones::Request> request,
        std::shared_ptr<robot_interfaces::srv::LoadProhibitedZones::Response> response)
    {
        try {
            bsoncxx::builder::stream::document filter;
            filter << "robot_name" << request->robot_name
                   << bsoncxx::builder::stream::finalize;

            auto cursor = collection_prohibited_zones_.find(filter.view());
            
            std::string json = "{\"zones\": [";
            bool first = true;
            int count = 0;

            for (auto&& doc : cursor) {
                if (!first) json += ", ";
                first = false;
                count++;

                // Convert BSON document to JSON (simplified)
                json += "{";
                json += "\"zone_name\": \"" + std::string(doc["zone_name"].get_string().value) + "\", ";
                json += "\"zone_type\": \"" + std::string(doc["zone_type"].get_string().value) + "\"";

                if (doc["zone_type"].get_string().value == bsoncxx::stdx::string_view("polygon")) {
                    json += ", \"polygon_points\": [";
                    auto points = doc["polygon_points"].get_array().value;
                    bool first_point = true;
                    for (auto&& point : points) {
                        if (!first_point) json += ", ";
                        first_point = false;
                        json += std::to_string(point.get_double().value);
                    }
                    json += "]";
                }
                json += "}";
            }
            
            json += "]}";

            response->success = true;
            response->zones_json = json;
            response->message = "Loaded " + std::to_string(count) + " zones";
            RCLCPP_INFO(this->get_logger(), "Loaded %d prohibited zones for robot '%s'",
                       count, request->robot_name.c_str());
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Error: ") + e.what();
            response->zones_json = "{\"zones\": []}";
            RCLCPP_ERROR(this->get_logger(), "Failed to load prohibited zones: %s", e.what());
        }
    }

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_amcl_pose_;

    // Services
    rclcpp::Service<robot_interfaces::srv::SaveProhibitedZone>::SharedPtr service_save_zone_;
    rclcpp::Service<robot_interfaces::srv::DeleteProhibitedZone>::SharedPtr service_delete_zone_;
    rclcpp::Service<robot_interfaces::srv::LoadProhibitedZones>::SharedPtr service_load_zones_;

    // Robot name parameter
    std::string robot_name_;

    // MongoDB
    mongocxx::instance instance_;
    mongocxx::client client_;
    mongocxx::database db_;
    mongocxx::collection collection_;
    mongocxx::collection collection_pose_;
    mongocxx::collection collection_prohibited_zones_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotDatabaseNode>());
    rclcpp::shutdown();
    return 0;
}