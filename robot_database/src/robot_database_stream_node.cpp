#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/srv/add_prohibited_zone.hpp>
#include <robot_interfaces/srv/remove_prohibited_zone.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>
#include <mongocxx/change_stream.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/builder/stream/array.hpp>
#include <bsoncxx/json.hpp>
#include <thread>
#include <atomic>

class RobotDatabaseStreamNode : public rclcpp::Node {
public:
    RobotDatabaseStreamNode() : Node("robot_database_stream_node"), instance_{} {
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
        collection_prohibited_zones_ = db_["prohibited_zones"];

        // Create service clients to notify ProhibitedLayer
        add_zone_client_ = this->create_client<robot_interfaces::srv::AddProhibitedZone>(
            "/global_costmap/global_costmap/add_prohibited_zone");
        
        remove_zone_client_ = this->create_client<robot_interfaces::srv::RemoveProhibitedZone>(
            "/global_costmap/global_costmap/remove_prohibited_zone");

        RCLCPP_INFO(this->get_logger(), "Robot Database Stream Node has been started");
        RCLCPP_INFO(this->get_logger(), "Robot name: %s", robot_name_.c_str());

        // Load existing zones from database on startup
        loadExistingZonesFromDatabase();

        // Start MongoDB Change Stream watcher in separate thread
        watch_thread_running_ = true;
        watch_thread_ = std::thread(&RobotDatabaseStreamNode::watchProhibitedZonesCollection, this);

        RCLCPP_INFO(this->get_logger(), "MongoDB Change Stream watcher started for prohibited_zones");
    }

    ~RobotDatabaseStreamNode() {
        // Stop watch thread
        watch_thread_running_ = false;
        if (watch_thread_.joinable()) {
            watch_thread_.join();
        }
    }

private:
    // Load existing zones from database on startup
    void loadExistingZonesFromDatabase() {
        RCLCPP_INFO(this->get_logger(), "Loading existing prohibited zones from database...");
        
        try {
            // Query all zones for this robot
            bsoncxx::builder::stream::document filter;
            filter << "robot_name" << robot_name_
                   << bsoncxx::builder::stream::finalize;

            auto cursor = collection_prohibited_zones_.find(filter.view());
            int count = 0;

            for (auto&& doc : cursor) {
                std::string zone_name = std::string(doc["zone_name"].get_string().value);
                std::string zone_type = std::string(doc["zone_type"].get_string().value);
                
                RCLCPP_INFO(this->get_logger(), "Loading zone: %s (type: %s)", 
                           zone_name.c_str(), zone_type.c_str());

                // Create request for add_prohibited_zone service
                auto request = std::make_shared<robot_interfaces::srv::AddProhibitedZone::Request>();
                request->zone_name = zone_name;
                request->zone_type = zone_type;
                
                if (zone_type == "polygon") {
                    auto points = doc["polygon_points"].get_array().value;
                    for (auto&& point : points) {
                        // Handle both int32 and double types
                        if (point.type() == bsoncxx::type::k_double) {
                            request->polygon_points.push_back(point.get_double().value);
                        } else if (point.type() == bsoncxx::type::k_int32) {
                            request->polygon_points.push_back(static_cast<double>(point.get_int32().value));
                        } else if (point.type() == bsoncxx::type::k_int64) {
                            request->polygon_points.push_back(static_cast<double>(point.get_int64().value));
                        }
                    }
                }
                
                // Wait for service to be available (with timeout)
                if (add_zone_client_->wait_for_service(std::chrono::seconds(5))) {
                    auto future = add_zone_client_->async_send_request(request);
                    count++;
                    RCLCPP_INFO(this->get_logger(), 
                               "✅ Added zone '%s' to costmap", zone_name.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), 
                               "❌ add_prohibited_zone service not available! Skipping zone '%s'",
                               zone_name.c_str());
                }
                
                // Small delay between service calls
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            RCLCPP_INFO(this->get_logger(), 
                       "Loaded %d existing prohibited zones from database", count);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), 
                        "Failed to load existing zones: %s", e.what());
        }
    }

    // MongoDB Change Stream Watcher
    void watchProhibitedZonesCollection() {
        RCLCPP_INFO(this->get_logger(), "Starting MongoDB change stream watcher...");
        
        try {
            // Create NEW MongoDB connection for this thread (thread safety)
            mongocxx::uri uri("mongodb://localhost:27017");
            mongocxx::client thread_client(uri);
            auto thread_db = thread_client["robot_database"];
            auto thread_collection = thread_db["prohibited_zones"];
            
            // Watch for changes with timeout
            mongocxx::options::change_stream options{};
            options.full_document("updateLookup");
            
            // Enable preImage for delete operations (requires MongoDB 6.0+)
            // This allows us to see the document before it was deleted
            // Note: Collection must be configured with changeStreamPreAndPostImages enabled
            bsoncxx::builder::stream::document pipeline_builder;
            auto pipeline = bsoncxx::builder::stream::array{} << bsoncxx::builder::stream::finalize;
            
            options.max_await_time(std::chrono::milliseconds(1000)); // 1 second timeout
            
            auto stream = thread_collection.watch(options);
            
            RCLCPP_INFO(this->get_logger(), "Change stream established, watching for changes...");
            
            while (watch_thread_running_ && rclcpp::ok()) {
                try {
                    // Get iterator and advance it (don't keep calling begin())
                    for (auto&& event : stream) {
                        
                        auto operation_type = event["operationType"].get_string().value;
                        
                        RCLCPP_INFO(this->get_logger(), "MongoDB change detected: %s", 
                                   std::string(operation_type).c_str());
                        
                        // Handle INSERT or UPDATE
                        if (operation_type == bsoncxx::stdx::string_view("insert") || 
                            operation_type == bsoncxx::stdx::string_view("update") ||
                            operation_type == bsoncxx::stdx::string_view("replace")) {
                            
                            auto full_doc = event["fullDocument"].get_document().value;
                            
                            // Check if it's for our robot
                            std::string doc_robot_name = std::string(full_doc["robot_name"].get_string().value);
                            
                            RCLCPP_INFO(this->get_logger(), "Zone change for robot: %s (my robot: %s)",
                                       doc_robot_name.c_str(), robot_name_.c_str());
                            
                            if (doc_robot_name != robot_name_) {
                                RCLCPP_INFO(this->get_logger(), "Skipping - not for this robot");
                                continue;
                            }
                            
                            // Call add_prohibited_zone service
                            auto request = std::make_shared<robot_interfaces::srv::AddProhibitedZone::Request>();
                            request->zone_name = std::string(full_doc["zone_name"].get_string().value);
                            request->zone_type = std::string(full_doc["zone_type"].get_string().value);
                            
                            if (request->zone_type == "polygon") {
                                auto points = full_doc["polygon_points"].get_array().value;
                                for (auto&& point : points) {
                                    // Handle both int32 and double types
                                    if (point.type() == bsoncxx::type::k_double) {
                                        request->polygon_points.push_back(point.get_double().value);
                                    } else if (point.type() == bsoncxx::type::k_int32) {
                                        request->polygon_points.push_back(static_cast<double>(point.get_int32().value));
                                    } else if (point.type() == bsoncxx::type::k_int64) {
                                        request->polygon_points.push_back(static_cast<double>(point.get_int64().value));
                                    }
                                }
                            }
                            
                            // Wait for service to be available
                            RCLCPP_INFO(this->get_logger(), 
                                       "Calling add_prohibited_zone service for '%s'...",
                                       request->zone_name.c_str());
                            
                            if (add_zone_client_->wait_for_service(std::chrono::seconds(2))) {
                                auto future = add_zone_client_->async_send_request(request);
                                RCLCPP_INFO(this->get_logger(), 
                                           "✅ Service called successfully for zone '%s'", 
                                           request->zone_name.c_str());
                            } else {
                                RCLCPP_WARN(this->get_logger(), 
                                           "❌ add_prohibited_zone service not available!");
                            }
                        }
                        
                        // Handle DELETE
                        else if (operation_type == bsoncxx::stdx::string_view("delete")) {
                            RCLCPP_INFO(this->get_logger(), "Zone delete detected from database");
                            
                            // Try to get fullDocumentBeforeChange (requires preImage configuration)
                            if (event["fullDocumentBeforeChange"]) {
                                auto deleted_doc = event["fullDocumentBeforeChange"].get_document().value;
                                
                                std::string doc_robot_name = std::string(deleted_doc["robot_name"].get_string().value);
                                std::string zone_name = std::string(deleted_doc["zone_name"].get_string().value);
                                
                                RCLCPP_INFO(this->get_logger(), "Delete zone '%s' for robot: %s (my robot: %s)",
                                           zone_name.c_str(), doc_robot_name.c_str(), robot_name_.c_str());
                                
                                if (doc_robot_name != robot_name_) {
                                    RCLCPP_INFO(this->get_logger(), "Skipping - not for this robot");
                                    continue;
                                }
                                
                                // Call remove_prohibited_zone service
                                auto request = std::make_shared<robot_interfaces::srv::RemoveProhibitedZone::Request>();
                                request->zone_name = zone_name;
                                
                                RCLCPP_INFO(this->get_logger(), 
                                           "Calling remove_prohibited_zone service for '%s'...",
                                           zone_name.c_str());
                                
                                if (remove_zone_client_->wait_for_service(std::chrono::seconds(2))) {
                                    auto future = remove_zone_client_->async_send_request(request);
                                    RCLCPP_INFO(this->get_logger(), 
                                               "✅ Remove service called successfully for zone '%s'", 
                                               zone_name.c_str());
                                } else {
                                    RCLCPP_WARN(this->get_logger(), 
                                               "❌ remove_prohibited_zone service not available!");
                                }
                            } else {
                                RCLCPP_WARN(this->get_logger(), 
                                           "⚠️ Delete detected but fullDocumentBeforeChange not available. "
                                           "Enable changeStreamPreAndPostImages on collection.");
                            }
                        }
                    }
                    
                    // Small sleep to prevent tight loop when no events
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Change stream iteration error: %s", e.what());
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start change stream: %s", e.what());
        }
        
        RCLCPP_INFO(this->get_logger(), "Change stream watcher stopped");
    }

    // Service clients
    rclcpp::Client<robot_interfaces::srv::AddProhibitedZone>::SharedPtr add_zone_client_;
    rclcpp::Client<robot_interfaces::srv::RemoveProhibitedZone>::SharedPtr remove_zone_client_;

    // Change stream thread
    std::thread watch_thread_;
    std::atomic<bool> watch_thread_running_;

    // Robot name parameter
    std::string robot_name_;

    // MongoDB
    mongocxx::instance instance_;
    mongocxx::client client_;
    mongocxx::database db_;
    mongocxx::collection collection_prohibited_zones_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotDatabaseStreamNode>());
    rclcpp::shutdown();
    return 0;
}
