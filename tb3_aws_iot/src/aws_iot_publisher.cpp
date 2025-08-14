#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#ifdef USE_SIMPLE_JSON
// Simple JSON generation without external library
#include <sstream>
#include <iomanip>
#else
#include <nlohmann/json.hpp>
#endif
#include <chrono>
#include <fstream>
#include <cmath>

class AwsIotPublisher : public rclcpp::Node
{
public:
  AwsIotPublisher()
  : Node("aws_iot_publisher")
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  {
    // Parameters
    this->declare_parameter("base_frame", "base_footprint");
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("publish_rate", 1.0);
    this->declare_parameter("aws_iot_endpoint", "");
    this->declare_parameter("thing_name", "turtlebot3");
    this->declare_parameter("output_file", "/tmp/aws_iot_data.json");
    
    base_frame_ = this->get_parameter("base_frame").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    double publish_rate = this->get_parameter("publish_rate").as_double();
    aws_iot_endpoint_ = this->get_parameter("aws_iot_endpoint").as_string();
    thing_name_ = this->get_parameter("thing_name").as_string();
    output_file_ = this->get_parameter("output_file").as_string();

    // Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&AwsIotPublisher::odom_callback, this, std::placeholders::_1));
      
    goal_markers_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/goal_markers", 10,
      std::bind(&AwsIotPublisher::goal_markers_callback, this, std::placeholders::_1));

    // Timer for periodic publishing
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
    timer_ = this->create_wall_timer(timer_period, 
      std::bind(&AwsIotPublisher::publish_to_aws_iot, this));

    RCLCPP_INFO(this->get_logger(), "AWS IoT Publisher initialized");
    RCLCPP_INFO(this->get_logger(), "Base frame: %s", base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "Map frame: %s", map_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "Thing name: %s", thing_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Output file: %s", output_file_.c_str());
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_ = msg;
    
    // Get robot position in map frame via TF2
    try {
      auto transform = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      
      robot_pose_in_map_.position.x = transform.transform.translation.x;
      robot_pose_in_map_.position.y = transform.transform.translation.y;
      robot_pose_in_map_.position.z = transform.transform.translation.z;
      robot_pose_in_map_.orientation = transform.transform.rotation;
      
      robot_pose_valid_ = true;
      
      RCLCPP_DEBUG(this->get_logger(), 
        "Robot position in map: (%.3f, %.3f, %.3f)", 
        robot_pose_in_map_.position.x, 
        robot_pose_in_map_.position.y,
        tf2::getYaw(robot_pose_in_map_.orientation));
        
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Could not transform from %s to %s: %s", 
        base_frame_.c_str(), map_frame_.c_str(), ex.what());
      robot_pose_valid_ = false;
    }
  }

  void goal_markers_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    for (const auto& marker : msg->markers) {
      // Look for goal position marker (type 0 = ARROW, ns = "goal_position")
      if (marker.type == visualization_msgs::msg::Marker::ARROW && 
          marker.ns == "goal_position") {
        goal_pose_in_map_.position = marker.pose.position;
        goal_pose_in_map_.orientation = marker.pose.orientation;
        goal_pose_valid_ = true;
        
        RCLCPP_INFO(this->get_logger(), 
          "Goal updated in map: (%.3f, %.3f, %.3f)", 
          goal_pose_in_map_.position.x, 
          goal_pose_in_map_.position.y,
          tf2::getYaw(goal_pose_in_map_.orientation));
        break; // Found the goal marker, no need to continue
      }
    }
  }

  void publish_to_aws_iot()
  {
    if (!robot_pose_valid_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
        "Robot pose not available, skipping AWS IoT publish");
      return;
    }

    std::string json_str = create_json_payload();

    // For demonstration, write to file (in real implementation, send to AWS IoT)
    write_to_file(json_str);
    
    // Log periodically
    static int log_counter = 0;
    if (++log_counter % 30 == 0) {  // Every 30 messages
      RCLCPP_INFO(this->get_logger(), 
        "Published to AWS IoT - Robot: (%.2f, %.2f, %.1f°)%s",
        robot_pose_in_map_.position.x,
        robot_pose_in_map_.position.y,
        tf2::getYaw(robot_pose_in_map_.orientation) * 180.0 / M_PI,
        goal_pose_valid_ ? 
          (", Goal: (" + 
           std::to_string(goal_pose_in_map_.position.x) + ", " +
           std::to_string(goal_pose_in_map_.position.y) + ")").c_str() : 
          ", No goal");
    }
  }

#ifdef USE_SIMPLE_JSON
  std::string create_json_payload()
  {
    std::ostringstream json;
    json << std::fixed << std::setprecision(6);
    
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
      
    json << "{\n";
    json << "  \"timestamp\": " << timestamp << ",\n";
    json << "  \"thing_name\": \"" << thing_name_ << "\",\n";
    
    // Robot pose
    json << "  \"robot_pose\": {\n";
    json << "    \"position\": {\n";
    json << "      \"x\": " << robot_pose_in_map_.position.x << ",\n";
    json << "      \"y\": " << robot_pose_in_map_.position.y << ",\n";
    json << "      \"z\": " << robot_pose_in_map_.position.z << "\n";
    json << "    },\n";
    json << "    \"orientation\": {\n";
    json << "      \"x\": " << robot_pose_in_map_.orientation.x << ",\n";
    json << "      \"y\": " << robot_pose_in_map_.orientation.y << ",\n";
    json << "      \"z\": " << robot_pose_in_map_.orientation.z << ",\n";
    json << "      \"w\": " << robot_pose_in_map_.orientation.w << "\n";
    json << "    },\n";
    json << "    \"yaw\": " << tf2::getYaw(robot_pose_in_map_.orientation) << "\n";
    json << "  },\n";
    
    // Goal pose
    if (goal_pose_valid_) {
      json << "  \"goal_pose\": {\n";
      json << "    \"position\": {\n";
      json << "      \"x\": " << goal_pose_in_map_.position.x << ",\n";
      json << "      \"y\": " << goal_pose_in_map_.position.y << ",\n";
      json << "      \"z\": " << goal_pose_in_map_.position.z << "\n";
      json << "    },\n";
      json << "    \"orientation\": {\n";
      json << "      \"x\": " << goal_pose_in_map_.orientation.x << ",\n";
      json << "      \"y\": " << goal_pose_in_map_.orientation.y << ",\n";
      json << "      \"z\": " << goal_pose_in_map_.orientation.z << ",\n";
      json << "      \"w\": " << goal_pose_in_map_.orientation.w << "\n";
      json << "    },\n";
      json << "    \"yaw\": " << tf2::getYaw(goal_pose_in_map_.orientation) << "\n";
      json << "  },\n";
      
      double dx = goal_pose_in_map_.position.x - robot_pose_in_map_.position.x;
      double dy = goal_pose_in_map_.position.y - robot_pose_in_map_.position.y;
      json << "  \"distance_to_goal\": " << std::sqrt(dx*dx + dy*dy) << ",\n";
    } else {
      json << "  \"goal_pose\": null,\n";
      json << "  \"distance_to_goal\": null,\n";
    }
    
    // Odometry
    if (last_odom_) {
      json << "  \"odometry\": {\n";
      json << "    \"linear_velocity\": {\n";
      json << "      \"x\": " << last_odom_->twist.twist.linear.x << ",\n";
      json << "      \"y\": " << last_odom_->twist.twist.linear.y << "\n";
      json << "    },\n";
      json << "    \"angular_velocity\": {\n";
      json << "      \"z\": " << last_odom_->twist.twist.angular.z << "\n";
      json << "    }\n";
      json << "  }\n";
    } else {
      json << "  \"odometry\": null\n";
    }
    
    json << "}";
    return json.str();
  }
#else
  std::string create_json_payload()
  {
    // Create JSON payload with nlohmann::json
    nlohmann::json payload;
    payload["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    payload["thing_name"] = thing_name_;

    // Robot pose in map coordinates
    payload["robot_pose"]["position"]["x"] = robot_pose_in_map_.position.x;
    payload["robot_pose"]["position"]["y"] = robot_pose_in_map_.position.y;
    payload["robot_pose"]["position"]["z"] = robot_pose_in_map_.position.z;
    payload["robot_pose"]["orientation"]["x"] = robot_pose_in_map_.orientation.x;
    payload["robot_pose"]["orientation"]["y"] = robot_pose_in_map_.orientation.y;
    payload["robot_pose"]["orientation"]["z"] = robot_pose_in_map_.orientation.z;
    payload["robot_pose"]["orientation"]["w"] = robot_pose_in_map_.orientation.w;
    payload["robot_pose"]["yaw"] = tf2::getYaw(robot_pose_in_map_.orientation);

    // Goal pose in map coordinates (if available)
    if (goal_pose_valid_) {
      payload["goal_pose"]["position"]["x"] = goal_pose_in_map_.position.x;
      payload["goal_pose"]["position"]["y"] = goal_pose_in_map_.position.y;
      payload["goal_pose"]["position"]["z"] = goal_pose_in_map_.position.z;
      payload["goal_pose"]["orientation"]["x"] = goal_pose_in_map_.orientation.x;
      payload["goal_pose"]["orientation"]["y"] = goal_pose_in_map_.orientation.y;
      payload["goal_pose"]["orientation"]["z"] = goal_pose_in_map_.orientation.z;
      payload["goal_pose"]["orientation"]["w"] = goal_pose_in_map_.orientation.w;
      payload["goal_pose"]["yaw"] = tf2::getYaw(goal_pose_in_map_.orientation);
      
      // Calculate distance to goal
      double dx = goal_pose_in_map_.position.x - robot_pose_in_map_.position.x;
      double dy = goal_pose_in_map_.position.y - robot_pose_in_map_.position.y;
      payload["distance_to_goal"] = std::sqrt(dx*dx + dy*dy);
    } else {
      payload["goal_pose"] = nullptr;
      payload["distance_to_goal"] = nullptr;
    }

    // Odometry data (if available)
    if (last_odom_) {
      payload["odometry"]["linear_velocity"]["x"] = last_odom_->twist.twist.linear.x;
      payload["odometry"]["linear_velocity"]["y"] = last_odom_->twist.twist.linear.y;
      payload["odometry"]["angular_velocity"]["z"] = last_odom_->twist.twist.angular.z;
    }

    return payload.dump(2);
  }
#endif

  void write_to_file(const std::string& json_str)
  {
    try {
      std::ofstream file(output_file_);
      if (file.is_open()) {
        file << json_str << std::endl;
        file.close();
      } else {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
          "Failed to open output file: %s", output_file_.c_str());
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
        "Error writing to file: %s", e.what());
    }
  }

  // ROS components
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr goal_markers_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  std::string base_frame_;
  std::string map_frame_;
  std::string aws_iot_endpoint_;
  std::string thing_name_;
  std::string output_file_;

  // State
  geometry_msgs::msg::Pose robot_pose_in_map_;
  geometry_msgs::msg::Pose goal_pose_in_map_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  bool robot_pose_valid_ = false;
  bool goal_pose_valid_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AwsIotPublisher>());
  rclcpp::shutdown();
  return 0;
}